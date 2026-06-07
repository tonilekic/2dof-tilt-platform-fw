/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "calib.h"
#include "imu.h"

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

/* Standard gravity. The fit absorbs sensitivity error, so this only sets the
 * output scale (corrected magnitude at rest). */
#define IMU_G 9.80665f

/* Settings: blob stored under "imu/cal". Bump the magic if the layout
 * changes so a stale blob from an older firmware is ignored on load. */
#define CALIB_SETTINGS_ROOT "imu"
#define CALIB_SETTINGS_KEY  "imu/cal"
#define CALIB_BLOB_MAGIC    0x434C4231u /* "CLB1" */

struct calib_blob {
	uint32_t magic;
	float A[3][4];
	float temp_c; /* die temp at solve time, NAN if unknown */
};

static const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));

/* Active map: valid=false means pass-through (uncalibrated), which reproduces
 * the pre-calibration behaviour exactly. */
static struct calib active = { .valid = false };
static float active_temp_c = NAN;

/* Calibration session: raw captures kept per-pose so a pose can be re-taken,
 * and the fit is (re)built fresh on solve. */
static float session_raw[IMU_CALIB_POSE_COUNT][3];
static bool session_have[IMU_CALIB_POSE_COUNT];

static const float pose_target[IMU_CALIB_POSE_COUNT][3] = {
	[IMU_CALIB_X_UP] = { +IMU_G, 0.0f, 0.0f },
	[IMU_CALIB_X_DOWN] = { -IMU_G, 0.0f, 0.0f },
	[IMU_CALIB_Y_UP] = { 0.0f, +IMU_G, 0.0f },
	[IMU_CALIB_Y_DOWN] = { 0.0f, -IMU_G, 0.0f },
	[IMU_CALIB_Z_UP] = { 0.0f, 0.0f, +IMU_G },
	[IMU_CALIB_Z_DOWN] = { 0.0f, 0.0f, -IMU_G },
};

/* --- persistence ------------------------------------------------------- */

static int calib_settings_set(const char *name, size_t len,
			      settings_read_cb read_cb, void *cb_arg)
{
	const char *next;

	if (settings_name_steq(name, "cal", &next) && !next) {
		struct calib_blob blob;

		if (len != sizeof(blob)) {
			LOG_WRN("calib blob size mismatch (%zu); ignoring", len);
			return -EINVAL;
		}
		if (read_cb(cb_arg, &blob, sizeof(blob)) != (ssize_t)sizeof(blob)) {
			return -EIO;
		}
		if (blob.magic != CALIB_BLOB_MAGIC) {
			LOG_WRN("calib blob magic mismatch; ignoring");
			return -EINVAL;
		}
		memcpy(active.A, blob.A, sizeof(active.A));
		active.valid = true;
		active_temp_c = blob.temp_c;
		LOG_INF("loaded stored calibration");
		return 0;
	}
	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(imu_calib, CALIB_SETTINGS_ROOT, NULL,
			       calib_settings_set, NULL, NULL);

static int calib_persist(const struct calib *c, float temp_c)
{
	struct calib_blob blob = { .magic = CALIB_BLOB_MAGIC, .temp_c = temp_c };

	memcpy(blob.A, c->A, sizeof(blob.A));
	return settings_save_one(CALIB_SETTINGS_KEY, &blob, sizeof(blob));
}

/* --- init -------------------------------------------------------------- */

int imu_init(void)
{
	int err;

	if (!device_is_ready(imu_dev)) {
		LOG_ERR("%s not ready", imu_dev->name);
		return -ENODEV;
	}

	err = settings_subsys_init();
	if (err) {
		LOG_WRN("settings init failed (%d); calibration won't persist",
			err);
	} else {
		settings_load_subtree(CALIB_SETTINGS_ROOT);
	}

	LOG_INF("%s ready (%s)", imu_dev->name,
		active.valid ? "calibrated" : "uncalibrated");
	return 0;
}

/* --- reads ------------------------------------------------------------- */

int imu_read_accel_raw(float a_out[3], unsigned int n)
{
	float sum[3] = { 0.0f, 0.0f, 0.0f };
	struct sensor_value v[3];
	int err;

	if (n == 0) {
		return -EINVAL;
	}

	for (unsigned int i = 0; i < n; i++) {
		err = sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ);
		if (err) {
			return err;
		}
		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &v[0]);
		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &v[1]);
		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &v[2]);
		sum[0] += sensor_value_to_float(&v[0]);
		sum[1] += sensor_value_to_float(&v[1]);
		sum[2] += sensor_value_to_float(&v[2]);
		k_msleep(10); /* matches the 104 Hz ODR with margin */
	}

	a_out[0] = sum[0] / (float)n;
	a_out[1] = sum[1] / (float)n;
	a_out[2] = sum[2] / (float)n;
	return 0;
}

int imu_read_accel_avg(float a_out[3], unsigned int n)
{
	int err = imu_read_accel_raw(a_out, n);

	if (err) {
		return err;
	}
	calib_apply(&active, a_out, a_out); /* in-place safe */
	return 0;
}

float imu_read_temp(void)
{
	struct sensor_value t;

	if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_DIE_TEMP)) {
		return NAN;
	}
	if (sensor_channel_get(imu_dev, SENSOR_CHAN_DIE_TEMP, &t)) {
		return NAN;
	}
	return sensor_value_to_float(&t);
}

/* --- calibration session ----------------------------------------------- */

void imu_calib_session_reset(void)
{
	memset(session_have, 0, sizeof(session_have));
}

int imu_calib_capture(enum imu_calib_pose pose, float raw_out[3],
		      unsigned int *n_have)
{
	float raw[3];
	int err;

	if (pose >= IMU_CALIB_POSE_COUNT) {
		return -EINVAL;
	}

	err = imu_read_accel_raw(raw, 64);
	if (err) {
		return err;
	}

	memcpy(session_raw[pose], raw, sizeof(raw));
	session_have[pose] = true;

	if (raw_out != NULL) {
		memcpy(raw_out, raw, sizeof(raw));
	}
	if (n_have != NULL) {
		unsigned int c = 0;

		for (int p = 0; p < IMU_CALIB_POSE_COUNT; p++) {
			c += session_have[p] ? 1 : 0;
		}
		*n_have = c;
	}
	return 0;
}

int imu_calib_solve_and_save(float *resid_rms)
{
	struct calib_fit f;
	struct calib c;
	int err;

	calib_fit_reset(&f);
	for (int p = 0; p < IMU_CALIB_POSE_COUNT; p++) {
		if (session_have[p]) {
			calib_fit_add(&f, session_raw[p], pose_target[p]);
		}
	}

	err = calib_fit_solve(&f, &c, resid_rms);
	if (err) {
		return err;
	}

	/* Install immediately so the result is live even if flash write fails. */
	active = c;
	active_temp_c = imu_read_temp();

	err = calib_persist(&c, active_temp_c);
	if (err) {
		LOG_WRN("calibration installed but not persisted (%d)", err);
		return err;
	}
	return 0;
}

void imu_calib_get(struct calib *out)
{
	*out = active;
}

int imu_calib_clear(void)
{
	active.valid = false;
	active_temp_c = NAN;
	imu_calib_session_reset();
	return settings_delete(CALIB_SETTINGS_KEY);
}
