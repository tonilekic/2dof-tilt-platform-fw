/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "imu.h"

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

static const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));

int imu_init(void)
{
	if (!device_is_ready(imu_dev)) {
		LOG_ERR("%s not ready", imu_dev->name);
		return -ENODEV;
	}
	LOG_INF("%s ready", imu_dev->name);
	return 0;
}

int imu_read_accel_avg(float a_out[3], unsigned int n)
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
