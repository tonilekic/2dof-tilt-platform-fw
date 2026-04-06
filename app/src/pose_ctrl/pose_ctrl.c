/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/zbus/zbus.h>

#include "app/zbus_channels.h"
#include "imu.h"
#include "kinematics.h"
#include "motion.h"
#include "pose_ctrl.h"

LOG_MODULE_REGISTER(pose_ctrl, LOG_LEVEL_INF);

int pose_ctrl_init(void)
{
	return 0;
}

int pose_ctrl_set(const struct shell *sh, float tilt_deg, float azimuth_deg)
{
	const int32_t probe = CONFIG_TT_HOMING_PROBE_STEPS;
	const float converge_thresh =
		(float)CONFIG_TT_HOMING_CONVERGE_THRESH_MILLI_MS2 / 1000.0f;
	const float fast_thresh = converge_thresh;
	const int max_iter = CONFIG_TT_HOMING_MAX_ITERATIONS;
	const k_timeout_t move_to = K_SECONDS(20);
	const float max_tilt_deg = 60.0f;
	const float g_ms2 = 9.80665f;
	float a0[3], a1[3];
	float jcol[2][2];
	float det;
	int err;

	if (tilt_deg < 0.0f || tilt_deg > max_tilt_deg) {
		shell_error(sh, "Tilt %.2f° out of range [0, %.1f]",
			    (double)tilt_deg, (double)max_tilt_deg);
		return -EINVAL;
	}

	float t = tilt_deg * KIN_RAD_PER_DEG;
	float p = azimuth_deg * KIN_RAD_PER_DEG;
	float a_target_full[3];

	kin_target_accel(t, p, g_ms2, a_target_full);

	float a_y_target = a_target_full[1];
	float a_z_target = a_target_full[2];

	shell_print(sh,
		    "Target: θ=%.2f° φ=%.2f° (a_y=%+.3f a_z=%+.3f m/s²)",
		    (double)tilt_deg, (double)azimuth_deg,
		    (double)a_y_target, (double)a_z_target);

	/* Read homing state via zbus channel — channel acts as a state cache. */
	struct homing_state hs;
	int rc = zbus_chan_read(&chan_homing_state, &hs, K_MSEC(10));

	if (rc == 0 && hs.homed) {
		float c1 = -hs.jac[0][1] / g_ms2;
		float c2 = -hs.jac[1][0] / g_ms2;

		if (fabsf(c1) < 1.0e-7f || fabsf(c2) < 1.0e-7f) {
			shell_warn(sh,
				   "Cached Jacobian degenerate (c1=%+.3g c2=%+.3g) "
				   "— falling through to Newton",
				   (double)c1, (double)c2);
			goto newton_path;
		}

		float alpha1_target, alpha2_target;

		kin_alpha_from_target(t, p, &alpha1_target, &alpha2_target);

		int32_t s1 = (int32_t)(alpha1_target / c1);
		int32_t s2 = (int32_t)(alpha2_target / c2);

		shell_print(sh,
			    "Fast: α=(%.2f°, %.2f°) → move_to motor1=%d motor2=%d",
			    (double)(alpha1_target * KIN_DEG_PER_RAD),
			    (double)(alpha2_target * KIN_DEG_PER_RAD), s1, s2);

		err = motion_pair_to(s1, s2, move_to);
		if (err) {
			goto fail;
		}

		float a[3];
		float tilt_now, az_now;

		err = imu_read_accel_avg(a, 32);
		if (err) {
			goto fail;
		}
		kin_compute_tilt(a, &tilt_now, &az_now);

		float ey = a[1] - a_y_target;
		float ez = a[2] - a_z_target;
		float lat_err = sqrtf(ey * ey + ez * ez);

		shell_print(sh, "Fast: θ=%.2f° φ=%.2f° |err|=%.3f m/s²",
			    (double)tilt_now, (double)az_now,
			    (double)lat_err);

		if (lat_err < fast_thresh) {
			shell_print(sh,
				    "Final: θ=%.2f° (target %.2f°, Δ=%+.2f°) "
				    "φ=%.2f° (target %.2f°, Δ=%+.2f°)",
				    (double)tilt_now, (double)tilt_deg,
				    (double)(tilt_now - tilt_deg),
				    (double)az_now, (double)azimuth_deg,
				    (double)(az_now - azimuth_deg));
			return 0;
		}

		shell_print(sh,
			    "  |err|=%.3f > %.3f m/s² — refining with Newton",
			    (double)lat_err, (double)fast_thresh);
	}

newton_path:
	shell_print(sh, "Probing Jacobian (%d steps per axis)...", probe);

	err = imu_read_accel_avg(a0, 16);
	if (err) {
		goto fail;
	}
	err = motion_one_by(MOTION_MOTOR1, probe, K_SECONDS(10));
	if (err) {
		goto fail;
	}
	err = imu_read_accel_avg(a1, 16);
	if (err) {
		goto fail;
	}
	jcol[0][0] = (a1[1] - a0[1]) / (float)probe;
	jcol[0][1] = (a1[2] - a0[2]) / (float)probe;
	err = motion_one_by(MOTION_MOTOR1, -probe, K_SECONDS(10));
	if (err) {
		goto fail;
	}

	err = imu_read_accel_avg(a0, 16);
	if (err) {
		goto fail;
	}
	err = motion_one_by(MOTION_MOTOR2, probe, K_SECONDS(10));
	if (err) {
		goto fail;
	}
	err = imu_read_accel_avg(a1, 16);
	if (err) {
		goto fail;
	}
	jcol[1][0] = (a1[1] - a0[1]) / (float)probe;
	jcol[1][1] = (a1[2] - a0[2]) / (float)probe;
	err = motion_one_by(MOTION_MOTOR2, -probe, K_SECONDS(10));
	if (err) {
		goto fail;
	}

	det = jcol[0][0] * jcol[1][1] - jcol[1][0] * jcol[0][1];
	if (fabsf(det) < 1.0e-5f) {
		shell_error(sh, "Jacobian singular (det=%.3g)", (double)det);
		return -EIO;
	}

	for (int it = 0; it < max_iter; it++) {
		float a[3];
		float tilt_now, az_now;

		err = imu_read_accel_avg(a, 32);
		if (err) {
			goto fail;
		}

		float ey = a[1] - a_y_target;
		float ez = a[2] - a_z_target;
		float lat_err = sqrtf(ey * ey + ez * ez);

		kin_compute_tilt(a, &tilt_now, &az_now);
		shell_print(sh,
			    "iter %d: θ=%.2f° φ=%.2f° |err|=%.3f m/s²",
			    it, (double)tilt_now, (double)az_now,
			    (double)lat_err);

		if (lat_err < converge_thresh) {
			break;
		}

		float dy = -ey, dz = -ez;
		float d1 = (jcol[1][1] * dy - jcol[1][0] * dz) / det;
		float d2 = (-jcol[0][1] * dy + jcol[0][0] * dz) / det;

		err = motion_pair_by((int32_t)d1, (int32_t)d2, move_to);
		if (err) {
			goto fail;
		}
	}

	float a_final[3];
	float tilt_final, az_final;

	err = imu_read_accel_avg(a_final, 32);
	if (err) {
		goto fail;
	}
	kin_compute_tilt(a_final, &tilt_final, &az_final);
	shell_print(sh,
		    "Final: θ=%.2f° (target %.2f°, Δ=%+.2f°) "
		    "φ=%.2f° (target %.2f°, Δ=%+.2f°)",
		    (double)tilt_final, (double)tilt_deg,
		    (double)(tilt_final - tilt_deg),
		    (double)az_final, (double)azimuth_deg,
		    (double)(az_final - azimuth_deg));
	return 0;

fail:
	shell_error(sh, "set failed: %d", err);
	return err;
}
