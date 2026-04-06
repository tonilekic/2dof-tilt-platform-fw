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
#include "homing.h"
#include "imu.h"
#include "kinematics.h"
#include "motion.h"

LOG_MODULE_REGISTER(homing, LOG_LEVEL_INF);

/* Half a revolution at 1/64 microstepping: 200 fullsteps × 64 / 2. */
#define HALF_REV_MICROSTEPS 6400

static struct homing_state cached_state = {
	.homed = false,
};

static int publish_state(void)
{
	return zbus_chan_pub(&chan_homing_state, &cached_state, K_MSEC(50));
}

int homing_init(void)
{
	cached_state.homed = false;
	publish_state();
	return 0;
}

bool homing_is_homed(void)
{
	return cached_state.homed;
}

int homing_get_level_jacobian(float jac[2][2], float *det)
{
	if (!cached_state.homed) {
		return -EAGAIN;
	}
	jac[0][0] = cached_state.jac[0][0];
	jac[0][1] = cached_state.jac[0][1];
	jac[1][0] = cached_state.jac[1][0];
	jac[1][1] = cached_state.jac[1][1];
	*det = cached_state.det;
	return 0;
}

void homing_invalidate(void)
{
	cached_state.homed = false;
	publish_state();
}

/* --- Stage 1: coarse pre-home using inverse kinematics ---------------- */
/*
 * One quick probe per motor extracts only the SIGN of each motor's positive-
 * step direction (the magnitude is geometry-fixed: 2π/12800 rad per micro-
 * step at 1/64). Sign extraction is robust even where the probe magnitude
 * is suppressed by cos α near the gravity equilibrium pose.
 *
 * After signs are known, IK from a single IMU reading directly gives the
 * (α₁, α₂) deviation from level. Iterate up to 3 times because at α₁ ≈ ±90°
 * the IK is gimbal-locked in α₂; the first move escapes the singularity.
 *
 * If even after a kick the probe response stays under noise on both motors,
 * we're stuck at exactly the saddle and ask the user to nudge manually.
 */
static int coarse_home(const struct shell *sh)
{
	const int32_t probe = CONFIG_TT_HOMING_PROBE_STEPS;
	const int32_t kick = 1000;
	const int32_t max_step = 12800;
	const float min_response = 0.05f;
	const float skip_threshold = 30.0f;
	const float flip_threshold = 1.0f;
	const int max_iter = 3;
	const int max_kick_retries = 2;
	const float c_mag = 2.0f * KIN_PI / 12800.0f;
	float a[3], a0[3], a1[3];
	float theta, phi;
	float dz1 = 0.0f, dy2 = 0.0f;
	int sign1 = 0, sign2 = 0;
	int err;

	err = imu_read_accel_avg(a, 32);
	if (err) {
		return err;
	}

	/* Pre-flip if obviously upside-down. The IK formula α₁ = arcsin(g_z/g)
	 * has range [-π/2, π/2], so it cannot see the upper-half basin near
	 * α₁ = ±π — there iterative IK converges to a stuck pose. A 180°
	 * motor1 rotation always maps upside-down (a_x > 0) to right-side-up
	 * regardless of α₂. Threshold 1 m/s² ≈ cos α₁ < −0.1, i.e. α₁ more
	 * than ~96° off level — below that, IK handles it on its own. */
	if (a[0] > flip_threshold) {
		shell_print(sh,
			    "Inverted at start (a_x=%+.3f) — pre-flipping motor1 by 180°",
			    (double)a[0]);
		err = motion_one_by(MOTION_MOTOR1, HALF_REV_MICROSTEPS,
				    K_SECONDS(20));
		if (err) {
			return err;
		}
		err = imu_read_accel_avg(a, 32);
		if (err) {
			return err;
		}
	}

	kin_compute_tilt(a, &theta, &phi);
	if (theta < skip_threshold) {
		return 0;
	}

	shell_print(sh, "Coarse pre-home: starting θ=%.2f°", (double)theta);

	for (int retry = 0; retry <= max_kick_retries; retry++) {
		err = imu_read_accel_avg(a0, 16);
		if (err) {
			return err;
		}
		err = motion_one_by(MOTION_MOTOR1, probe, K_SECONDS(10));
		if (err) {
			return err;
		}
		err = imu_read_accel_avg(a1, 16);
		if (err) {
			return err;
		}
		err = motion_one_by(MOTION_MOTOR1, -probe, K_SECONDS(10));
		if (err) {
			return err;
		}
		dz1 = a1[2] - a0[2];

		err = imu_read_accel_avg(a0, 16);
		if (err) {
			return err;
		}
		err = motion_one_by(MOTION_MOTOR2, probe, K_SECONDS(10));
		if (err) {
			return err;
		}
		err = imu_read_accel_avg(a1, 16);
		if (err) {
			return err;
		}
		err = motion_one_by(MOTION_MOTOR2, -probe, K_SECONDS(10));
		if (err) {
			return err;
		}
		dy2 = a1[1] - a0[1];

		shell_print(sh,
			    "  probe: Δa_z(m1)=%+.4f Δa_y(m2)=%+.4f m/s²",
			    (double)dz1, (double)dy2);

		if (fabsf(dz1) >= min_response && fabsf(dy2) >= min_response) {
			sign1 = (dz1 < 0) ? +1 : -1;
			sign2 = (dy2 < 0) ? +1 : -1;
			break;
		}

		if (retry < max_kick_retries) {
			shell_print(sh,
				    "  saddle-like response; kicking motor1=%+d motor2=%+d",
				    -kick, kick);
			err = motion_pair_by(-kick, kick, K_SECONDS(10));
			if (err) {
				return err;
			}
		}
	}

	if (sign1 == 0 || sign2 == 0) {
		shell_error(sh,
			    "Coarse pre-home: probe responses below %.2f m/s² "
			    "after kick — re-orient platform manually",
			    (double)min_response);
		return -EIO;
	}

	float c1 = sign1 * c_mag;
	float c2 = sign2 * c_mag;

	shell_print(sh,
		    "  signs: motor1=%+d motor2=%+d (c1=%+.4g c2=%+.4g rad/μstep)",
		    sign1, sign2, (double)c1, (double)c2);

	for (int iter = 0; iter < max_iter; iter++) {
		err = imu_read_accel_avg(a, 32);
		if (err) {
			return err;
		}
		kin_compute_tilt(a, &theta, &phi);

		if (theta < skip_threshold) {
			shell_print(sh, "  iter %d: θ=%.2f° (converged)",
				    iter, (double)theta);
			return 0;
		}

		float alpha1, alpha2;

		kin_alpha_from_accel(a, &alpha1, &alpha2);

		int32_t d1 = (int32_t)(-alpha1 / c1);
		int32_t d2 = (int32_t)(-alpha2 / c2);

		if (d1 > max_step) {
			d1 = max_step;
		} else if (d1 < -max_step) {
			d1 = -max_step;
		}
		if (d2 > max_step) {
			d2 = max_step;
		} else if (d2 < -max_step) {
			d2 = -max_step;
		}

		shell_print(sh,
			    "  iter %d: θ=%.2f° α=(%.2f°, %.2f°) → move (%d, %d)",
			    iter, (double)theta,
			    (double)(alpha1 * KIN_DEG_PER_RAD),
			    (double)(alpha2 * KIN_DEG_PER_RAD), d1, d2);

		err = motion_pair_by(d1, d2, K_SECONDS(60));
		if (err) {
			return err;
		}
	}

	err = imu_read_accel_avg(a, 16);
	if (err) {
		return err;
	}
	kin_compute_tilt(a, &theta, &phi);
	if (theta >= skip_threshold) {
		shell_warn(sh,
			   "Coarse pre-home: didn't converge in %d iters "
			   "(θ=%.2f°) — letting Newton try anyway",
			   max_iter, (double)theta);
	} else {
		shell_print(sh, "Coarse pre-home: converged (θ=%.2f°)",
			    (double)theta);
	}
	return 0;
}

/* --- Stage 2 + 3: probe-Jacobian Newton + right-side-up disambiguation --- */

int homing_run(const struct shell *sh)
{
	const int32_t probe = CONFIG_TT_HOMING_PROBE_STEPS;
	const float converge_thresh =
		(float)CONFIG_TT_HOMING_CONVERGE_THRESH_MILLI_MS2 / 1000.0f;
	const int max_iter = CONFIG_TT_HOMING_MAX_ITERATIONS;
	const k_timeout_t move_to = K_SECONDS(10);
	float a0[3], a1[3];
	float jcol[2][2];
	bool flipped = false;
	int err;

	err = coarse_home(sh);
	if (err) {
		return err;
	}

	shell_print(sh, "Probing Jacobian (%d steps per axis)...", probe);

	err = imu_read_accel_avg(a0, 16);
	if (err) {
		goto fail;
	}
	err = motion_one_by(MOTION_MOTOR1, probe, move_to);
	if (err) {
		goto fail;
	}
	err = imu_read_accel_avg(a1, 16);
	if (err) {
		goto fail;
	}
	jcol[0][0] = (a1[1] - a0[1]) / (float)probe;
	jcol[0][1] = (a1[2] - a0[2]) / (float)probe;
	err = motion_one_by(MOTION_MOTOR1, -probe, move_to);
	if (err) {
		goto fail;
	}

	err = imu_read_accel_avg(a0, 16);
	if (err) {
		goto fail;
	}
	err = motion_one_by(MOTION_MOTOR2, probe, move_to);
	if (err) {
		goto fail;
	}
	err = imu_read_accel_avg(a1, 16);
	if (err) {
		goto fail;
	}
	jcol[1][0] = (a1[1] - a0[1]) / (float)probe;
	jcol[1][1] = (a1[2] - a0[2]) / (float)probe;
	err = motion_one_by(MOTION_MOTOR2, -probe, move_to);
	if (err) {
		goto fail;
	}

	float det = jcol[0][0] * jcol[1][1] - jcol[1][0] * jcol[0][1];

	if (fabsf(det) < 1.0e-5f) {
		shell_error(sh, "Jacobian singular (det=%.3g)", (double)det);
		return -EIO;
	}
	shell_print(sh,
		    "Jacobian: [[%+.4f, %+.4f], [%+.4f, %+.4f]] det=%+.4g",
		    (double)jcol[0][0], (double)jcol[1][0],
		    (double)jcol[0][1], (double)jcol[1][1], (double)det);

	for (int it = 0; it < max_iter; it++) {
		float tilt, az, lat;

		err = imu_read_accel_avg(a0, 32);
		if (err) {
			goto fail;
		}
		lat = sqrtf(a0[1] * a0[1] + a0[2] * a0[2]);
		kin_compute_tilt(a0, &tilt, &az);
		shell_print(sh,
			    "iter %d: θ=%.2f° φ=%.2f° |lat|=%.3f m/s²",
			    it, (double)tilt, (double)az, (double)lat);
		if (lat < converge_thresh) {
			break;
		}

		float dy = -a0[1], dz = -a0[2];
		float d1 = (jcol[1][1] * dy - jcol[1][0] * dz) / det;
		float d2 = (-jcol[0][1] * dy + jcol[0][0] * dz) / det;

		err = motion_pair_by((int32_t)d1, (int32_t)d2, move_to);
		if (err) {
			goto fail;
		}
	}

	/* Disambiguate level vs upside-down (Newton only zeroes lateral).
	 * Stage 1's pre-flip means this rarely triggers, but keep it as a
	 * safety net for cases where Newton converged into the wrong basin. */
	err = imu_read_accel_avg(a0, 32);
	if (err) {
		goto fail;
	}
	if (a0[0] > 0.0f) {
		shell_print(sh,
			    "Inverted (a_x=%+.3f) — flipping motor1 by 180°",
			    (double)a0[0]);
		err = motion_one_by(MOTION_MOTOR1, HALF_REV_MICROSTEPS,
				    K_SECONDS(20));
		if (err) {
			goto fail;
		}
		err = imu_read_accel_avg(a0, 32);
		if (err) {
			goto fail;
		}
		flipped = true;
	}

	float lat_final = sqrtf(a0[1] * a0[1] + a0[2] * a0[2]);
	float tilt_final, az_final;

	kin_compute_tilt(a0, &tilt_final, &az_final);

	if (a0[0] >= 0.0f || lat_final > 0.5f) {
		shell_error(sh,
			    "Homing failed: a_x=%+.3f |lat|=%.3f (θ=%.2f° φ=%.2f°)",
			    (double)a0[0], (double)lat_final,
			    (double)tilt_final, (double)az_final);
		return -EIO;
	}
	shell_print(sh, "Final: θ=%.2f° φ=%.2f° |lat|=%.3f m/s²",
		    (double)tilt_final, (double)az_final, (double)lat_final);

	err = motion_set_reference(MOTION_MOTOR1, 0);
	if (err) {
		goto fail;
	}
	err = motion_set_reference(MOTION_MOTOR2, 0);
	if (err) {
		goto fail;
	}

	/* Reuse the Stage 2 Jacobian as the cached level Jacobian. It was
	 * probed at the post-coarse pose (within skip_threshold of level), so
	 * it differs from the true level Jacobian by a cos α attenuation —
	 * worst case ~13% at 30° off, but typically <1% since Newton lands
	 * within a few degrees. The fast-path's fall-through to Newton
	 * absorbs any residual error.
	 *
	 * If Stage 3 flipped motor1 by 180°, the equivalent Jacobian at the
	 * post-flip pose is −J: substituting α₁ → α₁ − π in the kinematic
	 * model negates every accel component, and every Jacobian entry along
	 * with it. det is invariant under full negation, so the singularity
	 * check on the original det still applies. */
	if (flipped) {
		jcol[0][0] = -jcol[0][0];
		jcol[0][1] = -jcol[0][1];
		jcol[1][0] = -jcol[1][0];
		jcol[1][1] = -jcol[1][1];
	}

	cached_state.jac[0][0] = jcol[0][0];
	cached_state.jac[0][1] = jcol[0][1];
	cached_state.jac[1][0] = jcol[1][0];
	cached_state.jac[1][1] = jcol[1][1];
	cached_state.det = det;
	cached_state.homed = true;
	shell_print(sh,
		    "Homing complete; counters zeroed and Stage 2 Jacobian "
		    "cached%s (det=%+.3g)",
		    flipped ? " (sign-flipped)" : "", (double)det);
	publish_state();
	return 0;

fail:
	shell_error(sh, "homing failed: %d", err);
	return err;
}
