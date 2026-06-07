/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU component — owns the LSM6DSO device, exposes averaged accel reads, and
 * owns the accelerometer calibration (the active affine map plus the 6-pose
 * capture session and its persistence). The calibration *math* is the pure,
 * host-tested calib module; this component is the hardware/orchestration side.
 */

#ifndef APP_IMU_H_
#define APP_IMU_H_

#include "calib.h"

int imu_init(void);

/** Fetch n samples and return the per-axis mean in m/s², with the active
 *  calibration applied. This is what homing and pose control read, so they
 *  get corrected data transparently. Sleeps ~10 ms between samples (104 Hz). */
int imu_read_accel_avg(float a_out[3], unsigned int n);

/** As imu_read_accel_avg but WITHOUT calibration — the raw sensor frame.
 *  Used by the calibration capture step, which must be independent of any
 *  map currently installed. */
int imu_read_accel_raw(float a_out[3], unsigned int n);

/** LSM6DSO die temperature in °C, or NAN if unavailable. Diagnostic only —
 *  logged at calibration time so offset drift vs. use temperature is visible. */
float imu_read_temp(void);

/* --- Calibration: 6-pose laser procedure ------------------------------- */

/* Each pose orients one IMU axis along ±gravity. The proper-acceleration the
 * chip reports is +g on the axis pointing UP (away from gravity), 0 elsewhere.
 * "X down" is the firmware's level pose (−x_imu up → a = (−g, 0, 0)). */
enum imu_calib_pose {
	IMU_CALIB_X_UP,
	IMU_CALIB_X_DOWN,
	IMU_CALIB_Y_UP,
	IMU_CALIB_Y_DOWN,
	IMU_CALIB_Z_UP,
	IMU_CALIB_Z_DOWN,
	IMU_CALIB_POSE_COUNT,
};

/** Discard any captured poses and begin a fresh calibration session. */
void imu_calib_session_reset(void);

/**
 * Capture the current static pose: average the raw accel and store it as the
 * observation for `pose` (overwrites a prior capture of the same pose).
 * @param raw_out  if non-NULL, receives the averaged raw accel (m/s²).
 * @param n_have   if non-NULL, receives the number of distinct poses captured.
 */
int imu_calib_capture(enum imu_calib_pose pose, float raw_out[3],
		      unsigned int *n_have);

/**
 * Solve the affine fit from the captured poses, install it as the active map,
 * and persist it. Requires ≥ CALIB_MIN_POSES distinct captures.
 * @param resid_rms  if non-NULL, receives the fit's RMS residual (m/s²).
 * @return 0, -EINVAL if too few poses, -EIO if the pose set is degenerate,
 *         or a settings error if persistence fails (map still installed).
 */
int imu_calib_solve_and_save(float *resid_rms);

/** Copy the active calibration map (valid=false ⇒ uncalibrated). */
void imu_calib_get(struct calib *out);

/** Drop the active calibration (back to pass-through) and erase the stored
 *  copy so it does not reload on next boot. */
int imu_calib_clear(void);

#endif /* APP_IMU_H_ */
