/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU component — owns the LSM6DSO device, exposes averaged accel reads.
 */

#ifndef APP_IMU_H_
#define APP_IMU_H_

int imu_init(void);

/** Fetch n samples from the IMU and return the per-axis arithmetic mean
 *  in m/s². Sleeps ~10 ms between samples to match a 104 Hz ODR. */
int imu_read_accel_avg(float a_out[3], unsigned int n);

#endif /* APP_IMU_H_ */
