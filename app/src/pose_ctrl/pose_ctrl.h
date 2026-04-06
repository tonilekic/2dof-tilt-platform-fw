/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pose controller — drives the platform to a commanded (tilt, azimuth)
 * pose using IMU feedback. Subscribes to chan_homing_state for the cached
 * level Jacobian; falls back to a full Newton path if not homed or if the
 * fast IK move misses the convergence threshold.
 */

#ifndef APP_POSE_CTRL_H_
#define APP_POSE_CTRL_H_

#include <zephyr/shell/shell.h>

int pose_ctrl_init(void);

/**
 * @brief Drive the platform to the commanded (tilt, azimuth).
 *
 *   Fast path (homed): closed-form IK using the cached level Jacobian, one
 *     parallel move_to, verify with IMU. If lateral residual < threshold,
 *     done; otherwise fall through to Newton.
 *   Newton path: probe a fresh Jacobian at the current pose, iterate
 *     modified Newton on the lateral residual to convergence.
 *
 * `sh` is used only for progress output and may be NULL.
 */
int pose_ctrl_set(const struct shell *sh, float tilt_deg, float azimuth_deg);

#endif /* APP_POSE_CTRL_H_ */
