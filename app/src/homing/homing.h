/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Homing component — implements the 3-stage soft-home algorithm and owns
 * the cached level-pose Jacobian used by the pose controller's fast path.
 * See app/README.md "Soft homing algorithm" for the math and state diagram.
 */

#ifndef APP_HOMING_H_
#define APP_HOMING_H_

#include <stdbool.h>

#include <zephyr/shell/shell.h>

int homing_init(void);

/**
 * @brief Run the 3-stage soft homing on the current platform pose.
 *
 *   1. Coarse pre-home: probe sign of each motor, IK iteration → ±30°.
 *   2. Newton refinement: probe Jacobian, drive lateral accel → 0.
 *   3. Right-side-up disambiguation: flip motor1 by 180° if upside-down.
 * On success, zeroes the step counters and caches the level Jacobian to
 * the chan_homing_state zbus channel.
 *
 * `sh` is used only for progress output and may be NULL.
 */
int homing_run(const struct shell *sh);

bool homing_is_homed(void);

/** Cached level Jacobian. Returns 0 on success or -EAGAIN if not homed. */
int homing_get_level_jacobian(float jac[2][2], float *det);

/** Mark homing state invalid (e.g. on `platform disable`). Republishes
 *  to chan_homing_state so consumers see the change. */
void homing_invalidate(void);

#endif /* APP_HOMING_H_ */
