/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zbus channel and message-type declarations shared across components.
 * Per-channel rule: zbus is used for state broadcasts and decoupled events;
 * tight control loops (Newton iterations) use direct calls instead.
 */

#ifndef APP_ZBUS_CHANNELS_H_
#define APP_ZBUS_CHANNELS_H_

#include <stdbool.h>

#include <zephyr/zbus/zbus.h>

/**
 * @brief Cached homing state.
 *
 * Published by the `homing` component after a successful soft-home, and on
 * invalidation (e.g. `platform disable`). Consumers read on demand via
 * zbus_chan_read() — the channel acts as a state cache.
 *
 * jac[motor][axis] holds the level-pose Jacobian:
 *   jac[0][0] = ∂a_y/∂step_motor1   jac[0][1] = ∂a_z/∂step_motor1
 *   jac[1][0] = ∂a_y/∂step_motor2   jac[1][1] = ∂a_z/∂step_motor2
 * det = jac[0][0]·jac[1][1] − jac[0][1]·jac[1][0].
 */
struct homing_state {
	bool homed;
	float jac[2][2];
	float det;
};

ZBUS_CHAN_DECLARE(chan_homing_state);

#endif /* APP_ZBUS_CHANNELS_H_ */
