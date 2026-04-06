/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Motion component — owns the stepper devices and provides parallel-motor
 * moves with an S-curve velocity ramp. See app/README.md "Motion control"
 * for the velocity profile and parallel-motor scheduling.
 */

#ifndef APP_MOTION_H_
#define APP_MOTION_H_

#include <stdint.h>

#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/kernel.h>

enum motion_motor {
	MOTION_MOTOR1 = 0,
	MOTION_MOTOR2 = 1,
};

/** Initialize stepper devices, register event callbacks, set initial step
 *  interval, then assert the EN pin inactive. Call once at boot. */
int motion_init(void);

/** Enable / disable both stepper drivers (powers the motor coils). */
int motion_enable(void);
int motion_disable(void);

/**
 * @brief Move both motors by relative steps in parallel using S-curve ramp.
 *
 * Pass 0 to skip a motor. Each motor follows its own per-motor profile, so a
 * small move doesn't stall the larger one. Returns once both motors stop.
 */
int motion_pair_by(int32_t steps1, int32_t steps2, k_timeout_t timeout);

/** Move both motors to absolute step targets in parallel. */
int motion_pair_to(int32_t target1, int32_t target2, k_timeout_t timeout);

/** Move one motor by relative steps; the other holds position. */
int motion_one_by(enum motion_motor m, int32_t steps, k_timeout_t timeout);

/** Continuous run / stop / position-counter API for raw shell commands. */
int motion_run(enum motion_motor m, enum stepper_ctrl_direction dir);
int motion_stop(enum motion_motor m);
int motion_get_position(enum motion_motor m, int32_t *pos);
int motion_set_reference(enum motion_motor m, int32_t value);

/**
 * @brief Resolve "motor1"|"motor2"|"1"|"2" string to a motion_motor enum.
 * Returns 0 on success and writes to *out, -EINVAL otherwise.
 */
int motion_motor_from_name(const char *name, enum motion_motor *out);

#endif /* APP_MOTION_H_ */
