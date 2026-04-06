/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "kinematics.h"
#include "motion.h"

LOG_MODULE_REGISTER(motion, LOG_LEVEL_INF);

static const struct device *const drv1 =
	DEVICE_DT_GET(DT_ALIAS(stepper_driver1));
static const struct device *const drv2 =
	DEVICE_DT_GET(DT_ALIAS(stepper_driver2));

static const struct device *const motor_dev[2] = {
	DEVICE_DT_GET(DT_ALIAS(motor1)),
	DEVICE_DT_GET(DT_ALIAS(motor2)),
};

static K_SEM_DEFINE(motor1_done, 0, 1);
static K_SEM_DEFINE(motor2_done, 0, 1);
static struct k_sem *const done_sem[2] = { &motor1_done, &motor2_done };

static int motor_index(const struct device *dev)
{
	if (dev == motor_dev[0]) {
		return 0;
	}
	if (dev == motor_dev[1]) {
		return 1;
	}
	return -1;
}

static void stepper_event_cb(const struct device *dev,
			     const enum stepper_ctrl_event event,
			     void *user_data)
{
	int idx = motor_index(dev);

	switch (event) {
	case STEPPER_CTRL_EVENT_STEPS_COMPLETED:
		LOG_DBG("%s: steps completed", dev->name);
		if (idx >= 0) {
			k_sem_give(done_sem[idx]);
		}
		break;
	case STEPPER_CTRL_EVENT_LEFT_END_STOP_DETECTED:
		LOG_WRN("%s: left end stop detected", dev->name);
		break;
	case STEPPER_CTRL_EVENT_RIGHT_END_STOP_DETECTED:
		LOG_WRN("%s: right end stop detected", dev->name);
		break;
	case STEPPER_CTRL_EVENT_STOPPED:
		LOG_DBG("%s: stopped", dev->name);
		if (idx >= 0) {
			k_sem_give(done_sem[idx]);
		}
		break;
	default:
		LOG_DBG("%s: event %d", dev->name, event);
		break;
	}
}

int motion_init(void)
{
	for (int i = 0; i < 2; i++) {
		if (!device_is_ready(motor_dev[i])) {
			LOG_ERR("%s not ready", motor_dev[i]->name);
			return -ENODEV;
		}

		int ret = stepper_ctrl_set_event_cb(motor_dev[i],
						    stepper_event_cb, NULL);

		if (ret) {
			LOG_ERR("%s: set_event_cb failed (%d)",
				motor_dev[i]->name, ret);
			return ret;
		}

		ret = stepper_ctrl_set_microstep_interval(motor_dev[i],
							  CONFIG_TT_MOTION_STEP_INTERVAL_NS);
		if (ret) {
			LOG_ERR("%s: set_microstep_interval failed (%d)",
				motor_dev[i]->name, ret);
			return ret;
		}
	}

	/* TMC2209 driver init configures EN as GPIO_OUTPUT without an initial
	 * level — the pin's value register defaults to 0, which with
	 * GPIO_ACTIVE_LOW means logical "active". Force inactive at boot. */
	(void)motion_disable();
	return 0;
}

int motion_enable(void)
{
	int ret = stepper_enable(drv1);

	if (ret) {
		return ret;
	}
	return stepper_enable(drv2);
}

int motion_disable(void)
{
	int ret = stepper_disable(drv1);

	if (ret) {
		return ret;
	}
	return stepper_disable(drv2);
}

/* --- S-curve segmented parallel moves --------------------------------- */

/* Issue one segment to each motor (steps != 0) and wait for both completion
 * events. Caller must have set the per-motor step interval beforehand. */
static int move_pair_segment(int32_t steps1, int32_t steps2,
			     k_timeout_t timeout)
{
	int err;

	if (steps1 != 0) {
		k_sem_reset(done_sem[0]);
	}
	if (steps2 != 0) {
		k_sem_reset(done_sem[1]);
	}

	if (steps1 != 0) {
		err = stepper_ctrl_move_by(motor_dev[0], steps1);
		if (err) {
			return err;
		}
	}
	if (steps2 != 0) {
		err = stepper_ctrl_move_by(motor_dev[1], steps2);
		if (err) {
			return err;
		}
	}

	if (steps1 != 0) {
		err = k_sem_take(done_sem[0], timeout);
		if (err) {
			return err;
		}
	}
	if (steps2 != 0) {
		err = k_sem_take(done_sem[1], timeout);
		if (err) {
			return err;
		}
	}
	return 0;
}

int motion_pair_by(int32_t steps1, int32_t steps2, k_timeout_t timeout)
{
	const int n_segs = CONFIG_TT_MOTION_RAMP_SEGMENTS;
	const uint32_t v_min = CONFIG_TT_MOTION_MIN_STEP_RATE;
	const uint32_t v_max = CONFIG_TT_MOTION_MAX_STEP_RATE;
	const int32_t n_ramp = CONFIG_TT_MOTION_RAMP_STEPS;
	int32_t abs1 = (steps1 < 0) ? -steps1 : steps1;
	int32_t abs2 = (steps2 < 0) ? -steps2 : steps2;
	int32_t dir1 = (steps1 >= 0) ? 1 : -1;
	int32_t dir2 = (steps2 >= 0) ? 1 : -1;
	int32_t prev1 = 0, prev2 = 0;
	int err;

	if (abs1 == 0 && abs2 == 0) {
		return 0;
	}

	for (int i = 0; i < n_segs; i++) {
		int32_t end1 = (int32_t)(((int64_t)(i + 1) * abs1) / n_segs);
		int32_t end2 = (int32_t)(((int64_t)(i + 1) * abs2) / n_segs);
		int32_t seg1 = end1 - prev1;
		int32_t seg2 = end2 - prev2;

		if (seg1 > 0) {
			uint32_t v = kin_scurve_velocity(prev1, abs1, v_min,
							 v_max, n_ramp);

			err = stepper_ctrl_set_microstep_interval(motor_dev[0],
								  1000000000ULL / v);
			if (err) {
				return err;
			}
		}
		if (seg2 > 0) {
			uint32_t v = kin_scurve_velocity(prev2, abs2, v_min,
							 v_max, n_ramp);

			err = stepper_ctrl_set_microstep_interval(motor_dev[1],
								  1000000000ULL / v);
			if (err) {
				return err;
			}
		}

		err = move_pair_segment(dir1 * seg1, dir2 * seg2, timeout);
		if (err) {
			return err;
		}

		prev1 = end1;
		prev2 = end2;
	}
	return 0;
}

int motion_pair_to(int32_t target1, int32_t target2, k_timeout_t timeout)
{
	int32_t curr1, curr2;
	int err;

	err = stepper_ctrl_get_actual_position(motor_dev[0], &curr1);
	if (err) {
		return err;
	}
	err = stepper_ctrl_get_actual_position(motor_dev[1], &curr2);
	if (err) {
		return err;
	}
	return motion_pair_by(target1 - curr1, target2 - curr2, timeout);
}

int motion_one_by(enum motion_motor m, int32_t steps, k_timeout_t timeout)
{
	if (m == MOTION_MOTOR1) {
		return motion_pair_by(steps, 0, timeout);
	}
	if (m == MOTION_MOTOR2) {
		return motion_pair_by(0, steps, timeout);
	}
	return -EINVAL;
}

int motion_run(enum motion_motor m, enum stepper_ctrl_direction dir)
{
	if (m != MOTION_MOTOR1 && m != MOTION_MOTOR2) {
		return -EINVAL;
	}
	return stepper_ctrl_run(motor_dev[m], dir);
}

int motion_stop(enum motion_motor m)
{
	if (m != MOTION_MOTOR1 && m != MOTION_MOTOR2) {
		return -EINVAL;
	}
	return stepper_ctrl_stop(motor_dev[m]);
}

int motion_get_position(enum motion_motor m, int32_t *pos)
{
	if (m != MOTION_MOTOR1 && m != MOTION_MOTOR2) {
		return -EINVAL;
	}
	return stepper_ctrl_get_actual_position(motor_dev[m], pos);
}

int motion_set_reference(enum motion_motor m, int32_t value)
{
	if (m != MOTION_MOTOR1 && m != MOTION_MOTOR2) {
		return -EINVAL;
	}
	return stepper_ctrl_set_reference_position(motor_dev[m], value);
}

int motion_motor_from_name(const char *name, enum motion_motor *out)
{
	if (strcmp(name, "motor1") == 0 || strcmp(name, "1") == 0) {
		*out = MOTION_MOTOR1;
		return 0;
	}
	if (strcmp(name, "motor2") == 0 || strcmp(name, "2") == 0) {
		*out = MOTION_MOTOR2;
		return 0;
	}
	return -EINVAL;
}
