/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * 2DOF Tilt Platform — application entry point.
 *
 * Initializes each component in dependency order. After this returns the
 * shell thread takes over; user-facing logic lives in src/shell_cmds and
 * the orchestrator components (homing, pose_ctrl).
 *
 * See app/README.md for architecture, math, and algorithms.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "homing.h"
#include "imu.h"
#include "motion.h"
#include "pose_ctrl.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

int main(void)
{
	LOG_INF("2DOF tilt platform firmware started");

	motion_init();
	imu_init();
	homing_init();
	pose_ctrl_init();

	LOG_INF("Drivers disabled; run 'platform enable' to power motors");
	return 0;
}
