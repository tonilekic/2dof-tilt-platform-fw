/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shell glue — translates `platform <cmd>` shell calls to component APIs.
 * No domain logic lives here; everything just dispatches.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <hal/nrf_power.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>

#include "homing.h"
#include "imu.h"
#include "kinematics.h"
#include "motion.h"
#include "pose_ctrl.h"

static int cmd_enable(const struct shell *sh, size_t argc, char **argv)
{
	int ret = motion_enable();

	if (ret) {
		shell_error(sh, "Failed to enable: %d", ret);
		return ret;
	}
	shell_print(sh, "Stepper drivers enabled (run 'platform home' to soft-home)");
	return 0;
}

static int cmd_disable(const struct shell *sh, size_t argc, char **argv)
{
	int ret = motion_disable();

	if (ret) {
		shell_error(sh, "Failed to disable: %d", ret);
		return ret;
	}
	homing_invalidate();
	shell_print(sh, "Stepper drivers disabled");
	return 0;
}

static int cmd_move(const struct shell *sh, size_t argc, char **argv)
{
	enum motion_motor m;
	int err = 0;

	if (motion_motor_from_name(argv[1], &m)) {
		shell_error(sh, "Unknown motor '%s' (use motor1 or motor2)", argv[1]);
		return -EINVAL;
	}

	int32_t steps = shell_strtol(argv[2], 10, &err);

	if (err) {
		shell_error(sh, "Invalid step count '%s'", argv[2]);
		return -EINVAL;
	}

	err = motion_one_by(m, steps, K_SECONDS(60));
	if (err) {
		shell_error(sh, "move failed: %d", err);
		return err;
	}
	shell_print(sh, "%s: moved %d steps", argv[1], steps);
	return 0;
}

static int cmd_run(const struct shell *sh, size_t argc, char **argv)
{
	enum motion_motor m;
	enum stepper_ctrl_direction dir;

	if (motion_motor_from_name(argv[1], &m)) {
		shell_error(sh, "Unknown motor '%s'", argv[1]);
		return -EINVAL;
	}

	if (strcmp(argv[2], "positive") == 0 || strcmp(argv[2], "+") == 0) {
		dir = STEPPER_CTRL_DIRECTION_POSITIVE;
	} else if (strcmp(argv[2], "negative") == 0 || strcmp(argv[2], "-") == 0) {
		dir = STEPPER_CTRL_DIRECTION_NEGATIVE;
	} else {
		shell_error(sh, "Invalid direction '%s'", argv[2]);
		return -EINVAL;
	}

	int err = motion_run(m, dir);

	if (err) {
		shell_error(sh, "run failed: %d", err);
		return err;
	}
	shell_print(sh, "%s: running", argv[1]);
	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	enum motion_motor m;

	if (motion_motor_from_name(argv[1], &m)) {
		shell_error(sh, "Unknown motor '%s'", argv[1]);
		return -EINVAL;
	}

	int err = motion_stop(m);

	if (err) {
		shell_error(sh, "stop failed: %d", err);
		return err;
	}
	shell_print(sh, "%s: stopped", argv[1]);
	return 0;
}

static int cmd_pos(const struct shell *sh, size_t argc, char **argv)
{
	enum motion_motor m;
	int32_t pos;

	if (motion_motor_from_name(argv[1], &m)) {
		shell_error(sh, "Unknown motor '%s'", argv[1]);
		return -EINVAL;
	}

	int err = motion_get_position(m, &pos);

	if (err) {
		shell_error(sh, "get_position failed: %d", err);
		return err;
	}
	shell_print(sh, "%s: position = %d", argv[1], pos);
	return 0;
}

static int cmd_imu_read(const struct shell *sh, size_t argc, char **argv)
{
	float a[3], tilt, az;
	int err = imu_read_accel_avg(a, 32);

	if (err) {
		shell_error(sh, "imu read failed: %d", err);
		return err;
	}

	float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

	kin_compute_tilt(a, &tilt, &az);

	shell_print(sh, "accel  ax=%+.3f ay=%+.3f az=%+.3f m/s²  |a|=%.3f",
		    (double)a[0], (double)a[1], (double)a[2], (double)n);
	shell_print(sh, "tilt   θ=%+.2f° azimuth φ=%+.2f°",
		    (double)tilt, (double)az);
	return 0;
}

static int cmd_home(const struct shell *sh, size_t argc, char **argv)
{
	return homing_run(sh);
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	char *endp;
	float tilt = strtof(argv[1], &endp);

	if (endp == argv[1] || *endp != '\0') {
		shell_error(sh, "Invalid tilt '%s'", argv[1]);
		return -EINVAL;
	}

	float az = strtof(argv[2], &endp);

	if (endp == argv[2] || *endp != '\0') {
		shell_error(sh, "Invalid azimuth '%s'", argv[2]);
		return -EINVAL;
	}

	return pose_ctrl_set(sh, tilt, az);
}

#define ADAFRUIT_BOOTLOADER_DFU_MAGIC 0x57

static int cmd_dfu(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Entering UF2 bootloader...");
	k_sleep(K_MSEC(100));
	nrf_power_gpregret_set(NRF_POWER, 0, ADAFRUIT_BOOTLOADER_DFU_MAGIC);
	sys_reboot(SYS_REBOOT_COLD);
	CODE_UNREACHABLE;
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu,
	SHELL_CMD(read, NULL, "Read accel and print tilt/azimuth", cmd_imu_read),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_platform,
	SHELL_CMD(enable, NULL, "Enable stepper drivers", cmd_enable),
	SHELL_CMD(disable, NULL, "Disable stepper drivers", cmd_disable),
	SHELL_CMD_ARG(move, NULL, "Move motor: platform move <motor> <steps>",
		      cmd_move, 3, 0),
	SHELL_CMD_ARG(run, NULL,
		      "Run motor: platform run <motor> <positive|negative>",
		      cmd_run, 3, 0),
	SHELL_CMD_ARG(stop, NULL, "Stop motor: platform stop <motor>",
		      cmd_stop, 2, 0),
	SHELL_CMD_ARG(pos, NULL, "Get position: platform pos <motor>",
		      cmd_pos, 2, 0),
	SHELL_CMD(home, NULL, "IMU-based soft homing (zero step counters at level)",
		  cmd_home),
	SHELL_CMD_ARG(set, NULL,
		      "Set pose: platform set <tilt_deg> <azimuth_deg>",
		      cmd_set, 3, 0),
	SHELL_CMD(imu, &sub_imu, "IMU commands", NULL),
	SHELL_CMD(dfu, NULL, "Enter UF2 bootloader mode", cmd_dfu),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(platform, &sub_platform, "Platform commands", NULL);
