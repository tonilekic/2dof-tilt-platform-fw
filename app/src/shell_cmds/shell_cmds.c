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

static const struct {
	const char *name;
	enum imu_calib_pose pose;
} calib_pose_names[] = {
	{ "x+", IMU_CALIB_X_UP },   { "x-", IMU_CALIB_X_DOWN },
	{ "y+", IMU_CALIB_Y_UP },   { "y-", IMU_CALIB_Y_DOWN },
	{ "z+", IMU_CALIB_Z_UP },   { "z-", IMU_CALIB_Z_DOWN },
};

static int cmd_calib_start(const struct shell *sh, size_t argc, char **argv)
{
	imu_calib_session_reset();
	shell_print(sh,
		    "Calibration session started. Level each IMU axis with the "
		    "laser, then 'platform calib capture <x±|y±|z±>'.");
	shell_print(sh,
		    "Capture %d+ poses (all six recommended), then "
		    "'platform calib solve'.", CALIB_MIN_POSES);
	return 0;
}

static int cmd_calib_capture(const struct shell *sh, size_t argc, char **argv)
{
	enum imu_calib_pose pose;
	bool found = false;

	for (size_t i = 0; i < ARRAY_SIZE(calib_pose_names); i++) {
		if (strcmp(argv[1], calib_pose_names[i].name) == 0) {
			pose = calib_pose_names[i].pose;
			found = true;
			break;
		}
	}
	if (!found) {
		shell_error(sh, "Unknown pose '%s' (use x+ x- y+ y- z+ z-)",
			    argv[1]);
		return -EINVAL;
	}

	float raw[3];
	unsigned int n_have;
	int err = imu_calib_capture(pose, raw, &n_have);

	if (err) {
		shell_error(sh, "capture failed: %d", err);
		return err;
	}
	shell_print(sh,
		    "Captured %s: raw ax=%+.3f ay=%+.3f az=%+.3f m/s²  (%u/6 poses)",
		    argv[1], (double)raw[0], (double)raw[1], (double)raw[2],
		    n_have);
	return 0;
}

static void print_calib_map(const struct shell *sh, const struct calib *c)
{
	for (int i = 0; i < 3; i++) {
		shell_print(sh, "  [%+.5f %+.5f %+.5f | %+.5f]",
			    (double)c->A[i][0], (double)c->A[i][1],
			    (double)c->A[i][2], (double)c->A[i][3]);
	}
}

static int cmd_calib_solve(const struct shell *sh, size_t argc, char **argv)
{
	float rms = 0.0f;
	int err = imu_calib_solve_and_save(&rms);

	if (err == -EINVAL) {
		shell_error(sh, "need at least %d captured poses",
			    CALIB_MIN_POSES);
		return err;
	}
	if (err == -EIO) {
		shell_error(sh,
			    "pose set is degenerate (re-capture with distinct "
			    "orientations)");
		return err;
	}
	if (err && err != -ENOENT) {
		/* Persist failures already warn in imu.c; map is still live. */
		shell_warn(sh, "calibration active but not saved (%d)", err);
	}

	struct calib c;

	imu_calib_get(&c);
	shell_print(sh, "Calibration solved, residual = %.4f m/s² (RMS).",
		    (double)rms);
	shell_print(sh, "A = [3×4 affine, a_corrected = A·(a_raw,1)]:");
	print_calib_map(sh, &c);
	return 0;
}

static int cmd_calib_show(const struct shell *sh, size_t argc, char **argv)
{
	struct calib c;

	imu_calib_get(&c);
	if (!c.valid) {
		shell_print(sh, "No calibration active (raw pass-through).");
		return 0;
	}
	shell_print(sh, "Active calibration (a_corrected = A·(a_raw,1)):");
	print_calib_map(sh, &c);
	return 0;
}

static int cmd_calib_clear(const struct shell *sh, size_t argc, char **argv)
{
	int err = imu_calib_clear();

	if (err) {
		shell_error(sh, "clear failed: %d", err);
		return err;
	}
	shell_print(sh, "Calibration cleared (raw pass-through).");
	return 0;
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

SHELL_STATIC_SUBCMD_SET_CREATE(sub_calib,
	SHELL_CMD(start, NULL, "Begin a fresh 6-pose calibration session",
		  cmd_calib_start),
	SHELL_CMD_ARG(capture, NULL,
		      "Capture current pose: calib capture <x+|x-|y+|y-|z+|z->",
		      cmd_calib_capture, 2, 0),
	SHELL_CMD(solve, NULL, "Solve the fit, install and persist it",
		  cmd_calib_solve),
	SHELL_CMD(show, NULL, "Show the active calibration map", cmd_calib_show),
	SHELL_CMD(clear, NULL, "Clear calibration (raw pass-through)",
		  cmd_calib_clear),
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
	SHELL_CMD(calib, &sub_calib, "Accelerometer calibration commands", NULL),
	SHELL_CMD(dfu, NULL, "Enter UF2 bootloader mode", cmd_dfu),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(platform, &sub_platform, "Platform commands", NULL);
