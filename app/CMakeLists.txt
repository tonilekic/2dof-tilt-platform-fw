# Copyright (c) 2026 Toni Lekic
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(2dof-tilt-platform)

target_include_directories(app PRIVATE
    include
    src/kinematics
    src/motion
    src/imu
    src/calib
    src/homing
    src/pose_ctrl
)

target_sources(app PRIVATE
    src/main.c
    src/zbus_channels.c
    src/kinematics/kinematics.c
    src/motion/motion.c
    src/imu/imu.c
    src/calib/calib.c
    src/homing/homing.c
    src/pose_ctrl/pose_ctrl.c
    src/shell_cmds/shell_cmds.c
)
