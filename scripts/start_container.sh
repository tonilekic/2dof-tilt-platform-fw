#!/usr/bin/env bash
# Copyright (c) 2026 Toni Lekic
# SPDX-License-Identifier: Apache-2.0
#
# Start the Zephyr developer Docker container with the west workspace mounted.

set -euo pipefail

ZEPHYR_DOCKER_IMAGE="ghcr.io/zephyrproject-rtos/zephyr-build:v0.29.1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACE_DIR="$(dirname "$REPO_DIR")"

docker run -ti \
    -v "$WORKSPACE_DIR":/workdir \
    -w /workdir/"$(basename "$REPO_DIR")" \
    "$ZEPHYR_DOCKER_IMAGE"
