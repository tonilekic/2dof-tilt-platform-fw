/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bringup sample – GPIO shell for PCB testing.
 *
 * This sample provides an interactive shell over USB CDC ACM with the
 * built-in Zephyr GPIO shell commands. Use it to toggle individual
 * pins and verify the circuit on the 2dof-tilt-platform PCB.
 */

#include <zephyr/kernel.h>

int main(void)
{
	/* The shell subsystem runs in its own thread – nothing to do here. */
	return 0;
}
