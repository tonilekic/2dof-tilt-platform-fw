/*
 * Copyright (c) 2026 Toni Lekic
 * SPDX-License-Identifier: Apache-2.0
 *
 * Single translation unit holding all ZBUS_CHAN_DEFINE macros so channels
 * are linked exactly once.
 */

#include "app/zbus_channels.h"

ZBUS_CHAN_DEFINE(chan_homing_state,
		 struct homing_state,
		 NULL,                          /* validator */
		 NULL,                          /* user_data */
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.homed = false));
