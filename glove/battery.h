/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#ifndef __BATTERY_H
#define __BATTERY_H

#include <BLEServer.h>

bool battery_init(void);
bool battery_addService(BLEServer *pServer);
void battery_loop(void);

#endif /* __BATTERY_H */
