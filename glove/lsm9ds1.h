/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#ifndef __LSM9DS1_H
#define __LSM9DS1_H

#include <BLEServer.h>

bool lsm9ds1_init(void);
bool lsm9ds1_addService(BLEServer *pServer);
void lsm9ds1_loop(void);

#endif /* __LSM9DS1_H */
