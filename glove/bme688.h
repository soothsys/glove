/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#ifndef __BME688_H
#define __BME688_H

#include <BLEServer.h>

#include "i2c_address.h"

bool bme688_init(i2c_address_t addr);
bool bme688_addService(BLEServer *pServer);
void bme688_loop(void);

#endif /* __BME688_H */
