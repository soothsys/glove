/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#ifndef __AS7341_H
#define __AS7341_H

#include <BLEServer.h>

#include "i2c_address.h"

bool as7341_init(i2c_address_t addr);
bool as7341_addService(BLEServer *pServer);
void as7341_loop(void);

#endif /* __AS7341_H */
