/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#ifndef __I2C_ADDRESS_H
#define __I2C_ADDRESS_H

typedef enum {
  i2c_address_as7341 = 0x39,
	i2c_address_bme688 = 0x77
} i2c_address_t;

#endif /* __I2C_ADDRESS_H */
