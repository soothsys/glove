#ifndef __BME688_H
#define __BME688_H

#include <BLEServer.h>

#include "i2c_address.h"

bool bme688_init(i2c_address_t addr);
bool bme688_addService(BLEServer *pServer);
void bme688_loop(void);

#endif /* __BME688_H */
