#ifndef __AS7341_H
#define __AS7341_H

#include <BLEServer.h>

#include "i2c_address.h"

bool as7341_init(i2c_address_t addr);
bool as7341_addService(BLEServer *pServer);
void as7341_loop(void);

#endif /* __AS7341_H */
