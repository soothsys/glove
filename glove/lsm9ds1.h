#ifndef __LSM9DS1_H
#define __LSM9DS1_H

#include <BLEServer.h>

bool lsm9ds1_init(void);
bool lsm9ds1_addService(BLEServer *pServer);
void lsm9ds1_loop(void);

#endif /* __LSM9DS1_H */
