#ifndef __BATTERY_H
#define __BATTERY_H

#include <BLEServer.h>

bool battery_init(void);
bool battery_addService(BLEServer *pServer);
void battery_loop(void);

#endif /* __BATTERY_H */
