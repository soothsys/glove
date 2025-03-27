#ifndef __ADAF1080_H
#define __ADAF1080_H

bool adaf1080_init(void);
bool adaf1080_addService(BLEServer *pServer);
void adaf1080_loop(void);

#endif /* __ADAF1080_H */
