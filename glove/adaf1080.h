/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#ifndef __ADAF1080_H
#define __ADAF1080_H

bool adaf1080_init(void);
bool adaf1080_addService(BLEServer *pServer);
void adaf1080_loop(void);

#endif /* __ADAF1080_H */
