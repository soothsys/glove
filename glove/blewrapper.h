/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#ifndef __BLEWRAPPER_H
#define __BLEWRAPPER_H

#include <stdint.h>
#include <BLECharacteristic.h>
#include <BLE2901.h>
#include <BLE2902.h>
#include <BLE2904.h>

/*
 * Unit UUIDs defined in Bluetooth Assigned Numbers specification, section 3.5
 * https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Assigned_Numbers/out/en/Assigned_Numbers.pdf
 */
enum class BLEUnit {
	Unitless	             = 0x2700,
  MetresPerSecondSquared = 0x2713,
  Radian                 = 0x2720,
	Pascal		             = 0x2724,
  Volt                   = 0x2728,
  uTesla                 = 0x272D,
	DegC	  	             = 0x272F,
  RadsPerSecond          = 0x2743,
	Percent		             = 0x27AD,
	PPM			               = 0x27C4,
  PPB                    = 0x27C5
};

class BLEWrapper {
	private:
    BLECharacteristic *m_pCharacteristic;
    BLE2901 m_nameDescriptor; //Characteristic user description descriptor
    BLE2902 m_cccDescriptor; //Client characteristic configuration descriptor
    BLE2904 m_presDescriptor; //Characteristic presentation format descriptor
		uint8_t m_format;
		int8_t m_exponent;
		BLEUnit m_unit;
    bool m_written = false;

    union {
      float f;
      bool b;
    } m_lastVal;
		
  public:
    BLEWrapper(BLECharacteristic *pCharacteristic, char *description, uint8_t format, int8_t exponent, BLEUnit unit);
    BLECharacteristic * getCharacteristic(void);
		void writeValue(float unscaled);
    void writeValue(bool b);

    static int calcNumHandles(int numCharacteristics);
};

#endif /* __BLEWRAPPER_H */
