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
	Unitless	= 0x2700,
	Pascal		= 0x2724,
	DegC	  	= 0x272F,
	Percent		= 0x27AD,
	PPM			  = 0x27C4,
  PPB       = 0x27C5
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
		
  public:
    BLEWrapper(BLECharacteristic *pCharacteristic, char *description, uint8_t format, int8_t exponent, BLEUnit unit);
    BLECharacteristic * getCharacteristic(void);
		void writeValue(float unscaled);

  static int NUM_HANDLES;
};

#endif /* __BLEWRAPPER_H */
