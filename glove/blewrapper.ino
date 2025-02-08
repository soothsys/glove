#include <BLECharacteristic.h>
#include <BLE2904.h>

#include "blewrapper.h"

static const int NUM_SERVICE_HANDLES = 3; //Each service requires 3 handles
static const int NUM_CHARACTERISTIC_HANDLES = 2; //Each characteristic requires 2 handles
static const int NUM_DESCRIPTOR_HANDLES = 3; //Each characteristic has 3 descriptors, each of which requires 1 handle

int BLEWrapper::calcNumHandles(int numCharacteristics) {
  return NUM_SERVICE_HANDLES + numCharacteristics * (NUM_CHARACTERISTIC_HANDLES + NUM_DESCRIPTOR_HANDLES);
}

BLEWrapper::BLEWrapper(BLECharacteristic *pCharacteristic, char *description, uint8_t format, int8_t exponent, BLEUnit unit) {
  m_pCharacteristic = pCharacteristic;
  m_format = format;
  m_exponent = exponent;
  m_unit = unit;

  //Namespace and description already populated with defaults
  m_presDescriptor.setFormat(m_format);
  m_presDescriptor.setExponent(m_exponent);
  m_presDescriptor.setUnit(static_cast<uint16_t>(m_unit));
  m_nameDescriptor.setDescription(description);

  m_pCharacteristic->addDescriptor(&m_nameDescriptor);
  m_pCharacteristic->addDescriptor(&m_cccDescriptor);
  m_pCharacteristic->addDescriptor(&m_presDescriptor);
}

BLECharacteristic * BLEWrapper::getCharacteristic(void) {
  return m_pCharacteristic;
}

void BLEWrapper::writeValue(float unscaled) {
	float scaleFactor = pow(10.0f, -m_exponent);
	float scaled = round(unscaled * scaleFactor);
	
	void *rawPtr;
	uint32_t uval;
	int32_t ival;
	switch (m_format) {
    case BLE2904::FORMAT_BOOLEAN:
      uval = ((unscaled == 0.0f) ? 0 : 1);
      rawPtr = static_cast<void *>(&uval);
      break;

		case BLE2904::FORMAT_UINT8:
		case BLE2904::FORMAT_UINT16:
		case BLE2904::FORMAT_UINT32:
			uval = static_cast<uint32_t>(scaled);
			rawPtr = static_cast<void *>(&uval);
			break;
		
		case BLE2904::FORMAT_SINT8:
		case BLE2904::FORMAT_SINT16:
		case BLE2904::FORMAT_SINT32:
		default:
			ival = static_cast<int32_t>(scaled);
			rawPtr = static_cast<void *>(&ival);
			break;
	}
	
	size_t length;
	switch (m_format) {
    case BLE2904::FORMAT_BOOLEAN:
		case BLE2904::FORMAT_UINT8:
		case BLE2904::FORMAT_SINT8:
			length = 1;
			break;
		
		case BLE2904::FORMAT_UINT16:
		case BLE2904::FORMAT_SINT16:
			length = 2;
			break;
		
		case BLE2904::FORMAT_UINT32:
		case BLE2904::FORMAT_SINT32:
		default:
			length = 4;
			break;
	}
	
	uint8_t *bytes = static_cast<uint8_t *>(rawPtr);
	m_pCharacteristic->setValue(bytes, length);

  if (!m_written || (m_lastVal.f != unscaled)) {
    m_pCharacteristic->notify();
  }

  m_written = true;
  m_lastVal.f = unscaled;
}

void BLEWrapper::writeValue(bool b) {
  uint8_t bytes[1];
  bytes[0] = (b ? 1 : 0);
  m_pCharacteristic->setValue(bytes, 1);

  if (!m_written || (m_lastVal.b != b)) {
    m_pCharacteristic->notify();
  }

  m_written = true;
  m_lastVal.b = b;  
}
