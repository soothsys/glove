#define ERR_MODULE_NAME "BATTERY"

#include <BLEServer.h>
#include <BLEUtils.h>

#include "blewrapper.h"
#include "err.h"

#define SAMPLE_TIME 1000 //milliseconds
#define NUM_AVERAGE_SAMPLES 10 //Average battery voltage over 10 seconds to remove fluctuations due to load
#define PIN_VBAT A13
#define VBAT_SCALE (1.0f / 500.0f)
#define VBAT_FULL 4.10f
#define VBAT_LOW 3.60f
#define VBAT_CRITICAL 3.30f
#define VBAT_EMPTY 3.20f

#define BLE_INST_ID 0
#define NUM_CHARACTERISTICS 3

#define BLE_SERVICE_UUID BLEUUID((uint16_t)0x180F)
#define LEVEL_UUID BLEUUID((uint16_t)0x2A19)
#define CRITICAL_UUID BLEUUID((uint16_t)0x2BE9)
#define VOLTAGE_UUID "56b2c2d5-abc6-4801-a39a-02dee738b38c"

#define LEVEL_FORMAT BLE2904::FORMAT_UINT8
#define CRITICAL_FORMAT BLE2904::FORMAT_BOOLEAN
#define VOLTAGE_FORMAT BLE2904::FORMAT_UINT16

#define LEVEL_EXPONENT 0
#define CRITICAL_EXPONENT 0
#define VOLTAGE_EXPONENT -2

#define LEVEL_UNIT BLEUnit::Percent
#define CRITICAL_UNIT BLEUnit::Unitless
#define VOLTAGE_UNIT BLEUnit::Volt

#define LEVEL_NAME "Battery level"
#define CRITICAL_NAME "Battery critical"
#define VOLTAGE_NAME "Battery voltage"

static BLECharacteristic m_levelCharacteristic(LEVEL_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_criticalCharacteristic(CRITICAL_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_voltageCharacteristic(VOLTAGE_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

static BLEWrapper m_levelWrapper(&m_levelCharacteristic, LEVEL_NAME, LEVEL_FORMAT, LEVEL_EXPONENT, LEVEL_UNIT);
static BLEWrapper m_criticalWrapper(&m_criticalCharacteristic, CRITICAL_NAME, CRITICAL_FORMAT, CRITICAL_EXPONENT, CRITICAL_UNIT);
static BLEWrapper m_voltageWrapper(&m_voltageCharacteristic, VOLTAGE_NAME, VOLTAGE_FORMAT, VOLTAGE_EXPONENT, VOLTAGE_UNIT);

static float m_avgBuf[NUM_AVERAGE_SAMPLES];
static int m_avgBufPos = 0;
static unsigned long m_lastTime;

static float readVoltage(void) {
  return VBAT_SCALE * (float)analogReadMilliVolts(PIN_VBAT);
}

bool battery_init(void) {
  float vbat = readVoltage();
  int i;
  for (i = 0; i < NUM_AVERAGE_SAMPLES; i++) {
    m_avgBuf[i] = vbat; //Initialise average buffer
  }

  m_lastTime = millis();
  return true;
}

bool battery_addService(BLEServer *pServer) {
  int numHandles = BLEWrapper::calcNumHandles(NUM_CHARACTERISTICS);
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID, numHandles, BLE_INST_ID);
  if (pService == NULL) {
    ERROR("Cannot add BLE service");
    return false;
  }

  pService->addCharacteristic(&m_levelCharacteristic);
  pService->addCharacteristic(&m_criticalCharacteristic);
  pService->addCharacteristic(&m_voltageCharacteristic);

  pService->start();
  return true;
}

static void printBatteryVoltage(float vbat) {
  Serial.print("(Battery voltage: ");
  Serial.print(vbat, 2);
  Serial.println("V)");
}

float getAverage(float next) {
  m_avgBuf[m_avgBufPos++] = next;
  if (m_avgBufPos >= NUM_AVERAGE_SAMPLES) {
    m_avgBufPos = 0;
  }

  float sum = 0.0f;
  int i;
  for (i = 0; i < NUM_AVERAGE_SAMPLES; i++) {
    sum += m_avgBuf[i];
  }

  return sum / (float)NUM_AVERAGE_SAMPLES;
}

void battery_loop(void) {
  unsigned long now = millis();
  if (now - m_lastTime >= SAMPLE_TIME) {
    m_lastTime = now;

    float vbat = getAverage(readVoltage());
    bool low = (vbat < VBAT_LOW);
    bool critical = (vbat < VBAT_CRITICAL);
    if (critical) {
      Serial.println("Battery is critically low!");
      printBatteryVoltage(vbat);
    } else if (low) {
      Serial.println("Battery is low!");
      printBatteryVoltage(vbat);
    }

    /*
     * Rough estimate of battery level as a percentage.
     * Better method would take discharge curve into account.
     */
    float level = 100.0f * (vbat - VBAT_EMPTY) / (VBAT_FULL - VBAT_EMPTY);
    if (level < 0.0f) {
      level = 0.0f;
    } else if (level > 100.0f) {
      level = 100.0f;
    }
    
    m_levelWrapper.writeValue(level);
    m_levelCharacteristic.notify();
    m_criticalWrapper.writeValue(critical);
    m_criticalCharacteristic.notify();
    m_voltageWrapper.writeValue(vbat);
    m_voltageCharacteristic.notify();
  }
}
