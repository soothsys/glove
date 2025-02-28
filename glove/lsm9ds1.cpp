#define ERR_MODULE_NAME "LSM9DS1"

#include <Wire.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#include "blewrapper.h"
#include "err.h"

#define SAMPLE_TIME 1000 //milliseconds
#define BLE_INST_ID 0
#define NUM_CHARACTERISTICS 3

#define BLE_SERVICE_UUID BLEUUID("606a0692-1e69-422a-9f73-de87d239aade")
#define MAG_X_UUID BLEUUID("14bbfa6b-347a-4cb1-ad8c-4c81cdc4259b")
#define MAG_Y_UUID BLEUUID("21e5d780-5ff4-452e-8a29-6d04a8f004a5")
#define MAG_Z_UUID BLEUUID("8fccbd0f-7afd-419d-a01e-9ee6ca6f6f16")

#define MAG_FORMAT BLE2904::FORMAT_SINT16

#define MAG_EXPONENT -2

#define MAG_UNIT BLEUnit::uTelsa

#define MAG_X_NAME "Magnetic flux density (X)"
#define MAG_Y_NAME "Magnetic flux density (Y)"
#define MAG_Z_NAME "Magnetic flux density (Z)"

static BLECharacteristic m_magXCharacteristic(MAG_X_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_magYCharacteristic(MAG_Y_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_magZCharacteristic(MAG_Z_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

static BLEWrapper m_magXWrapper(&m_magXCharacteristic, MAG_X_NAME, MAG_FORMAT, MAG_EXPONENT, MAG_UNIT);
static BLEWrapper m_magYWrapper(&m_magYCharacteristic, MAG_Y_NAME, MAG_FORMAT, MAG_EXPONENT, MAG_UNIT);
static BLEWrapper m_magZWrapper(&m_magZCharacteristic, MAG_Z_NAME, MAG_FORMAT, MAG_EXPONENT, MAG_UNIT);

static Adafruit_LSM9DS1 m_sensor = Adafruit_LSM9DS1();
static unsigned long m_lastTime;
static bool m_ready = false;

bool lsm9ds1_init(void) {
  if (!m_sensor.begin()) {
    ERROR("Could not initialise sensor");
    return false;
  }

  m_sensor.setupMag(m_sensor.LSM9DS1_MAGGAIN_4GAUSS);
  m_lastTime = millis();
  m_ready = true;
  return true;
}

bool lsm9ds1_addService(BLEServer *pServer) {
  if (!m_ready) {
    return false;
  }

  int numHandles = BLEWrapper::calcNumHandles(NUM_CHARACTERISTICS);
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID, numHandles, BLE_INST_ID);
  if (pService == NULL) {
    ERROR("Cannot add BLE service");
    m_ready = false;
    return false;
  }

  pService->addCharacteristic(&m_magXCharacteristic);
  pService->addCharacteristic(&m_magYCharacteristic);
  pService->addCharacteristic(&m_magZCharacteristic);

  pService->start();
  return true;
}

void lsm9ds1_loop(void) {
  unsigned long now = millis();
  if (m_ready && (now - m_lastTime >= SAMPLE_TIME)) {
    m_lastTime = now;

    sensors_event_t a, m, g, t;
    if (!m_sensor.getEvent(&a, &m, &g, &t)) {
      ERROR("Error reading sensor");
      return;
    }

    m_magXWrapper.writeValue(m.magnetic.x);
    m_magYWrapper.writeValue(m.magnetic.y);
    m_magZWrapper.writeValue(m.magnetic.z);
  }
}
