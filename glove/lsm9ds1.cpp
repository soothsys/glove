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
#define NUM_CHARACTERISTICS 9

#define BLE_SERVICE_UUID BLEUUID("606a0692-1e69-422a-9f73-de87d239aade")
#define ACCEL_X_UUID BLEUUID("0436b72d-c94e-4cf8-93e0-60fb68c0f6dd")
#define ACCEL_Y_UUID BLEUUID("10eb8627-99af-47a8-867b-f19712fab232")
#define ACCEL_Z_UUID BLEUUID("14bbfa6b-347a-4cb1-ad8c-4c81cdc4259b")
#define MAG_X_UUID BLEUUID("1ae54544-eeb9-46b9-89a4-6c23889d0ed3")
#define MAG_Y_UUID BLEUUID("21e5d780-5ff4-452e-8a29-6d04a8f004a5")
#define MAG_Z_UUID BLEUUID("350a3ecf-2c8f-4d19-a2d8-f1b6d8302df0")
#define GYRO_X_UUID BLEUUID("8628c9c7-81a8-44d8-a00a-72d241898c82")
#define GYRO_Y_UUID BLEUUID("8fccbd0f-7afd-419d-a01e-9ee6ca6f6f16")
#define GYRO_Z_UUID BLEUUID("acd5b86b-f7ed-42b3-82fe-96668ca32a08")

#define ACCEL_FORMAT BLE2904::FORMAT_SINT16
#define MAG_FORMAT BLE2904::FORMAT_SINT16
#define GYRO_FORMAT BLE2904::FORMAT_SINT16

#define ACCEL_EXPONENT -2
#define MAG_EXPONENT -2
#define GYRO_EXPONENT -2

#define ACCEL_UNIT BLEUnit::MetresPerSecondSquared
#define MAG_UNIT BLEUnit::uTelsa
#define GYRO_UNIT BLEUnit::RadsPerSecond

#define ACCEL_X_NAME "Acceleration (X)"
#define ACCEL_Y_NAME "Acceleration (Y)"
#define ACCEL_Z_NAME "Acceleration (Z)"
#define MAG_X_NAME "Magnetic flux density (X)"
#define MAG_Y_NAME "Magnetic flux density (Y)"
#define MAG_Z_NAME "Magnetic flux density (Z)"
#define GYRO_X_NAME "Angular velocity (X)"
#define GYRO_Y_NAME "Angular velocity (Y)"
#define GYRO_Z_NAME "Angular velocity (Z)"

static BLECharacteristic m_accelXCharacteristic(ACCEL_X_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_accelYCharacteristic(ACCEL_Y_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_accelZCharacteristic(ACCEL_Z_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_magXCharacteristic(MAG_X_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_magYCharacteristic(MAG_Y_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_magZCharacteristic(MAG_Z_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_gyroXCharacteristic(GYRO_X_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_gyroYCharacteristic(GYRO_Y_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_gyroZCharacteristic(GYRO_Z_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

static BLEWrapper m_accelXWrapper(&m_accelXCharacteristic, ACCEL_X_NAME, ACCEL_FORMAT, ACCEL_EXPONENT, ACCEL_UNIT);
static BLEWrapper m_accelYWrapper(&m_accelYCharacteristic, ACCEL_Y_NAME, ACCEL_FORMAT, ACCEL_EXPONENT, ACCEL_UNIT);
static BLEWrapper m_accelZWrapper(&m_accelZCharacteristic, ACCEL_Z_NAME, ACCEL_FORMAT, ACCEL_EXPONENT, ACCEL_UNIT);
static BLEWrapper m_magXWrapper(&m_magXCharacteristic, MAG_X_NAME, MAG_FORMAT, MAG_EXPONENT, MAG_UNIT);
static BLEWrapper m_magYWrapper(&m_magYCharacteristic, MAG_Y_NAME, MAG_FORMAT, MAG_EXPONENT, MAG_UNIT);
static BLEWrapper m_magZWrapper(&m_magZCharacteristic, MAG_Z_NAME, MAG_FORMAT, MAG_EXPONENT, MAG_UNIT);
static BLEWrapper m_gyroXWrapper(&m_gyroXCharacteristic, GYRO_X_NAME, GYRO_FORMAT, GYRO_EXPONENT, GYRO_UNIT);
static BLEWrapper m_gyroYWrapper(&m_gyroYCharacteristic, GYRO_Y_NAME, GYRO_FORMAT, GYRO_EXPONENT, GYRO_UNIT);
static BLEWrapper m_gyroZWrapper(&m_gyroZCharacteristic, GYRO_Z_NAME, GYRO_FORMAT, GYRO_EXPONENT, GYRO_UNIT);

static Adafruit_LSM9DS1 m_sensor = Adafruit_LSM9DS1();
static unsigned long m_lastTime;
static bool m_ready = false;

bool lsm9ds1_init(void) {
  if (!m_sensor.begin()) {
    ERROR("Could not initialise sensor");
    return false;
  }

  m_sensor.setupAccel(m_sensor.LSM9DS1_ACCELRANGE_2G, m_sensor.LSM9DS1_ACCELDATARATE_10HZ);
  m_sensor.setupMag(m_sensor.LSM9DS1_MAGGAIN_4GAUSS);
  m_sensor.setupGyro(m_sensor.LSM9DS1_GYROSCALE_245DPS);

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

  pService->addCharacteristic(&m_accelXCharacteristic);
  pService->addCharacteristic(&m_accelYCharacteristic);
  pService->addCharacteristic(&m_accelZCharacteristic);
  pService->addCharacteristic(&m_magXCharacteristic);
  pService->addCharacteristic(&m_magYCharacteristic);
  pService->addCharacteristic(&m_magZCharacteristic);
  pService->addCharacteristic(&m_gyroXCharacteristic);
  pService->addCharacteristic(&m_gyroYCharacteristic);
  pService->addCharacteristic(&m_gyroZCharacteristic);

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

    m_accelXWrapper.writeValue(a.acceleration.x);
    m_accelYWrapper.writeValue(a.acceleration.y);
    m_accelZWrapper.writeValue(a.acceleration.z);
    m_magXWrapper.writeValue(m.magnetic.x);
    m_magYWrapper.writeValue(m.magnetic.y);
    m_magZWrapper.writeValue(m.magnetic.z);
    m_gyroXWrapper.writeValue(g.gyro.x);
    m_gyroYWrapper.writeValue(g.gyro.y);
    m_gyroZWrapper.writeValue(g.gyro.z);
  }
}
