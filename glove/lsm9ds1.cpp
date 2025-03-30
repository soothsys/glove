/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#define ERR_MODULE_NAME "LSM9DS1"

#include <Wire.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "SensorFusion.h"

#include "blewrapper.h"
#include "err.h"

#define SAMPLE_TIME 1000 //milliseconds
#define BLE_INST_ID 0
#define NUM_CHARACTERISTICS 12

#define BLE_SERVICE_UUID BLEUUID("606a0692-1e69-422a-9f73-de87d239aade")
#define ACCEL_X_UUID BLEUUID("0436b72d-c94e-4cf8-93e0-60fb68c0f6dd")
#define ACCEL_Y_UUID BLEUUID("0b4c9db7-3d78-48b0-8015-27601c4eab25")
#define ACCEL_Z_UUID BLEUUID("10eb8627-99af-47a8-867b-f19712fab232")
#define MAG_X_UUID BLEUUID("14bbfa6b-347a-4cb1-ad8c-4c81cdc4259b")
#define MAG_Y_UUID BLEUUID("1ae54544-eeb9-46b9-89a4-6c23889d0ed3")
#define MAG_Z_UUID BLEUUID("21e5d780-5ff4-452e-8a29-6d04a8f004a5")
#define GYRO_X_UUID BLEUUID("350a3ecf-2c8f-4d19-a2d8-f1b6d8302df0")
#define GYRO_Y_UUID BLEUUID("3b7856ae-eb8b-4733-83d9-85b1a49db875")
#define GYRO_Z_UUID BLEUUID("8628c9c7-81a8-44d8-a00a-72d241898c82")
#define PITCH_UUID BLEUUID("8fccbd0f-7afd-419d-a01e-9ee6ca6f6f16")
#define ROLL_UUID BLEUUID("acd5b86b-f7ed-42b3-82fe-96668ca32a08")
#define YAW_UUID BLEUUID("bb54840e-2907-40ce-bd38-5d967b66e036")

#define ACCEL_FORMAT BLE2904::FORMAT_SINT16
#define MAG_FORMAT BLE2904::FORMAT_SINT16
#define GYRO_FORMAT BLE2904::FORMAT_SINT16
#define ANGLE_FORMAT BLE2904::FORMAT_SINT16

#define ACCEL_EXPONENT -2
#define MAG_EXPONENT -2
#define GYRO_EXPONENT -2
#define ANGLE_EXPONENT -2

#define ACCEL_UNIT BLEUnit::MetresPerSecondSquared
#define MAG_UNIT BLEUnit::uTesla
#define GYRO_UNIT BLEUnit::RadsPerSecond
#define ANGLE_UNIT BLEUnit::Radian

#define ACCEL_X_NAME "Acceleration (X)"
#define ACCEL_Y_NAME "Acceleration (Y)"
#define ACCEL_Z_NAME "Acceleration (Z)"
#define MAG_X_NAME "Magnetic flux density (X)"
#define MAG_Y_NAME "Magnetic flux density (Y)"
#define MAG_Z_NAME "Magnetic flux density (Z)"
#define GYRO_X_NAME "Angular velocity (X)"
#define GYRO_Y_NAME "Angular velocity (Y)"
#define GYRO_Z_NAME "Angular velocity (Z)"
#define PITCH_NAME "Pitch"
#define ROLL_NAME "Roll"
#define YAW_NAME "Yaw"

static BLECharacteristic m_accelXCharacteristic(ACCEL_X_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_accelYCharacteristic(ACCEL_Y_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_accelZCharacteristic(ACCEL_Z_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_magXCharacteristic(MAG_X_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_magYCharacteristic(MAG_Y_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_magZCharacteristic(MAG_Z_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_gyroXCharacteristic(GYRO_X_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_gyroYCharacteristic(GYRO_Y_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_gyroZCharacteristic(GYRO_Z_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_pitchCharacteristic(PITCH_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_rollCharacteristic(ROLL_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_yawCharacteristic(YAW_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

static BLEWrapper m_accelXWrapper(&m_accelXCharacteristic, ACCEL_X_NAME, ACCEL_FORMAT, ACCEL_EXPONENT, ACCEL_UNIT);
static BLEWrapper m_accelYWrapper(&m_accelYCharacteristic, ACCEL_Y_NAME, ACCEL_FORMAT, ACCEL_EXPONENT, ACCEL_UNIT);
static BLEWrapper m_accelZWrapper(&m_accelZCharacteristic, ACCEL_Z_NAME, ACCEL_FORMAT, ACCEL_EXPONENT, ACCEL_UNIT);
static BLEWrapper m_magXWrapper(&m_magXCharacteristic, MAG_X_NAME, MAG_FORMAT, MAG_EXPONENT, MAG_UNIT);
static BLEWrapper m_magYWrapper(&m_magYCharacteristic, MAG_Y_NAME, MAG_FORMAT, MAG_EXPONENT, MAG_UNIT);
static BLEWrapper m_magZWrapper(&m_magZCharacteristic, MAG_Z_NAME, MAG_FORMAT, MAG_EXPONENT, MAG_UNIT);
static BLEWrapper m_gyroXWrapper(&m_gyroXCharacteristic, GYRO_X_NAME, GYRO_FORMAT, GYRO_EXPONENT, GYRO_UNIT);
static BLEWrapper m_gyroYWrapper(&m_gyroYCharacteristic, GYRO_Y_NAME, GYRO_FORMAT, GYRO_EXPONENT, GYRO_UNIT);
static BLEWrapper m_gyroZWrapper(&m_gyroZCharacteristic, GYRO_Z_NAME, GYRO_FORMAT, GYRO_EXPONENT, GYRO_UNIT);
static BLEWrapper m_pitchWrapper(&m_pitchCharacteristic, PITCH_NAME, ANGLE_FORMAT, ANGLE_EXPONENT, ANGLE_UNIT);
static BLEWrapper m_rollWrapper(&m_rollCharacteristic, ROLL_NAME, ANGLE_FORMAT, ANGLE_EXPONENT, ANGLE_UNIT);
static BLEWrapper m_yawWrapper(&m_yawCharacteristic, YAW_NAME, ANGLE_FORMAT, ANGLE_EXPONENT, ANGLE_UNIT);

static Adafruit_LSM9DS1 m_sensor = Adafruit_LSM9DS1();
static SF m_fusion;
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
  pService->addCharacteristic(&m_pitchCharacteristic);
  pService->addCharacteristic(&m_rollCharacteristic);
  pService->addCharacteristic(&m_yawCharacteristic);

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

    float deltaT = m_fusion.deltatUpdate();
    m_fusion.MadgwickUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, m.magnetic.x, m.magnetic.y, m.magnetic.z, deltaT);
    float pitch = m_fusion.getPitchRadians();
    float roll = m_fusion.getRollRadians();
    float yaw = m_fusion.getYawRadians();

    m_accelXWrapper.writeValue(a.acceleration.x);
    m_accelYWrapper.writeValue(a.acceleration.y);
    m_accelZWrapper.writeValue(a.acceleration.z);
    m_magXWrapper.writeValue(m.magnetic.x);
    m_magYWrapper.writeValue(m.magnetic.y);
    m_magZWrapper.writeValue(m.magnetic.z);
    m_gyroXWrapper.writeValue(g.gyro.x);
    m_gyroYWrapper.writeValue(g.gyro.y);
    m_gyroZWrapper.writeValue(g.gyro.z);
    m_pitchWrapper.writeValue(pitch);
    m_rollWrapper.writeValue(roll);
    m_yawWrapper.writeValue(yaw);
  }
}
