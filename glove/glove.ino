/*
 * This file includes code which is redistributed under the following copyright license:
 *
 * Rui Santos
 * Complete project details at https://RandomNerdTutorials.com/esp32-ble-server-environmental-sensing-service/
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files. 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#define ERR_MODULE_NAME "Core"

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "err.h"
#include "i2c_address.h"
#include "powermgmt.h"
#include "battery.h"
#include "bme688.h"
#include "as7341.h"
#include "lsm9ds1.h"

#define PRINT_INTERVAL 1000000 //1 second in us
#define BAUD_RATE			 115200

#define BLE_SERVER_NAME		"SmartGlove"

static BLEAdvertising *pAdvert = NULL;
static bool m_deviceConnected = false;

static unsigned long m_lastLoopTime;
static unsigned long m_lastPrintTime;
static unsigned long m_minLoopLength;
static unsigned long m_maxLoopLength;
static unsigned long m_avgLoopLength;
static unsigned long m_numIterations;

class MyServerCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer *pServer) {
		m_deviceConnected = true;
		Serial.println("Client connected");
	}
	
	void onDisconnect(BLEServer *pServer) {
		m_deviceConnected = false;
		Serial.println("Client disconnected");
    if (pAdvert != NULL) {
      pAdvert->start(); //Resume advertising for next client
    }
	}
};

static void resetLoopStats(void) {
  m_minLoopLength = UINT32_MAX;
  m_maxLoopLength = 0;
  m_avgLoopLength = 0;
  m_numIterations = 0;
}

static void printLoopStats(void) {
  unsigned long avg = m_avgLoopLength / m_numIterations;
  Serial.print("Loop statistics: min = ");
  Serial.print(m_minLoopLength);
  Serial.print("us, max = ");
  Serial.print(m_maxLoopLength);
  Serial.print("us, avg = ");
  Serial.print(avg);
  Serial.println("us");
}

void setup(void) {
	err_init(); //Set up error LED pin
	Serial.begin(BAUD_RATE);
	Wire.begin();
	
  powermgmt_init();
  battery_init();

	int nSensors = 0;
	if (bme688_init(i2c_address_bme688)) {
		nSensors++;
	}
  if (as7341_init(i2c_address_as7341)) {
    nSensors++;
  }
  if (lsm9ds1_init()) {
    nSensors++;
  }

	if (nSensors == 0) {
		ERROR_HALT("Failed to initialise any sensors");
	}
	
	BLEDevice::init(BLE_SERVER_NAME);
	BLEServer *pServer = BLEDevice::createServer();
	if (pServer == NULL) {
		ERROR_HALT("Failed to create BLE server");
	}
	
	pServer->setCallbacks(new MyServerCallbacks());
  if (!battery_addService(pServer)) {
    ERROR("Failed to add battery monitor service");
  }
	if (!bme688_addService(pServer)) {
		ERROR("Failed to add BME688 service");
	}
  if (!as7341_addService(pServer)) {
    ERROR("Failed to add AS7341 service");
  }
  if (!lsm9ds1_addService(pServer)) {
    ERROR("Failed to add LSM9DS1 service");
  }
	
	pAdvert = pServer->getAdvertising();
	if (pAdvert == NULL) {
		ERROR_HALT("Failed to start advertising BLE services");
	}
	
	pAdvert->start();
	Serial.println("Waiting for client");

  resetLoopStats();
  m_lastLoopTime = m_lastPrintTime = micros();
}

void loop(void) {
  powermgmt_loop();
  
	if (m_deviceConnected) {
    battery_loop();
		bme688_loop();
    as7341_loop();
    lsm9ds1_loop();
	}

  unsigned long now = micros();
  unsigned long duration = now - m_lastLoopTime;
  m_lastLoopTime = now;
  m_avgLoopLength += duration;
  m_numIterations++;

  if (duration < m_minLoopLength) {
    m_minLoopLength = duration;
  }

  if (duration > m_maxLoopLength) {
    m_maxLoopLength = duration;
  }

  if (now - m_lastPrintTime >= PRINT_INTERVAL) {
    m_lastPrintTime = now;
    printLoopStats();
    resetLoopStats();
  }
}
