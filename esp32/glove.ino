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
#include "bme688.h"

#define BAUD_RATE			115200

#define BLE_SERVER_NAME		"SmartGlove"

bool m_deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer *pServer) {
		m_deviceConnected = true;
		Serial.println("Client connected");
	}
	
	void onDisconnect(BLEServer *pServer) {
		m_deviceConnected = false;
		Serial.println("Client disconnected");
	}
};

void setup(void) {
	err_init(); //Set up error LED pin
	Serial.begin(BAUD_RATE);
	Wire.begin();
	
	int nSensors = 0;
	if (bme688_init(i2c_address_bme688)) {
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
	if (!bme688_addServices(pServer)) {
		ERROR("Failed to add BME688 services");
	}
	
	BLEAdvertising *pAdvert = pServer->getAdvertising();
	if (pAdvert == NULL) {
		ERROR_HALT("Failed to start advertising BLE services");
	}
	
	pAdvert->start();
	Serial.println("Waiting for client");
}

void loop(void) {
	if (m_deviceConnected) {
		bme688_loop();
	}
}
