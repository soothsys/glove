/*
 * This file includes code which is redistributed under the following copyright license:
 *
 * Rui Santos
 * Complete project details at https://RandomNerdTutorials.com/esp32-ble-server-environmental-sensing-service/
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files. 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#define ERR_MODULE_NAME "BME688"

#include <Wire.h>
#include <bsec2.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "i2c_address.h"
#include "err.h"

#define	SAMPLE_RATE					BSEC_SAMPLE_RATE_LP
#define TEMP_OFFSET					TEMP_OFFSET_LP

#define BLE_SERVICE_UUID			0x181A
#define DESCRIPTOR_UUID				0x2902
#define TEMP_CHARACTERISTIC_UUID	0x2A6E
#define HUM_CHARACTERISTIC_UUID		0x2A6F
#define PRES_CHARACTERISTIC_UUID	0x2A6D
#define TEMP_SCALE    				100.0f
#define HUM_SCALE   				100.0f
#define PRES_SCALE  				100.0f

BLECharacteristic m_tempCharacteristic(BLEUUID((uint16_t)TEMP_CHARACTERISTIC_UUID), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_humCharacteristic(BLEUUID((uint16_t)HUM_CHARACTERISTIC_UUID), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_presCharacteristic(BLEUUID((uint16_t)PRES_CHARACTERISTIC_UUID), BLECharacteristic::PROPERTY_NOTIFY);

BLEDescriptor m_tempDescriptor(BLEUUID((uint16_t)DESCRIPTOR_UUID));
BLEDescriptor m_humDescriptor(BLEUUID((uint16_t)DESCRIPTOR_UUID));
BLEDescriptor m_presDescriptor(BLEUUID((uint16_t)DESCRIPTOR_UUID));

Bsec2 m_envSensor;

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec); //Hack gets around a type definition error in the library

static bool handleError(const char *message) {
	int libStatus = m_envSensor.status;
	int sensorStatus = m_envSensor.sensor.status;
	bool isError = (libStatus < BSEC_OK) || (sensorStatus < BME68X_OK);
	bool isWarning = (libStatus > BSEC_OK) || (sensorStatus > BME68X_OK);

	if (isError) {
		ERROR("Error %s", message);
	} else if (isWarning) {
		ERROR("Warning %s", message);
	}

	if (isError || isWarning) {
		ERROR("BSEC library status code: %d", libStatus);
		ERROR("Sensor status code: %d", sensorStatus);
	}

	return !isError;
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
	int i;
	for (i = 0; i < outputs.nOutputs; i++) {
		const bsecData output = outputs.output[i];
		BLECharacteristic *pCharacteristic = NULL;
		float scaleFactor = 1.0f;

		switch (output.sensor_id) {
			case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
				pCharacteristic = &m_tempCharacteristic;
				scaleFactor = TEMP_SCALE;
				break;
			
			case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
				pCharacteristic = &m_humCharacteristic;
				scaleFactor = HUM_SCALE;
				break;
				
			case BSEC_OUTPUT_RAW_PRESSURE:
				pCharacteristic = &m_presCharacteristic;
				scaleFactor = PRES_SCALE;
				break;
				
			default:
				break;
		}
		
		if (pCharacteristic != NULL) {
			uint16_t uval = (uint16_t)(output.signal * scaleFactor);
			pCharacteristic->setValue(uval);
			pCharacteristic->notify();
		}
	}
}

bool bme688_init(i2c_address_t addr) {
	bsecSensor sensorList[] = {
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
		BSEC_OUTPUT_RAW_PRESSURE/*,							//No compensated pressure option exists
		BSEC_OUTPUT_IAQ,									//Raw IAQ measurement
		BSEC_OUTPUT_STATIC_IAQ,								//"Delta" IAQ measurement relative to running average, intended to show changes in air quality against "normal" value for location
		BSEC_OUTPUT_CO2_EQUIVALENT,
		BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
		BSEC_OUTPUT_STABILIZATION_STATUS,					//0 = stabilisation in progress, 1 = stabilisation finished
		BSEC_OUTPUT_RUN_IN_STATUS							//0 = run-in in progress, 1 = run-in finished*/
	};

	if (!m_envSensor.begin(addr, Wire)) {
		if (!handleError("initialising sensor")) {
			return false;
		}
	}
	
	m_envSensor.setTemperatureOffset(TEMP_OFFSET);
	if (!m_envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), SAMPLE_RATE)) {
		if (!handleError("subscribing to data outputs")) {
			return false;
		}
	}
	
	m_envSensor.attachCallback(newDataCallback);
	return true;
}

bool bme688_addServices(BLEServer *pServer) {
	BLEService *pService = pServer->createService(BLEUUID((uint16_t)BLE_SERVICE_UUID));
	if (pService == NULL) {
		ERROR("Cannot add BLE service");
		return false;
	}
	
	pService->addCharacteristic(&m_tempCharacteristic);
	pService->addCharacteristic(&m_humCharacteristic);
	pService->addCharacteristic(&m_presCharacteristic);
	m_tempCharacteristic.addDescriptor(&m_tempDescriptor);
	m_humCharacteristic.addDescriptor(&m_humDescriptor);
	m_presCharacteristic.addDescriptor(&m_presDescriptor);
	
	pService->start();
	return true;
}

void bme688_loop(void) {
	if (!m_envSensor.run()) {
		handleError("reading sensor data");
	}
}
