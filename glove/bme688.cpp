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

#define	SAMPLE_RATE					BSEC_SAMPLE_RATE_CONT
#define TEMP_OFFSET					TEMP_OFFSET_LP

#define BLE_NUM_HANDLES       22 //Need more than the default 15 handles due to large number of characteristics
#define BLE_INST_ID           0
#define BLE_SERVICE_UUID			BLEUUID((uint16_t)0x181A)
#define DESCRIPTOR_UUID				BLEUUID((uint16_t)0x2902)
#define TEMP_CHARACTERISTIC_UUID	BLEUUID((uint16_t)0x2A6E)
#define HUM_CHARACTERISTIC_UUID		BLEUUID((uint16_t)0x2A6F)
#define PRES_CHARACTERISTIC_UUID	BLEUUID((uint16_t)0x2A6D)
#define IAQ_CHARACTERISTIC_UUID   "b52338a6-b7fa-47d9-8db4-dbb86ac6b05c" //Custom UUID - IAQ does not exist in BLE spec
#define SIAQ_CHARACTERISTIC_UUID  "0d1ab684-14a4-479b-9dcd-86b6fc2e99fa"
#define CO2_CHARACTERISTIC_UUID   BLEUUID((uint16_t)0x2B8C)
#define BVOC_CHARACTERISTIC_UUID   BLEUUID((uint16_t)0x2BE7)

BLECharacteristic m_tempCharacteristic(TEMP_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_humCharacteristic(HUM_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_presCharacteristic(PRES_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_iaqCharacteristic(IAQ_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_siaqCharacteristic(SIAQ_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_co2Characteristic(CO2_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_bvocCharacteristic(BVOC_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);

BLEDescriptor m_tempDescriptor(DESCRIPTOR_UUID);
BLEDescriptor m_humDescriptor(DESCRIPTOR_UUID);
BLEDescriptor m_presDescriptor(DESCRIPTOR_UUID);
BLEDescriptor m_iaqDescriptor(DESCRIPTOR_UUID);
BLEDescriptor m_siaqDescriptor(DESCRIPTOR_UUID);
BLEDescriptor m_co2Descriptor(DESCRIPTOR_UUID);
BLEDescriptor m_bvocDescriptor(DESCRIPTOR_UUID);

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

		switch (output.sensor_id) {
			case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
				pCharacteristic = &m_tempCharacteristic;
				break;
			
			case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
				pCharacteristic = &m_humCharacteristic;
				break;
				
			case BSEC_OUTPUT_RAW_PRESSURE:
				pCharacteristic = &m_presCharacteristic;
				break;

      case BSEC_OUTPUT_IAQ:
        pCharacteristic = &m_iaqCharacteristic;
        break;

      case BSEC_OUTPUT_STATIC_IAQ:
        pCharacteristic = &m_siaqCharacteristic;
        break;
      
      case BSEC_OUTPUT_CO2_EQUIVALENT:
        pCharacteristic = &m_co2Characteristic;
        break;
      
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
        pCharacteristic = &m_bvocCharacteristic;
        break;
				
			default:
				break;
		}
		
		if (pCharacteristic != NULL) {
      float val = output.signal;
			pCharacteristic->setValue(val);
			pCharacteristic->notify();
		}
	}
}

bool bme688_init(i2c_address_t addr) {
	bsecSensor sensorList[] = {
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
		BSEC_OUTPUT_RAW_PRESSURE,							//No compensated pressure option exists
		BSEC_OUTPUT_IAQ,									//Raw IAQ measurement
		BSEC_OUTPUT_STATIC_IAQ,								//"Delta" IAQ measurement relative to running average, intended to show changes in air quality against "normal" value for location
		BSEC_OUTPUT_CO2_EQUIVALENT,
		BSEC_OUTPUT_BREATH_VOC_EQUIVALENT/*,
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
	BLEService *pService = pServer->createService(BLE_SERVICE_UUID, BLE_NUM_HANDLES, BLE_INST_ID);
	if (pService == NULL) {
		ERROR("Cannot add BLE service");
		return false;
	}
	
	pService->addCharacteristic(&m_tempCharacteristic);
	pService->addCharacteristic(&m_humCharacteristic);
	pService->addCharacteristic(&m_presCharacteristic);
  pService->addCharacteristic(&m_iaqCharacteristic);
  pService->addCharacteristic(&m_siaqCharacteristic);
  pService->addCharacteristic(&m_co2Characteristic);
  pService->addCharacteristic(&m_bvocCharacteristic);
	m_tempCharacteristic.addDescriptor(&m_tempDescriptor);
	m_humCharacteristic.addDescriptor(&m_humDescriptor);
	m_presCharacteristic.addDescriptor(&m_presDescriptor);
  m_iaqCharacteristic.addDescriptor(&m_iaqDescriptor);
  m_siaqCharacteristic.addDescriptor(&m_siaqDescriptor);
  m_co2Characteristic.addDescriptor(&m_co2Descriptor);
  m_bvocCharacteristic.addDescriptor(&m_bvocDescriptor);
	
	pService->start();
	return true;
}

void bme688_loop(void) {
	if (!m_envSensor.run()) {
		handleError("reading sensor data");
	}
}
