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

#include "blewrapper.h"
#include "i2c_address.h"
#include "err.h"

#define	SAMPLE_RATE 					BSEC_SAMPLE_RATE_CONT
#define TEMP_OFFSET	  				TEMP_OFFSET_LP

#define BLE_INST_ID           0
#define NUM_CHARACTERISTICS   7

#define BLE_SERVICE_UUID			BLEUUID((uint16_t)0x181A)
#define TEMP_UUID	            BLEUUID((uint16_t)0x2A6E)
#define HUM_UUID		          BLEUUID((uint16_t)0x2A6F)
#define PRES_UUID	            BLEUUID((uint16_t)0x2A6D)
#define IAQ_UUID              "b52338a6-b7fa-47d9-8db4-dbb86ac6b05c" //Custom UUID - IAQ does not exist in BLE spec
#define SIAQ_UUID             "0d1ab684-14a4-479b-9dcd-86b6fc2e99fa"
#define CO2_UUID              BLEUUID((uint16_t)0x2B8C)
#define BVOC_UUID             BLEUUID((uint16_t)0x2BE7)

#define TEMP_FORMAT           BLE2904::FORMAT_SINT16
#define HUM_FORMAT            BLE2904::FORMAT_UINT16
#define PRES_FORMAT           BLE2904::FORMAT_UINT32
#define IAQ_FORMAT            BLE2904::FORMAT_UINT16
#define SIAQ_FORMAT           BLE2904::FORMAT_UINT16
#define CO2_FORMAT            BLE2904::FORMAT_UINT16
#define BVOC_FORMAT           BLE2904::FORMAT_UINT16

#define TEMP_EXPONENT         -2
#define HUM_EXPONENT          -2
#define PRES_EXPONENT         -1
#define IAQ_EXPONENT          0
#define SIAQ_EXPONENT         0
#define CO2_EXPONENT          0
#define BVOC_EXPONENT         0

#define TEMP_UNIT             BLEUnit::DegC
#define HUM_UNIT              BLEUnit::Percent
#define PRES_UNIT             BLEUnit::Pascal
#define IAQ_UNIT              BLEUnit::Unitless
#define SIAQ_UNIT             BLEUnit::Unitless
#define CO2_UNIT              BLEUnit::PPM
#define BVOC_UNIT             BLEUnit::PPB

#define TEMP_NAME             "Temperature"
#define HUM_NAME              "Humidity"
#define PRES_NAME             "Pressure"
#define IAQ_NAME              "Index of air quality (adjusted)"
#define SIAQ_NAME             "Index of air quality (raw)"
#define CO2_NAME              "CO2 equivalent concentration"
#define BVOC_NAME             "Breath VOC equivalent concentration"

#define TEMP_SCALE            1.0f
#define HUM_SCALE             1.0f
#define PRES_SCALE            100.0f //Sensor reports pressure in hPa, BLE reports pressure in Pa (1hPa = 100Pa)
#define IAQ_SCALE             1.0f
#define SIAQ_SCALE            1.0f
#define CO2_SCALE             1.0f
#define BVOC_SCALE            1000.0f //Sensor reports BVOCs in PPM but converting to PPB for display as reading is so small

BLECharacteristic m_tempCharacteristic(TEMP_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_humCharacteristic(HUM_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_presCharacteristic(PRES_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_iaqCharacteristic(IAQ_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_siaqCharacteristic(SIAQ_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_co2Characteristic(CO2_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic m_bvocCharacteristic(BVOC_UUID, BLECharacteristic::PROPERTY_NOTIFY);

BLEWrapper m_tempWrapper(&m_tempCharacteristic, TEMP_NAME, TEMP_FORMAT, TEMP_EXPONENT, TEMP_UNIT);
BLEWrapper m_humWrapper(&m_humCharacteristic, HUM_NAME, HUM_FORMAT, HUM_EXPONENT, HUM_UNIT);
BLEWrapper m_presWrapper(&m_presCharacteristic, PRES_NAME, PRES_FORMAT, PRES_EXPONENT, PRES_UNIT);
BLEWrapper m_iaqWrapper(&m_iaqCharacteristic, IAQ_NAME, IAQ_FORMAT, IAQ_EXPONENT, IAQ_UNIT);
BLEWrapper m_siaqWrapper(&m_siaqCharacteristic, SIAQ_NAME, SIAQ_FORMAT, SIAQ_EXPONENT, SIAQ_UNIT);
BLEWrapper m_co2Wrapper(&m_co2Characteristic, CO2_NAME, CO2_FORMAT, CO2_EXPONENT, CO2_UNIT);
BLEWrapper m_bvocWrapper(&m_bvocCharacteristic, BVOC_NAME, BVOC_FORMAT, BVOC_EXPONENT, BVOC_UNIT);

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
    BLEWrapper *pWrapper = NULL;
    float scaleFactor;

		switch (output.sensor_id) {
			case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
				pWrapper = &m_tempWrapper;
        scaleFactor = TEMP_SCALE;
				break;
			
			case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
				pWrapper = &m_humWrapper;
        scaleFactor = HUM_SCALE;
				break;
				
			case BSEC_OUTPUT_RAW_PRESSURE:
				pWrapper = &m_presWrapper;
        scaleFactor = PRES_SCALE;
				break;

      case BSEC_OUTPUT_IAQ:
				pWrapper = &m_iaqWrapper;
        scaleFactor = IAQ_SCALE;
        break;

      case BSEC_OUTPUT_STATIC_IAQ:
				pWrapper = &m_siaqWrapper;
        scaleFactor = SIAQ_SCALE;
        break;
      
      case BSEC_OUTPUT_CO2_EQUIVALENT:
				pWrapper = &m_co2Wrapper;
        scaleFactor = CO2_SCALE;
        break;
      
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
				pWrapper = &m_bvocWrapper;
        scaleFactor = BVOC_SCALE;
        break;
				
			default:
				break;
		}
		
		if (pWrapper != NULL) {
      BLECharacteristic *pCharacteristic = pWrapper->getCharacteristic();
      if (pCharacteristic != NULL) {
		  	pWrapper->writeValue(scaleFactor * output.signal);
			  pCharacteristic->notify();
      }
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
  int numHandles = 1 + NUM_CHARACTERISTICS * BLEWrapper::NUM_HANDLES;
	BLEService *pService = pServer->createService(BLE_SERVICE_UUID, numHandles, BLE_INST_ID);
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
	
	pService->start();
	return true;
}

void bme688_loop(void) {
	if (!m_envSensor.run()) {
		handleError("reading sensor data");
	}
}
