/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#define ERR_MODULE_NAME "BME688"

#include <Wire.h>
#include <bsec2.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "blewrapper.h"
#include "i2c_address.h"
#include "err.h"

#define SAMPLE_RATE BSEC_SAMPLE_RATE_CONT
#define TEMP_OFFSET TEMP_OFFSET_LP

#define BLE_INST_ID 0
#define NUM_CHARACTERISTICS 9

#define BLE_SERVICE_UUID BLEUUID((uint16_t)0x181A)
#define TEMP_UUID BLEUUID((uint16_t)0x2A6E)
#define HUM_UUID BLEUUID((uint16_t)0x2A6F)
#define PRES_UUID BLEUUID((uint16_t)0x2A6D)
#define IAQ_UUID "b52338a6-b7fa-47d9-8db4-dbb86ac6b05c"  //Custom UUID - IAQ does not exist in BLE spec
#define SIAQ_UUID "0d1ab684-14a4-479b-9dcd-86b6fc2e99fa"
#define CO2_UUID BLEUUID((uint16_t)0x2B8C)
#define BVOC_UUID BLEUUID((uint16_t)0x2BE7)
#define STAB_UUID "313fe0fb-3844-4ecb-a356-714248c9861f"
#define RUNIN_UUID "8e9a5a91-be3f-445a-af3c-c6db247cb975"

#define TEMP_FORMAT BLE2904::FORMAT_SINT16
#define HUM_FORMAT BLE2904::FORMAT_UINT16
#define PRES_FORMAT BLE2904::FORMAT_UINT32
#define IAQ_FORMAT BLE2904::FORMAT_UINT16
#define SIAQ_FORMAT BLE2904::FORMAT_UINT16
#define CO2_FORMAT BLE2904::FORMAT_UINT16
#define BVOC_FORMAT BLE2904::FORMAT_UINT16
#define STAB_FORMAT BLE2904::FORMAT_BOOLEAN
#define RUNIN_FORMAT BLE2904::FORMAT_BOOLEAN

#define TEMP_EXPONENT -2
#define HUM_EXPONENT -2
#define PRES_EXPONENT -1
#define IAQ_EXPONENT 0
#define SIAQ_EXPONENT 0
#define CO2_EXPONENT 0
#define BVOC_EXPONENT 0
#define STAB_EXPONENT 0
#define RUNIN_EXPONENT 0

#define TEMP_UNIT BLEUnit::DegC
#define HUM_UNIT BLEUnit::Percent
#define PRES_UNIT BLEUnit::Pascal
#define IAQ_UNIT BLEUnit::Unitless
#define SIAQ_UNIT BLEUnit::Unitless
#define CO2_UNIT BLEUnit::PPM
#define BVOC_UNIT BLEUnit::PPM
#define STAB_UNIT BLEUnit::Unitless
#define RUNIN_UNIT BLEUnit::Unitless

#define TEMP_NAME "Temperature"
#define HUM_NAME "Humidity"
#define PRES_NAME "Pressure"
#define IAQ_NAME "Index of air quality (adjusted)"
#define SIAQ_NAME "Index of air quality (raw)"
#define CO2_NAME "CO2 concentration"
#define BVOC_NAME "Breath VOC concentration"
#define STAB_NAME "Stabilised"
#define RUNIN_NAME "Run in"

#define TEMP_SCALE 1.0f
#define HUM_SCALE 1.0f
#define PRES_SCALE 100.0f //Sensor reports pressure in hPa, BLE reports pressure in Pa (1hPa = 100Pa)
#define IAQ_SCALE 1.0f
#define SIAQ_SCALE 1.0f
#define CO2_SCALE 1.0f
#define BVOC_SCALE 1.0f

static BLECharacteristic m_tempCharacteristic(TEMP_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_humCharacteristic(HUM_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_presCharacteristic(PRES_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_iaqCharacteristic(IAQ_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_siaqCharacteristic(SIAQ_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_co2Characteristic(CO2_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_bvocCharacteristic(BVOC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_stabCharacteristic(STAB_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_runinCharacteristic(RUNIN_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

static BLEWrapper m_tempWrapper(&m_tempCharacteristic, TEMP_NAME, TEMP_FORMAT, TEMP_EXPONENT, TEMP_UNIT);
static BLEWrapper m_humWrapper(&m_humCharacteristic, HUM_NAME, HUM_FORMAT, HUM_EXPONENT, HUM_UNIT);
static BLEWrapper m_presWrapper(&m_presCharacteristic, PRES_NAME, PRES_FORMAT, PRES_EXPONENT, PRES_UNIT);
static BLEWrapper m_iaqWrapper(&m_iaqCharacteristic, IAQ_NAME, IAQ_FORMAT, IAQ_EXPONENT, IAQ_UNIT);
static BLEWrapper m_siaqWrapper(&m_siaqCharacteristic, SIAQ_NAME, SIAQ_FORMAT, SIAQ_EXPONENT, SIAQ_UNIT);
static BLEWrapper m_co2Wrapper(&m_co2Characteristic, CO2_NAME, CO2_FORMAT, CO2_EXPONENT, CO2_UNIT);
static BLEWrapper m_bvocWrapper(&m_bvocCharacteristic, BVOC_NAME, BVOC_FORMAT, BVOC_EXPONENT, BVOC_UNIT);
static BLEWrapper m_stabWrapper(&m_stabCharacteristic, STAB_NAME, STAB_FORMAT, STAB_EXPONENT, STAB_UNIT);
static BLEWrapper m_runinWrapper(&m_runinCharacteristic, RUNIN_NAME, RUNIN_FORMAT, RUNIN_EXPONENT, RUNIN_UNIT);

static Bsec2 m_envSensor;
static bool m_stabilised = false;
static bool m_runIn = false;
static bool m_ready = false;

static void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);  //Hack gets around a type definition error in the library

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

static void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
  int i;
  for (i = 0; i < outputs.nOutputs; i++) {
    const bsecData output = outputs.output[i];
    BLEWrapper *pWrapper = NULL;
    float scaleFactor = 1.0f;

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
      
      case BSEC_OUTPUT_STABILIZATION_STATUS:
        pWrapper = &m_stabWrapper;
        break;
      
      case BSEC_OUTPUT_RUN_IN_STATUS:
        pWrapper = &m_runinWrapper;
        break;

      default:
        break;
    }

    if (pWrapper != NULL) {
      BLECharacteristic *pCharacteristic = pWrapper->getCharacteristic();
      if (pCharacteristic != NULL) {
        pWrapper->writeValue(scaleFactor * output.signal);
      }
    }
  }
}

bool bme688_init(i2c_address_t addr) {
  bsecSensor sensorList[] = {
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_RAW_PRESSURE,  //No compensated pressure option exists
    BSEC_OUTPUT_IAQ,           //Raw IAQ measurement
    BSEC_OUTPUT_STATIC_IAQ,    //"Delta" IAQ measurement relative to running average, intended to show changes in air quality against "normal" value for location
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_STABILIZATION_STATUS,  //0 = stabilisation in progress, 1 = stabilisation finished
    BSEC_OUTPUT_RUN_IN_STATUS          //0 = run-in in progress, 1 = run-in finished
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
  m_ready = true;
  return true;
}

bool bme688_addService(BLEServer *pServer) {
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

  pService->addCharacteristic(&m_tempCharacteristic);
  pService->addCharacteristic(&m_humCharacteristic);
  pService->addCharacteristic(&m_presCharacteristic);
  pService->addCharacteristic(&m_iaqCharacteristic);
  pService->addCharacteristic(&m_siaqCharacteristic);
  pService->addCharacteristic(&m_co2Characteristic);
  pService->addCharacteristic(&m_bvocCharacteristic);
  pService->addCharacteristic(&m_stabCharacteristic);
  pService->addCharacteristic(&m_runinCharacteristic);

  pService->start();
  return true;
}

void bme688_loop(void) {
  if (m_ready && !m_envSensor.run()) {
    handleError("reading sensor data");
  }
}
