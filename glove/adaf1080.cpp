/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 *
 * This module reads magnetic field strength data from the Analog Devices EVAL-ADAF1080SDZ evaluation board. The board contains:
 *
 *    - The ADAF1080 sensor itself (analog output)
 *    - An AD4002 analog-to-digital converter (SPI access)
 *    - A circuit to generate a defined magnetic field strength which can be used to prove the sensor. This is enabled by the DIAG_EN pin
 *
 * For more information see:
 *
 *    - EVAL-ADAF1080SDZ User Guide (includes schematic): https://www.analog.com/media/en/technical-documentation/user-guides/eval-adaf1080sdz-ug-2067.pdf
 *    - ADAF1080 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/adaf1080.pdf
 *    - AD4002 datasheet (includes SPI protocol): https://www.analog.com/media/en/technical-documentation/data-sheets/ad4002-4006-4010.pdf
 */

#define ERR_MODULE_NAME "ADAF1080"

#include <float.h>
#include <SPI.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "blewrapper.h"
#include "err.h"

typedef struct {
  uint8_t enStatusBits      : 1;
  uint8_t enSpanCompression : 1;
  uint8_t enHighZMode       : 1;
  uint8_t enTurboMode       : 1;
} ad4002_cfg_t;

#define AD4002_REG_READ_CMD 0x54U //0b01010100
#define AD4002_REG_WRITE_CMD 0x14U //0b00010100
#define AD4002_EN_STATUS_BITS (1U << 4U)
#define AD4002_EN_SPAN_COMPRESSION (1U << 3U)
#define AD4002_EN_HIGH_Z_MODE (1U << 2U)
#define AD4002_EN_TURBO_MODE (1U << 1U)

/*
 * Calculated by inverting equation 7 from ADAF1080 datasheet:
 *
 *    Output code = Bsense * Sdevice * 2^n / 5 + Midcode
 *
 *    Bsense * Sdevice * 2^n / 5 = Output code - Midcode
 *
 *    Bsense = 5 * (Output code - Midcode) / (Sdevice * 2^n)
 *           = K * (Output code - Midcode)
 *
 * where    Bsense = magnetic field strength
 *         Sdevice = device sensitivity
 *                 = 199.25 mV/mT (for +/-8mT range, gain = 80)
 *                 = 0.00019925 V/uT
 *               n = ADC bit depth
 *                 = 18 bits
 *         Midcode = 2^(n - 1)
 *                 = 2^(18 - 1)
 *                 = 131072
 *
 * Therefore   K = 5 / (Sdevice * 2^n)
 *               = 5 / (0.00019925 * 2^18)
 *               = 0.095726
 */
#define ADAF1080_SCALE_FACTOR 0.095726F
#define AD4002_MIDCODE 131072

#define SPI_CLOCK_RATE 1000000 //1MHz
#define SPI_BIT_ORDER MSBFIRST
#define SPI_MODE SPI_MODE0

#define PIN_DIAG_EN 27
#define PIN_FLIP_DRV 33
#define PIN_CNV A5

/*
 * Diag coil produces approx. -18uT field strength based on empirical measurments. This is less than datasheet value of 22.8uT (for 100mA drive).
 * Upper and lower limits allow for noise, as 18uT is close to noise floor even with averaging.
 */
#define DIAG_FIELD_MIN -20.0F
#define DIAG_FIELD_MAX -16.0F

#define STARTUP_DELAY 50 //ms
#define FLIP_DELAY 1 //ms
#define DIAG_DELAY 100 //us
#define SAMPLE_TIME 4000 //4000us = 250Hz
#define NUM_SAMPLES 250 //Calculate statistics every second
#define CAL_AVERAGE_SAMPLES 32 //Average over multiple samples during calibration process to reduce noise
#define SAT_AVERAGE_SAMPLES 8

#define BLE_INST_ID 0
#define NUM_CHARACTERISTICS 9

#define BLE_SERVICE_UUID BLEUUID("7749eb1b-2b16-4d32-8422-e792dae7adb8")
#define CALIBRATE_UUID BLEUUID("0b541f35-34c1-4769-b206-8deaaa7e0922")
#define SATURATED_UUID BLEUUID("3c510d3d-3d82-4fd9-9dd3-da928916662b")
#define OFFSET_UUID BLEUUID("3c70df7e-3b42-4e52-bbdb-ff47728bec8a")
#define AVG_UUID BLEUUID("5fd8a802-0645-492f-bb0e-541972833add")
#define RMS_UUID BLEUUID("949b3518-826e-4a4b-b638-fea08b01e1a0")
#define PK_UUID BLEUUID("9c60f79f-18e0-4343-967e-b6474b305c8b")
#define PP_UUID BLEUUID("eea8f3a7-d5b1-4454-8e5b-44ce3c0fb372")
#define MIN_UUID BLEUUID("f3303f8c-89f4-4020-9912-de79a9617da1")
#define MAX_UUID BLEUUID("fc13446a-8329-4a00-8b74-6119d1129485")

#define CALIBRATE_FORMAT BLE2904::FORMAT_BOOLEAN
#define SATURATED_FORMAT BLE2904::FORMAT_BOOLEAN
#define MAGFIELD_FORMAT BLE2904::FORMAT_SINT32

#define CALIBRATE_EXPONENT 0
#define SATURATED_EXPONENT 0
#define MAGFIELD_EXPONENT -2 //10nT precision

#define CALIBRATE_UNIT BLEUnit::Unitless
#define SATURATED_UNIT BLEUnit::Unitless
#define MAGFIELD_UNIT BLEUnit::uTesla

#define CALIBRATE_NAME "Calibrate sensor"
#define SATURATED_NAME "Sensor saturated"
#define OFFSET_NAME "Sensor offset correction"
#define AVG_NAME "Average (DC)"
#define RMS_NAME "Root mean square (AC RMS)"
#define PK_NAME "Peak"
#define PP_NAME "Peak-to-peak"
#define MIN_NAME "Minimum"
#define MAX_NAME "Maximum"

static BLECharacteristic m_calibrateCharacteristic(CALIBRATE_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_saturatedCharacteristic(SATURATED_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_offsetCharacteristic(OFFSET_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_avgCharacteristic(AVG_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_rmsCharacteristic(RMS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_pkCharacteristic(PK_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_ppCharacteristic(PP_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_minCharacteristic(MIN_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_maxCharacteristic(MAX_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

static BLEWrapper m_calibrateWrapper(&m_calibrateCharacteristic, CALIBRATE_NAME, CALIBRATE_FORMAT, CALIBRATE_EXPONENT, CALIBRATE_UNIT);
static BLEWrapper m_saturatedWrapper(&m_saturatedCharacteristic, SATURATED_NAME, SATURATED_FORMAT, SATURATED_EXPONENT, SATURATED_UNIT);
static BLEWrapper m_offsetWrapper(&m_offsetCharacteristic, OFFSET_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_avgWrapper(&m_avgCharacteristic, AVG_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_rmsWrapper(&m_rmsCharacteristic, RMS_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_pkWrapper(&m_pkCharacteristic, PK_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_ppWrapper(&m_ppCharacteristic, PP_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_minWrapper(&m_minCharacteristic, MIN_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_maxWrapper(&m_maxCharacteristic, MAX_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);

static unsigned long m_lastTime;
static bool m_ready = false;
static volatile bool m_requestCalibration = false;
static int32_t m_offsetCorrection = 0; //Offset correction factor measured in ADC counts

static int m_sampleCount;
static float m_minValue;
static float m_maxValue;
static double m_avgAccum; //Use double precision for accumulators to prevent "catastrophic cancellation" error when calculating ACRMS = sqrt(RMS^2 - Average^2)
static double m_rmsAccum;

class CalibrateCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic == NULL) {
      return;
    }

    size_t dataLen = pCharacteristic->getLength();
    if (dataLen < 1) {
      return;
    }

    uint8_t *pData = pCharacteristic->getData();
    if (pData[0]) { //Client writes '1' to start calibration
      m_requestCalibration = true; //Callback runs in different thread. Can't run calibration from here for thread safety so request main thread to do it instead
    }
  }
};

static void ad4002_writeConfig(ad4002_cfg_t cfg) {
  /*
   * AD4002 config register write requires writing 16 bits and reading 18 bits, but Arduino only allows SPI transactions in multiples of 8 bits.
   * Pad transaction to 24 bits by stuffing with 1s so that MOSI idles HIGH.
   */
  uint8_t buffer[] = { AD4002_REG_WRITE_CMD, 0x01U, 0xFFU }; //Last bit of config word must be 1
  if (cfg.enStatusBits) { buffer[1] |= AD4002_EN_STATUS_BITS; }
  if (cfg.enSpanCompression) { buffer[1] |= AD4002_EN_SPAN_COMPRESSION; }
  if (cfg.enHighZMode) { buffer[1] |= AD4002_EN_HIGH_Z_MODE; }
  if (cfg.enTurboMode) { buffer[1] |= AD4002_EN_TURBO_MODE; }

  digitalWrite(PIN_CNV, HIGH); //Generate CNV pulse. tCNVH assured by MCU clock speed
  digitalWrite(PIN_CNV, LOW);

  SPI.beginTransaction(SPISettings(SPI_CLOCK_RATE, SPI_BIT_ORDER, SPI_MODE));
  SPI.transfer(buffer, 3);
  SPI.endTransaction();
}

static ad4002_cfg_t ad4002_readConfig(void) {
  uint8_t buffer[] = { AD4002_REG_READ_CMD, 0xFFU }; //Trailing 8 transmitted bits should be 1s

  digitalWrite(PIN_CNV, HIGH); //Generate CNV pulse. tCNVH assured by MCU clock speed
  digitalWrite(PIN_CNV, LOW);

  SPI.beginTransaction(SPISettings(SPI_CLOCK_RATE, SPI_BIT_ORDER, SPI_MODE));
  SPI.transfer(buffer, 2);
  SPI.endTransaction();

  ad4002_cfg_t cfg = { false, false, false, false };
  if (buffer[1] & AD4002_EN_STATUS_BITS) { cfg.enStatusBits = true; }
  if (buffer[1] & AD4002_EN_SPAN_COMPRESSION) { cfg.enSpanCompression = true; }
  if (buffer[1] & AD4002_EN_HIGH_Z_MODE) { cfg.enHighZMode = true; }
  if (buffer[1] & AD4002_EN_TURBO_MODE) { cfg.enTurboMode = true; }
  return cfg;
}

static uint32_t ad4002_readResult(void) {
  /*
   * AD4002 data read requires reading 18 bits, but Arduino only allows SPI transactions in multiples of 8 bits. Read 24 bits but discard 6 LSBs.
   */
  uint8_t buffer[] = { 0xFF, 0xFF, 0xFF }; //MOSI should be kept high during read i.e. transmit all 1s

  digitalWrite(PIN_CNV, HIGH); //Generate CNV pulse
  delayMicroseconds(1); //tCONV
  digitalWrite(PIN_CNV, LOW);

  SPI.beginTransaction(SPISettings(SPI_CLOCK_RATE, SPI_BIT_ORDER, SPI_MODE));
  SPI.transfer(buffer, 3);
  SPI.endTransaction();

  uint32_t result = buffer[0] << 10;
  result |= buffer[1] << 2;
  result |= buffer[2] >> 6; //Only use 2 MSBs of last byte, discarding 6 LSBs
  return result;
}

static float ad4002_readAverage(int nSamples) {
  float accum = 0.0f;
  int i;
  for (i = 0; i < nSamples; i++) {
    accum += (float)ad4002_readResult();
  }

  return accum / (float)nSamples;
}

static void resetStatistics(void) {
  m_sampleCount = 0;
  m_minValue = FLT_MAX;
  m_maxValue = -FLT_MAX;
  m_avgAccum = m_rmsAccum = 0.0f;
}

static float readSensor(void) {
  uint32_t adcCounts = ad4002_readResult(); //18 bit unipolar ADC result
  int32_t bipolar = (int32_t)adcCounts - AD4002_MIDCODE; //18 bit bipolar ADC result i.e symmetrical about 0
  return (float)(bipolar - m_offsetCorrection) * ADAF1080_SCALE_FACTOR;
}

static float readSensorAverage(int nSamples) {
  float avgAdcCounts = ad4002_readAverage(nSamples);
  float totalOffset = (float)(AD4002_MIDCODE + m_offsetCorrection);
  return (avgAdcCounts - totalOffset) * ADAF1080_SCALE_FACTOR;
}

static float calibrateSensor(void) {
  /*
   * See ADAF1080 datasheet page 27 for details of offset correction
   */
  digitalWrite(PIN_FLIP_DRV, HIGH); //Flip sensor in both directions so that we guarantee at least one flip regardless which direction we started in
  delay(FLIP_DELAY);
  digitalWrite(PIN_FLIP_DRV, LOW);
  delay(FLIP_DELAY);
  float negReading = ad4002_readAverage(CAL_AVERAGE_SAMPLES); //Sensor is now in negative polarity

  digitalWrite(PIN_FLIP_DRV, HIGH); //Flip sensor back to positive polarity
  delay(FLIP_DELAY);
  float posReading = ad4002_readAverage(CAL_AVERAGE_SAMPLES);
  
  float fOffsetCorrection = posReading - negReading;
  m_offsetCorrection = (int32_t)round(fOffsetCorrection); //AD4002 output is only 18-bit so we don't have to worry about overflowing 32-bit integer

  resetStatistics(); //Previously gathered statistics are now invalid due to change of offset, start from scratch
  return fOffsetCorrection * ADAF1080_SCALE_FACTOR; //We now know the sensor offset in raw ADC counts. Convert that back to uTesla for reporting
}

static bool isSensorSaturated(void) {
  /*
   * See EVAL-ADAF1080SGZ board includes a "diagnostic coil" feature, which passes a known current through the ADAF1080 IC's leadframe when the DIAG_EN pin is high.
   * This produces a known magnetic field directly within the sensor package.
   *
   * We take a pair of measurements in quick succession - one with the diagnostic coil on, and a second with it off. The difference between these measurements should
   * be the diagnostic coil field (plus some noise, which is reduced by averaging across multiple samples). If the sensor is functioning properly, this should be a 
   * close match to the datasheet predicted value.
   *
   * If the sensor is saturated, it is no longer responding properly to magnetic fields. We should see that the field strength does not change significantly whether 
   * the diagnostic coil is on or off.
   *
   * See ADAF1080 datasheet page 23 for more details.
   */
  digitalWrite(PIN_DIAG_EN, HIGH); //Turn diag coil on
  delayMicroseconds(DIAG_DELAY);
  float diagOn = readSensorAverage(SAT_AVERAGE_SAMPLES);

  digitalWrite(PIN_DIAG_EN, LOW); //Turn diag coil off
  delayMicroseconds(DIAG_DELAY);
  float diagOff = readSensorAverage(SAT_AVERAGE_SAMPLES);

  float diff = diagOn - diagOff;
  return (diff <= DIAG_FIELD_MIN) || (diff >= DIAG_FIELD_MAX); //If measured field strength change is outside of limits, sensor is likely saturated
}

bool adaf1080_init(void) {
  pinMode(PIN_FLIP_DRV, OUTPUT);
  digitalWrite(PIN_FLIP_DRV, LOW); //Start with FLIP_DRV low ready to generate positive edge
  pinMode(PIN_DIAG_EN, OUTPUT);
  digitalWrite(PIN_DIAG_EN, LOW); //Turn diag coil off
  pinMode(PIN_CNV, OUTPUT);
  digitalWrite(PIN_CNV, LOW); //CNV should idle low between transactions

  delay(STARTUP_DELAY); //Allow boost converter to start
  digitalWrite(PIN_FLIP_DRV, HIGH); //Flip sensor back to positive direction
  SPI.begin();

  ad4002_cfg_t cfg = {
    .enStatusBits = true,
    .enSpanCompression = false,
    .enHighZMode = true,
    .enTurboMode = false
  };

  /*
   * Write config register, then read back and confirm results match what we just wrote. This proves that AD4002 is connected and functioning
   */
  ad4002_writeConfig(cfg);
  ad4002_cfg_t readBack = ad4002_readConfig();
  if ((readBack.enStatusBits != cfg.enStatusBits) ||
      (readBack.enSpanCompression != cfg.enSpanCompression) ||
      (readBack.enHighZMode != cfg.enHighZMode) ||
      (readBack.enTurboMode != cfg.enTurboMode)) {
    ERROR("Could not initialise sensor");
    return false;
  }

  /*
   * ESP32 SPI driver sets MOSI pin low between transactions. AD4002 doesn't like this. As we don't need to transmit to read the sensor, we can just force MOSI high
   */
  pinMode(MOSI, OUTPUT);
  digitalWrite(MOSI, HIGH);

  resetStatistics(); //Initialise counters for statistical measurement
  m_lastTime = millis();
  m_ready = true;
  return true;
}

bool adaf1080_addService(BLEServer *pServer) {
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

  pService->addCharacteristic(&m_calibrateCharacteristic);
  pService->addCharacteristic(&m_saturatedCharacteristic);
  pService->addCharacteristic(&m_offsetCharacteristic);
  pService->addCharacteristic(&m_avgCharacteristic);
  pService->addCharacteristic(&m_rmsCharacteristic);
  pService->addCharacteristic(&m_pkCharacteristic);
  pService->addCharacteristic(&m_ppCharacteristic);
  pService->addCharacteristic(&m_minCharacteristic);
  pService->addCharacteristic(&m_maxCharacteristic);
  m_calibrateCharacteristic.setCallbacks(new CalibrateCallbacks());
  pService->start();

  uint8_t temp = 0;
  m_calibrateCharacteristic.setValue(&temp, 1);
  m_calibrateCharacteristic.notify();
  return true;
}

void adaf1080_loop(void) {
  if (m_ready && m_requestCalibration) { //BTC_TASK thread has requsted calibration
    m_requestCalibration = false;
    float offset = calibrateSensor();
    uint8_t temp = 0;
    m_calibrateCharacteristic.setValue(&temp, 1); //Set value back to '0' when calibration is complete
    m_calibrateCharacteristic.notify();
    m_offsetWrapper.writeValue(offset);
  }

  unsigned long now = micros();
  if (m_ready && (now - m_lastTime >= SAMPLE_TIME)) {
    m_lastTime = now;
    m_sampleCount++;
    float magField = readSensor();
    
    if (magField < m_minValue) {
      m_minValue = magField;
    }

    if (magField > m_maxValue) {
      m_maxValue = magField;
    }

    /*
     * Use double precision for accumulators to prevent "catastrophic cancellation" bug when calculating ACRMS = sqrt(RMS^2 - Average^2)
     */
    double dMagField = (double)magField;
    m_avgAccum += dMagField;
    m_rmsAccum += dMagField * dMagField;

    if (m_sampleCount >= NUM_SAMPLES) {
      double dAvg = m_avgAccum / (double)m_sampleCount;
      double rawRmsSquared = m_rmsAccum / (double)m_sampleCount;
      double acRmsSquared = rawRmsSquared - dAvg * dAvg; //Remove DC offset when calculating RMS - more useful for cable detection
      if (acRmsSquared < 0.0f) {
        /*
         * Final layer of protection in case "catastrophic cancellation" bug still shows up despite best efforts (using double precision).
         *
         * In theory, RMS >= Average, therefore ACRMS = sqrt(RMS^2 - Average^2) should always yield a valid value. Even when no AC field is present,
         * (RMS^2 - Average^2) = 0 but never becomes negative.
         *
         * In practice, in presence of large DC magnetic field, RMS and Average may both be very large and so subject to floating point rounding errors.
         * (RMS^2 - Average^2) may become negative, and sqrt() returns NaN. This gets even worse because BLE stack internally uses fixed-point numbers
         * which have no representation for NaN. End result is that NaN becomes INT32_MIN i.e. very large negative number.
         *
         * This hack will cause a "wrong" result in this rare edge case, but user seeing 0 is better than INT32_MIN (especially as correct result is ~0 anyway).
         */
        acRmsSquared = 0.0f;
        ERROR("Catastrophic cancellation error detected in calculation of AC RMS");
      }

      float avg = (float)dAvg;
      float acRms = (float)sqrt(acRmsSquared);
      float pp = m_maxValue - m_minValue; //Peak-to-peak is the difference between largest and smallest values

      float pk; //Peak is the difference between the largest peak (+ve or -ve) and the average
      if (m_maxValue > -m_minValue) {
        pk = m_maxValue - avg;
      } else {
        pk = avg - m_minValue;
      }

      m_saturatedWrapper.writeValue(isSensorSaturated());
      m_avgWrapper.writeValue(avg);
      m_rmsWrapper.writeValue(acRms);
      m_pkWrapper.writeValue(pk);
      m_ppWrapper.writeValue(pp);
      m_minWrapper.writeValue(m_minValue);
      m_maxWrapper.writeValue(m_maxValue);

      resetStatistics();
    }
  }
}
