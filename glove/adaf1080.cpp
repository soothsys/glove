/*
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
 *                 = 183 mV/mT (for +/-8mT range, gain = 80)
 *                 = 0.000183 V/uT
 *               n = ADC bit depth
 *                 = 18 bits
 *         Midcode = 2^(n - 1)
 *                 = 2^(18 - 1)
 *                 = 131072
 *
 * Therefore   K = 5 / (Sdevice * 2^n)
 *               = 5 / (0.000183 * 2^18)
 *               = 0.104227
 */
#define ADAF1080_SCALE_FACTOR 0.104227F
#define AD4002_MIDCODE 131072

#define SPI_CLOCK_RATE 1000000 //1MHz
#define SPI_BIT_ORDER MSBFIRST
#define SPI_MODE SPI_MODE0

#define PIN_PWR_EN 12
#define PIN_DIAG_EN 27
#define PIN_FLIP_DRV 33
#define PIN_CNV A5

#define STARTUP_DELAY 50 //ms
#define FLIP_DELAY 1 //ms
#define SAMPLE_TIME 4000 //4000us = 250Hz
#define NUM_SAMPLES 250 //Calculate statistics every second
#define CAL_AVERAGE_SAMPLES 32 //Average over multiple samples during calibration process to reduce noise

#define BLE_INST_ID 0
#define NUM_CHARACTERISTICS 10

#define BLE_SERVICE_UUID BLEUUID("7749eb1b-2b16-4d32-8422-e792dae7adb8")
#define CALIBRATE_UUID BLEUUID("0b541f35-34c1-4769-b206-8deaaa7e0922")
#define DIAG_SET_UUID BLEUUID("15e53a14-9e4c-489b-ab8c-98569758764d")
#define DIAG_GET_UUID BLEUUID("1de21519-a563-428b-9e18-b033f8bd7348")
#define OFFSET_UUID BLEUUID("3c510d3d-3d82-4fd9-9dd3-da928916662b")
#define AVG_UUID BLEUUID("3c70df7e-3b42-4e52-bbdb-ff47728bec8a")
#define RMS_UUID BLEUUID("5fd8a802-0645-492f-bb0e-541972833add")
#define PK_UUID BLEUUID("949b3518-826e-4a4b-b638-fea08b01e1a0")
#define PP_UUID BLEUUID("9c60f79f-18e0-4343-967e-b6474b305c8b")
#define MIN_UUID BLEUUID("eea8f3a7-d5b1-4454-8e5b-44ce3c0fb372")
#define MAX_UUID BLEUUID("fc13446a-8329-4a00-8b74-6119d1129485")

#define CALIBRATE_FORMAT BLE2904::FORMAT_BOOLEAN
#define DIAG_SET_FORMAT BLE2904::FORMAT_BOOLEAN
#define DIAG_GET_FORMAT BLE2904::FORMAT_BOOLEAN
#define MAGFIELD_FORMAT BLE2904::FORMAT_SINT32

#define CALIBRATE_EXPONENT 0
#define DIAG_SET_EXPONENT 0
#define DIAG_GET_EXPONENT 0
#define MAGFIELD_EXPONENT -2 //10nT precision

#define CALIBRATE_UNIT BLEUnit::Unitless
#define DIAG_SET_UNIT BLEUnit::Unitless
#define DIAG_GET_UNIT BLEUnit::Unitless
#define MAGFIELD_UNIT BLEUnit::uTesla

#define CALIBRATE_NAME "Calibrate sensor"
#define DIAG_SET_NAME "Diag coil on/off"
#define DIAG_GET_NAME "Diag coil status"
#define OFFSET_NAME "Sensor offset correction"
#define AVG_NAME "Average (DC)"
#define RMS_NAME "Root mean square (AC RMS)"
#define PK_NAME "Peak"
#define PP_NAME "Peak-to-peak"
#define MIN_NAME "Minimum"
#define MAX_NAME "Maximum"

static BLECharacteristic m_calibrateCharacteristic(CALIBRATE_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_diagSetCharacteristic(DIAG_SET_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_diagGetCharacteristic(DIAG_GET_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_offsetCharacteristic(OFFSET_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_avgCharacteristic(AVG_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_rmsCharacteristic(RMS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_pkCharacteristic(PK_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_ppCharacteristic(PP_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_minCharacteristic(MIN_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLECharacteristic m_maxCharacteristic(MAX_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

static BLEWrapper m_calibrateWrapper(&m_calibrateCharacteristic, CALIBRATE_NAME, CALIBRATE_FORMAT, CALIBRATE_EXPONENT, CALIBRATE_UNIT);
static BLEWrapper m_diagSetWrapper(&m_diagSetCharacteristic, DIAG_SET_NAME, DIAG_SET_FORMAT, DIAG_SET_EXPONENT, DIAG_SET_UNIT);
static BLEWrapper m_diagGetWrapper(&m_diagGetCharacteristic, DIAG_GET_NAME, DIAG_GET_FORMAT, DIAG_GET_EXPONENT, DIAG_GET_UNIT);
static BLEWrapper m_offsetWrapper(&m_offsetCharacteristic, OFFSET_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_avgWrapper(&m_avgCharacteristic, AVG_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_rmsWrapper(&m_rmsCharacteristic, RMS_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_pkWrapper(&m_pkCharacteristic, PK_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_ppWrapper(&m_ppCharacteristic, PP_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_minWrapper(&m_minCharacteristic, MIN_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);
static BLEWrapper m_maxWrapper(&m_maxCharacteristic, MAX_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);

static unsigned long m_lastTime;
static bool m_ready = false;
static bool m_diagCoilOn = false;
static int32_t m_offsetCorrection = 0; //Offset correction factor measured in ADC counts
static float m_reportedOffset = 0.0f; //Actual sensor offset in uTesla (only used for reporting)

static int m_sampleCount;
static float m_minValue;
static float m_maxValue;
static float m_avgAccum;
static float m_rmsAccum;

static void calibrateSensor(void);
static void diagCoilOn(void);
static void diagCoilOff(void);

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
      calibrateSensor();
      uint8_t newVal = 0;
      pCharacteristic->setValue(&newVal, 1); //Set value back to '0' when calibration is complete
      pCharacteristic->notify();
    }
  }
};

class DiagCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic == NULL) {
      return;
    }

    size_t dataLen = pCharacteristic->getLength();
    if (dataLen < 1) {
      return;
    }

    uint8_t *pData = pCharacteristic->getData();
    if (pData[0]) {
      diagCoilOn();
    } else {
      diagCoilOff();
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
    delay(1); //Prevent timing violation
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

static void calibrateSensor(void) {
  /*
   * See ADAF1080 datasheet page 27 for details of offset correction
   */
  Serial.print("ADAF1080: beginning sensor calibration... ");

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
  m_reportedOffset = fOffsetCorrection * ADAF1080_SCALE_FACTOR; //We now know the sensor offset in raw ADC counts. Convert that back to uTesla for reporting

  resetStatistics(); //Previously gathered statistics are now invalid due to change of offset, start from scratch
  Serial.println("Complete!");
  Serial.print("ADAF1080: Offset correction factor is now ");
  Serial.print(m_offsetCorrection);
  Serial.println(" LSBs");
}

static void diagCoilOn(void) {
  Serial.println("Enabling diag coil");
  digitalWrite(PIN_DIAG_EN, HIGH);
  m_diagCoilOn = true;
}

static void diagCoilOff(void) {
  Serial.println("Disabling diag coil");
  digitalWrite(PIN_DIAG_EN, LOW);
  m_diagCoilOn = false;
}

bool adaf1080_init(void) {
  pinMode(PIN_FLIP_DRV, OUTPUT);
  digitalWrite(PIN_FLIP_DRV, LOW); //Start with FLIP_DRV low ready to generate positive edge
  pinMode(PIN_DIAG_EN, OUTPUT);
  digitalWrite(PIN_DIAG_EN, LOW); //Turn diag coil off
  pinMode(PIN_CNV, OUTPUT);
  digitalWrite(PIN_CNV, LOW); //CNV should idle low between transactions
  pinMode(PIN_PWR_EN, OUTPUT);
  digitalWrite(PIN_PWR_EN, HIGH); //Power on 5V boost converter

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
  pService->addCharacteristic(&m_diagSetCharacteristic);
  pService->addCharacteristic(&m_diagGetCharacteristic);
  pService->addCharacteristic(&m_offsetCharacteristic);
  pService->addCharacteristic(&m_avgCharacteristic);
  pService->addCharacteristic(&m_rmsCharacteristic);
  pService->addCharacteristic(&m_pkCharacteristic);
  pService->addCharacteristic(&m_ppCharacteristic);
  pService->addCharacteristic(&m_minCharacteristic);
  pService->addCharacteristic(&m_maxCharacteristic);
  m_calibrateCharacteristic.setCallbacks(new CalibrateCallbacks());
  m_diagSetCharacteristic.setCallbacks(new DiagCallbacks());
  pService->start();

  uint8_t temp = 0;
  m_calibrateCharacteristic.setValue(&temp, 1);
  m_calibrateCharacteristic.notify();
  m_diagSetCharacteristic.setValue(&temp, 1);
  m_diagSetCharacteristic.notify();
  return true;
}

void adaf1080_loop(void) {
  unsigned long now = micros();
  if (m_ready && (now - m_lastTime >= SAMPLE_TIME)) {
    m_lastTime = now;

    float magField = readSensor();
    m_avgAccum += magField;
    m_rmsAccum += magField * magField; //RMS is the square-root of the average of the squares

    if (magField < m_minValue) {
      m_minValue = magField;
    }

    if (magField > m_maxValue) {
      m_maxValue = magField;
    }

    m_sampleCount++;
    if (m_sampleCount >= NUM_SAMPLES) {
      float avg = m_avgAccum / (float)m_sampleCount;
      float rms = sqrt(m_rmsAccum / (float)m_sampleCount - avg * avg); //Remove DC offset when calculating RMS - more useful for cable detection
      float pp = m_maxValue - m_minValue; //Peak-to-peak is the difference between largest and smallest values

      float pk; //Peak is the difference between the largest peak (+ve or -ve) and the average
      if (m_maxValue > -m_minValue) {
        pk = m_maxValue - avg;
      } else {
        pk = -(m_minValue - avg);
      }

      m_diagGetWrapper.writeValue(m_diagCoilOn);
      m_offsetWrapper.writeValue(m_reportedOffset);
      m_avgWrapper.writeValue(avg);
      m_rmsWrapper.writeValue(rms);
      m_pkWrapper.writeValue(pk);
      m_ppWrapper.writeValue(pp);
      m_minWrapper.writeValue(m_minValue);
      m_maxWrapper.writeValue(m_maxValue);

      resetStatistics();
    }
  }
}
