#define ERR_MODULE_NAME "AS7341"

#include <BLEServer.h>
#include <BLEUtils.h>
#include <Adafruit_AS7341.h>

#include "i2c_address.h"
#include "blewrapper.h"
#include "err.h"

#define NUM_GAINS 11
const as7341_gain_t AS7341_GAIN_LIST[NUM_GAINS] = {
  AS7341_GAIN_0_5X,
  AS7341_GAIN_1X,
  AS7341_GAIN_2X,
  AS7341_GAIN_4X,
  AS7341_GAIN_8X,
  AS7341_GAIN_16X,
  AS7341_GAIN_32X,
  AS7341_GAIN_64X,
  AS7341_GAIN_128X,
  AS7341_GAIN_256X,
  AS7341_GAIN_512X
};

const float AS7341_GAIN_VALS[NUM_GAINS] = {
  0.5f,
  1.0f,
  2.0f,
  4.0f,
  8.0f,
  16.0f,
  32.0f,
  64.0f,
  128.0f,
  256.0f,
  512.0f
};

/*
 * Integration time = (ATIME + 1) * (ASTEP + 1) * 2.78us
 *                  = (599 + 1) * (29 + 1) * 2.78us
 *                  = 50ms
 */
#define DEFAULT_ATIME 29
#define DEFAULT_ASTEP 599
#define DEFAULT_GAIN_INDEX 9
#define MAX_ADC_COUNTS ((DEFAULT_ATIME + 1) * (DEFAULT_ASTEP + 1))

#define AUTOGAIN_INCR_THRES (25 * (MAX_ADC_COUNTS / 100))
#define AUTOGAIN_DECR_THRES (75 * (MAX_ADC_COUNTS / 100))
#define AUTOGAIN_MIN_INDEX 0
#define AUTOGAIN_MAX_INDEX (NUM_GAINS - 1)

#define NUM_CHANNELS 12
#define SAMPLE_TIME 1000 //milliseconds

#define BLE_INST_ID 0
#define NUM_SENSOR_CHARACTERISTICS 10
#define NUM_CHARACTERISTICS (NUM_SENSOR_CHARACTERISTICS + 1) //Add 1 for gain characeristic

#define BLE_SERVICE_UUID BLEUUID((uint16_t)0x054D)
#define LIGHT_415NM_UUID "5dc8e630-d5d9-4829-a8f1-9e134ceba7a2"
#define LIGHT_445NM_UUID "950de366-308a-4387-9217-776e6631cebf"
#define LIGHT_480NM_UUID "57c33b79-9e54-48e2-a311-c871cd093370"
#define LIGHT_515NM_UUID "b640e35f-e4b0-4a89-922a-eea4e6af30e6"
#define LIGHT_555NM_UUID "784dc5f5-c76a-4d34-a9ee-47e4a8959fa1"
#define LIGHT_590NM_UUID "a4585db9-cf81-4022-bb46-735d32c66650"
#define LIGHT_630NM_UUID "7b7d42c0-f1bf-4b37-a6ea-b51669863b2c"
#define LIGHT_680NM_UUID "e75ed433-6c87-4c78-bdbd-6b8d0398f237"
#define LIGHT_CLEAR_UUID "0091c8af-1571-4857-ad20-3979ad0988a6"
#define LIGHT_NIR_UUID "95dadecf-e892-4b3b-b231-1652e1b80e45"
#define GAIN_UUID "d5b7ab0d-aab7-4016-8dfa-6b1977fa4870"

#define LIGHT_FORMAT BLE2904::FORMAT_UINT16
#define GAIN_FORMAT BLE2904::FORMAT_UINT16
#define LIGHT_EXPONENT -2
#define GAIN_EXPONENT -1
#define LIGHT_UNIT BLEUnit::Unitless
#define GAIN_UNIT BLEUnit::Unitless

#define LIGHT_415NM_NAME "Violet (415nm)"
#define LIGHT_445NM_NAME "Dark blue (445nm)"
#define LIGHT_480NM_NAME "Light blue (480nm)"
#define LIGHT_515NM_NAME "Green (515nm)"
#define LIGHT_555NM_NAME "Yellow-green (555nm)"
#define LIGHT_590NM_NAME "Yellow (590nm)"
#define LIGHT_630NM_NAME "Orange (630nm)"
#define LIGHT_680NM_NAME "Red (680nm)"
#define LIGHT_CLEAR_NAME "Clear"
#define LIGHT_NIR_NAME "Near infrared"
#define GAIN_NAME "Gain"

static BLECharacteristic m_characteristics[] = {
  BLECharacteristic(LIGHT_415NM_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY),
  BLECharacteristic(LIGHT_445NM_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY),
  BLECharacteristic(LIGHT_480NM_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY),
  BLECharacteristic(LIGHT_515NM_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY),
  BLECharacteristic(LIGHT_555NM_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY),
  BLECharacteristic(LIGHT_590NM_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY),
  BLECharacteristic(LIGHT_630NM_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY),
  BLECharacteristic(LIGHT_680NM_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY),
  BLECharacteristic(LIGHT_CLEAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY),
  BLECharacteristic(LIGHT_NIR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY)
};

static BLEWrapper m_wrappers[] = {
  BLEWrapper(&(m_characteristics[0]), LIGHT_415NM_NAME, LIGHT_FORMAT, LIGHT_EXPONENT, LIGHT_UNIT),
  BLEWrapper(&(m_characteristics[1]), LIGHT_445NM_NAME, LIGHT_FORMAT, LIGHT_EXPONENT, LIGHT_UNIT),
  BLEWrapper(&(m_characteristics[2]), LIGHT_480NM_NAME, LIGHT_FORMAT, LIGHT_EXPONENT, LIGHT_UNIT),
  BLEWrapper(&(m_characteristics[3]), LIGHT_515NM_NAME, LIGHT_FORMAT, LIGHT_EXPONENT, LIGHT_UNIT),
  BLEWrapper(&(m_characteristics[4]), LIGHT_555NM_NAME, LIGHT_FORMAT, LIGHT_EXPONENT, LIGHT_UNIT),
  BLEWrapper(&(m_characteristics[5]), LIGHT_590NM_NAME, LIGHT_FORMAT, LIGHT_EXPONENT, LIGHT_UNIT),
  BLEWrapper(&(m_characteristics[6]), LIGHT_630NM_NAME, LIGHT_FORMAT, LIGHT_EXPONENT, LIGHT_UNIT),
  BLEWrapper(&(m_characteristics[7]), LIGHT_680NM_NAME, LIGHT_FORMAT, LIGHT_EXPONENT, LIGHT_UNIT),
  BLEWrapper(&(m_characteristics[8]), LIGHT_CLEAR_NAME, LIGHT_FORMAT, LIGHT_EXPONENT, LIGHT_UNIT),
  BLEWrapper(&(m_characteristics[9]), LIGHT_NIR_NAME, LIGHT_FORMAT, LIGHT_EXPONENT, LIGHT_UNIT)
};

static BLECharacteristic m_gainCharacteristic(GAIN_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLEWrapper m_gainWrapper(&m_gainCharacteristic, GAIN_NAME, GAIN_FORMAT, GAIN_EXPONENT, GAIN_UNIT);

static Adafruit_AS7341 m_sensor;
static int m_gainIndex = DEFAULT_GAIN_INDEX;
static bool m_gainChanged = false;
static unsigned long m_lastTime;
static bool m_ready = false;

bool as7341_init(i2c_address_t addr) {
  if (!m_sensor.begin(addr)) {
    ERROR("Could not initialise sensor");
    return false;
  }

  m_sensor.setATIME(DEFAULT_ATIME);
  m_sensor.setASTEP(DEFAULT_ASTEP);
  m_sensor.setGain(AS7341_GAIN_LIST[m_gainIndex]);
  m_sensor.startReading();

  m_lastTime = millis();
  m_ready = true;
  return true;
}

bool as7341_addService(BLEServer *pServer) {
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
  
  int i;
  for (i = 0; i < NUM_SENSOR_CHARACTERISTICS; i++) {
    pService->addCharacteristic(&(m_characteristics[i]));
  }

  pService->addCharacteristic(&m_gainCharacteristic);
  pService->start();
  m_gainWrapper.writeValue(AS7341_GAIN_VALS[m_gainIndex]);
  return true;
}

int autogain(int currGainIndex, uint16_t *readings) {
  uint16_t maxReading = 0;
  int i;
  for (i = 0; i < NUM_CHANNELS; i++) {
    if (readings[i] > maxReading) {
      maxReading = readings[i];
    }
  }

  if (maxReading > AUTOGAIN_DECR_THRES) { //Decrease gain
    if (currGainIndex > AUTOGAIN_MIN_INDEX) {
      currGainIndex--;
    }
  } else if (maxReading < AUTOGAIN_DECR_THRES) {
    if (currGainIndex < AUTOGAIN_MAX_INDEX) {
      currGainIndex++;
    }
  }

  return currGainIndex;
}

static void reportReadings(uint16_t *readings) {
  int nChannel, nCharacteristic = 0;
  for (nChannel = 0; nChannel < NUM_CHANNELS; nChannel++) {
    if ((nChannel != 4) && (nChannel != 5)) {  //Skip channels 4 and 5 - these are duplicates
      float corrected = m_sensor.toBasicCounts(readings[nChannel]);
      m_wrappers[nCharacteristic].writeValue(corrected);
      nCharacteristic++;
    }
  }

  if (m_gainChanged) {
    m_gainChanged = false;
    float gainVal = AS7341_GAIN_VALS[m_gainIndex];
    m_gainWrapper.writeValue(gainVal);
  }
}

static void handleReadings(uint16_t *readings) {
  /*
   * Readings are reported at the "sample rate", but we actually sample the sensor continually.
   * This allows the autogain algorithm to react quickly.
   */
  unsigned long now = millis();
  if (now - m_lastTime >= SAMPLE_TIME) {
    m_lastTime = now;
    reportReadings(readings);
  }

  /*
   * Do any gain changes AFTER reporting the readings so that we don't screw up gain correction.
   * Gain change will apply to next reading
   */
  int newGainIndex = autogain(m_gainIndex, readings);
  as7341_gain_t newGain = AS7341_GAIN_LIST[newGainIndex];
  if ((newGainIndex != m_gainIndex) && m_sensor.setGain(newGain)) { //Only report gain change if applied successfully
    m_gainIndex = newGainIndex;
    m_gainChanged = true;
  }
}

void as7341_loop(void) {
  if (m_ready && m_sensor.checkReadingProgress()) {
    uint16_t readings[NUM_CHANNELS];
    if (m_sensor.getAllChannels(readings)) {
      handleReadings(readings);
    } else {
      ERROR("Error reading sensor");
    }

    m_sensor.startReading();
  }
}
