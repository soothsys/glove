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
 *               = 1.042267 * 10^-4
 */
#define ADAF1080_SCALE_FACTOR 0.104227F
#define AD4002_MIDCODE 131072

#define SPI_CLOCK_RATE 1000000 //1MHz
#define SPI_BIT_ORDER MSBFIRST
#define SPI_MODE SPI_MODE0

#define PIN_CNV A5
#define PIN_DUMMY_MOSI 33

#define SAMPLE_TIME 1000

#define BLE_INST_ID 0
#define NUM_CHARACTERISTICS 1

#define BLE_SERVICE_UUID BLEUUID("7749eb1b-2b16-4d32-8422-e792dae7adb8")
#define MAGFIELD_UUID BLEUUID("1de21519-a563-428b-9e18-b033f8bd7348")

#define MAGFIELD_FORMAT BLE2904::FORMAT_SINT16
#define MAGFIELD_EXPONENT -2
#define MAGFIELD_UNIT BLEUnit::uTesla
#define MAGFIELD_NAME "Magnetic field strength"

static BLECharacteristic m_magFieldCharacteristic(MAGFIELD_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
static BLEWrapper m_magFieldWrapper(&m_magFieldCharacteristic, MAGFIELD_NAME, MAGFIELD_FORMAT, MAGFIELD_EXPONENT, MAGFIELD_UNIT);

static unsigned long m_lastTime;
static bool m_ready = false;

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

  int i;
  Serial.print("Read 3 bytes:");
  for (i = 0; i < 3; i++) {
    Serial.print(" 0x");
    Serial.print(buffer[i], HEX);
  }
  Serial.println();

  uint32_t result = buffer[0] << 10;
  result |= buffer[1] << 2;
  result |= buffer[2] >> 6; //Only use 2 MSBs of last byte, discarding 6 LSBs
  return result;
}

static float adaf1080_read(void) {
  uint32_t adcCounts = ad4002_readResult(); //18 bit unipolar ADC result

  float vref = 4.5f;
  int numBits = 18;
  float adcMax = pow(2.0f, (float)numBits);
  float volts = adcCounts * vref / adcMax;
  Serial.print("Raw ADC val = ");
  Serial.println(adcCounts);
  Serial.print("Voltage = ");
  Serial.println(volts);

  int32_t bipolar = (int32_t)adcCounts - AD4002_MIDCODE; //18 bit bipolar ADC result i.e symmetrical about 0
  return (float)bipolar * ADAF1080_SCALE_FACTOR;
}

bool adaf1080_init(void) {
  digitalWrite(PIN_CNV, LOW);
  pinMode(PIN_CNV, OUTPUT);

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

  pService->addCharacteristic(&m_magFieldCharacteristic);
  pService->start();
  return true;
}

void adaf1080_loop(void) {
  unsigned long now = millis();
  if (m_ready && (now - m_lastTime >= SAMPLE_TIME)) {
    m_lastTime = now;
    float magField = adaf1080_read();
    m_magFieldWrapper.writeValue(magField);
  }
}
