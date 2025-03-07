#define ERR_MODULE_NAME "PowerMgmt"

#include <stdbool.h>
#include <Arduino.h>

#include "driver/rtc_io.h"
#include "err.h"

#define BTTN_PIN 38
#define BTTN_PIN_MASK (1ULL << BTTN_PIN)
#define DEBOUNCE_TIME 100 //ms

static bool m_lastState = HIGH;
static bool m_released = false;
static unsigned long m_debounceStartTime;

void powermgmt_init(void) {
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  pinMode(BTTN_PIN, INPUT);

  /*
   * In case user is still holding standby button after reboot, make sure we don't immediately go back to sleep
   */
  if (digitalRead(BTTN_PIN) == LOW) {
    Serial.println("Standby button is still pressed, waiting until released");
    while (digitalRead(BTTN_PIN) == LOW) {
      delay(DEBOUNCE_TIME);
    }

    Serial.println("Standby button released!");
  }
}

static void goToSleep(void) {
  Serial.println("Powering down! Zzzzzzz");
  digitalWrite(NEOPIXEL_I2C_POWER, LOW); //Power down sensors
  esp_sleep_enable_ext1_wakeup_io(BTTN_PIN_MASK, ESP_EXT1_WAKEUP_ALL_LOW); //System will reboot when button next pressed
  esp_deep_sleep_start();
  ERROR_HALT("Failed to enter deep sleep mode!"); //Resume from deep sleep causes a CPU reset. If all goes well we should never get here
}

void powermgmt_loop(void) {
  bool state = digitalRead(BTTN_PIN);
  if ((m_lastState == LOW) && (state == HIGH)) { //Rising edge, button released
    m_released = true;
    m_debounceStartTime = millis();
  }

  m_lastState = state;

  if (m_released && (millis() - m_debounceStartTime >= DEBOUNCE_TIME)) {
    if (state == LOW) {
      m_debounceStartTime = millis(); //Button is still pressed, restart debounce time
    } else {
      goToSleep(); //Button was released, time to sleep
    }
  }
}
