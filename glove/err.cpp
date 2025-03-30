/*
 * Smart Glove Demo v1.0
 *
 * Copyright (C) 2025 Soothsayer Systems Ltd. All rights reserved.
 *
 * Author: Tom Coates <tom@soothsys.com>
 */

#include <stdbool.h>
#include <Arduino.h>

#include "powermgmt.h"

#define ERR_LED_PIN			LED_BUILTIN		//Use default LED pin for now
#define FLASH_TIME      200           //ms

void err_init(void) {
  digitalWrite(ERR_LED_PIN, LOW);
	pinMode(ERR_LED_PIN, OUTPUT);
}

static void innerPrint(bool halt, const char *module, const char *message) {
	Serial.print(module);
	Serial.print(": ");
	Serial.println(message);
	
	if (halt) {
    unsigned long lastTime = 0;
    bool state = HIGH;
		Serial.println("Halting!");

		while (1) {
      unsigned long now = millis();
      if (now - lastTime >= FLASH_TIME) {
  			digitalWrite(ERR_LED_PIN, state); //Flash LED
        state = !state;
        lastTime = now;
      }

      powermgmt_loop(); //Continue to monitor power button
		}
	}
}

void err_print(bool halt, const char *module, const char *message, ...) {
	va_list args;
	va_start(args, message);
	
	int len = vsnprintf(NULL, 0, message, args) + 1; //Leave space for null terminator
	char *buffer = (char *)malloc(len);
	if (buffer == NULL) {
		innerPrint(true, "ERR", "An error occured while allocating memory to print a previous error message");
	} else {
		vsnprintf(buffer, len, message, args);
		innerPrint(halt, module, buffer);
		free(buffer);
	}
}
