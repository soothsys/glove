#include <stdbool.h>
#include <Arduino.h>

#define ERR_LED_PIN			LED_BUILTIN		//Use default LED pin for now

void err_init(void) {
	pinMode(ERR_LED_PIN, OUTPUT);
}

static void innerPrint(bool halt, const char *module, const char *message) {
	Serial.print(module);
	Serial.print(": ");
	Serial.println(message);
	
	if (halt) {
		Serial.println("Halting!");
		while (1) {
			digitalWrite(LED_BUILTIN, LOW);
			delay(200);
			digitalWrite(LED_BUILTIN, HIGH);
			delay(200);
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
