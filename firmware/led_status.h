#pragma once
#include "ch32fun.h"
#include "systick.h"
#include <stdint.h>

#define LED_PIN_R PC13
#define LED_PIN_G PC14
#define LED_PIN_B PC15

typedef struct {
	uint32_t counter;
} LedStateT;

// hast to be called after funGpioInitAll
LedStateT led_status_init() {
	// setup gpios
	funPinMode( LED_PIN_R, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );
	funPinMode( LED_PIN_G, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );
	funPinMode( LED_PIN_B, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );

	funDigitalWrite(LED_PIN_R, 0);
	funDigitalWrite(LED_PIN_G, 0);
	funDigitalWrite(LED_PIN_B, 0);
	
	LedStateT state = {
		.counter=0
	};

	return state;

}
void led_status_process(LedStateT *state) {
	uint32_t t = millis() % 3000;
	if(t < 1000) {
		funDigitalWrite(LED_PIN_R, 1);
		funDigitalWrite(LED_PIN_G, 0);
		funDigitalWrite(LED_PIN_B, 0);
	}else if (t < 2000) {
		funDigitalWrite(LED_PIN_R, 0);
		funDigitalWrite(LED_PIN_G, 1);
		funDigitalWrite(LED_PIN_B, 0);
	
	}else {
		funDigitalWrite(LED_PIN_R, 0);
		funDigitalWrite(LED_PIN_G, 0);
		funDigitalWrite(LED_PIN_B, 1);
	}

}