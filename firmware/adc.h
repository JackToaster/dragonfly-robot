#pragma once
#include "ch32fun.h"
#include "ch32v20xhw.h"

#include <stdio.h>

#define ADC_CHANNELS 9

#define AI_SEL_A PB8
#define AI_SEL_B PB9

#define ADC_BUFFER_ISNS0 0
#define ADC_BUFFER_ISNS1 1
#define ADC_BUFFER_ISNS2 2
#define ADC_BUFFER_ISNS3 3

#define ADC_BUFFER_POT 4
#define ADC_BUFFER_AI_TEMP 5

#define ADC_BUFFER_VSNS_BAT 6
#define ADC_BUFFER_ISNS_BAT 7

#define ADC_BUFFER_VREF 8


typedef struct ADCStateT {
	uint16_t buffer[ADC_CHANNELS];
	uint8_t selected_ai;

	uint16_t pot[4];
	uint16_t temp_ai[4];
	int16_t isns_motor[4];
	int16_t vsns_bat;
	int16_t isns_bat;
	uint16_t vref;

} ADCStateT;
extern ADCStateT adc_state;

void init_adc_in_pin(GPIO_TypeDef *gpio, uint32_t pin);

void adc_dma_init( void );


/*
 * initialize adc for polling
 */
void adc_init( void );


/*
 * start conversion, wait for result
 */
void adc_poll_dma( void );
void adc_wait_for_data( void );

// process data (subtract vref, split up pot channels, maybe filter eventually?)
void adc_process_data( void );

// select input on analog mux
void adc_ai_select(uint8_t input);



void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt));
