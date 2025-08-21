#include "ch32fun.h"
#include <stdio.h>
#include "ch32v20xhw.h"
#include "led_status.h"
#include "adc.h"
#include "systick.h"


// Debug printf
uint32_t count;

int last = 0;
void handle_debug_input( int numbytes, uint8_t * data )
{
	last = data[0];
	count += numbytes;
}


// LED indicator state
LedStateT status_led_state;

int main()
{
	SystemInit();

	// APB2 clock /2 (72MHz)
	RCC->CFGR0 |= RCC_PPRE2_DIV2;

	funGpioInitAll(); // Enable GPIOs

	// MCO for clock debugging
	// RCC->CFGR0 |= RCC_CFGR0_MCO_SYSCLK;


	funPinMode(AI_SEL_A, GPIO_CFGLR_OUT_10Mhz_PP);
	funPinMode(AI_SEL_B, GPIO_CFGLR_OUT_10Mhz_PP);

	// // initializing peripherals
	status_led_state = led_status_init();

	printf("Initializing ADC...\n");
	adc_init();
	printf("ADC Initialized\n");


	// enable systick
	systick_init();

	while(1)
	{
		for(uint8_t ai_sel_no = 0; ai_sel_no < 4; ai_sel_no++) {
			adc_ai_select(ai_sel_no);
			adc_poll_dma();
			adc_wait_for_data();
			adc_process_data();
			// printf("%d ", adc_state.buffer[4]);
		}
		led_status_process(&status_led_state);

		printf("Motor 3 position: %d current: %d\n", adc_state.pot[3], adc_state.isns_motor[3]);
		
		// Delay_Ms(1);
		// printf("Attempting adc conversion...\n")
		// printf(" ADC is %d\n", val);
		// printf("DMA Values!\n");

		// for(uint8_t i = 0; i < ADC_CHANNELS; ++i) {
		// 	printf("%d,", adc_state.buffer[i]);
		// }
		// printf("\n");

	}
}