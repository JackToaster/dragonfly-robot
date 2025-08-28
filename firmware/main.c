#include "ch32fun.h"
#include <stdint.h>
#include <stdio.h>
#include "ch32v20xhw.h"

#include "main.h"
#include "led_status.h"
#include "adc.h"
#include "serial.h"
#include "systick.h"
#include "motor.h"

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

uint32_t get_vbat( void ){
	return (uint32_t) adc_state.vsns_bat * ADC_VDD_mV / ADC_COUNTS * 57 / 10 ;
}


uint8_t adc_sel_no = 0;
void system_update( void ) {


	led_status_process(&status_led_state);
	// start polling new adc data
	adc_poll_dma();
	adc_wait_for_data();
	// read last data
	adc_process_data();

	motor_update(); // update motor control loops

	++adc_sel_no;
	if(adc_sel_no >= 4) { adc_sel_no = 0; }
	adc_ai_select(adc_sel_no);
}

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

	// initializing peripherals
	status_led_state = led_status_init();

	// Serial
	printf("Initializing serial...\n");
	serial_init();
	dma_uart_setup();
	printf("Serial initialized!");

	// ADC
	printf("Initializing ADC...\n");
	adc_init();
	printf("ADC Initialized\n");

	motor_init();

	// enable systick
	systick_init();
	uint8_t duty = 100;

	uint32_t last_t;
	uint32_t last_pos;

	wakeup_motors();
	set_position(&motors[3], 1500);

	static const char message[] = "abcdefghijklmnopqrstuvwxyz";
	// motors[3].duty_cap = MOTOR_DUTY_MAX / 2; // 50% duty limit
	while(1)
	{
		Delay_Us(systick_remaining_us()); // wait for next tick

		// printf("M3 P:%d\n", adc_state.pot[3]);
		system_update();
	

		if(millis() % 2000 == 0) {
			set_position(&motors[3], 1900);
		}
		if(millis() % 2000 == 1000) {
			set_position(&motors[3], 2100);
		}

		if(millis() % 100 == 0) {
			uint32_t pos = adc_state.pot[3];
			uint32_t time = millis();
			uint32_t dt_millis = time - last_t;
			uint32_t dx = last_pos - pos;

			float speed = (float) dx / ((float) dt_millis) * 1000.0;
			// printf("dt: %d dx: %d speed: %d ", dt_millis, dx, (uint32_t)speed);
			
			last_pos = pos;
			last_t = time;
			// printf("D %d P %d S %d\n",  motors[3].duty, adc_state.pot[3],  motors[3].position_setpoint);
			// duty -= 1; if(duty == 0) duty = 1;
			uint32_t bytes = bytes_available(&uart3_rxbuf);
			uint8_t* data = read_data(&uart3_rxbuf, bytes_available(&uart3_rxbuf));
			
			printf("Tx: %s\nRx: %s\n", message, data);
			printf("Tx (hex): ");
			for(uint32_t i = 0; i < bytes; ++i) {
				printf("%x ", message[i]);
			}
			printf("\nRx (hex): ");

			for(uint32_t i = 0; i < bytes; ++i) {
				printf("%x ", data[i]);
			}
			printf("\n");
			Delay_Ms(100);
			dma_uart_tx(message, sizeof(message) - 1);

			Delay_Ms(100);


		}


		// Delay_Ms(1);
		// printf("Attempting adc conversion...\n")
		// printf(" ADC is %d\n", val);
		// printf("DMA Values!\n");


		// printf("vbat: %d\n", get_vbat());

	}
}