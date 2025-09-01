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
#include "crsf.h"

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
	printf("starting serial...\n");
	serial_init();
	dma_uart_setup();
	printf("Serial up\n");

	// ADC
	printf("starting ADC...\n");
	adc_init();
	printf("ADC up\n");

	motor_init();

	// enable systick
	systick_init();
	uint8_t duty = 100;

	uint32_t last_t;
	uint32_t last_pos;

	for(uint8_t i = 0; i < N_MOTORS; ++i) {
		motors[i].duty_cap = MOTOR_DUTY_MAX / 3; // 33% duty limit to avoid burning out servos while testing
	}

	wakeup_motors();
	// set_position(&motors[3], 1500);

	while(1)
	{
		Delay_Us(systick_remaining_us()); // wait for next tick

		// printf("M3 P:%d\n", adc_state.pot[3]);
		system_update();
	

		// if(millis() % 2000 == 0) {
		// 	set_position(&motors[3], 1900);
		// }
		// if(millis() % 2000 == 1000) {
		// 	set_position(&motors[3], 2100);
		// }
		
		// https://github.com/crsf-wg/crsf/wiki/Message-Format
		uint8_t bytes;
		while((bytes = bytes_available(&uart3_rxbuf)) >= 4) { // minimum size for CSRF message is 4 bytes
			uint8_t sync = peek(&uart3_rxbuf);
			if(sync == 0xC8 || sync == 0xEE) { // check for CRSF Sync byte
				uint8_t* header = peek_n(&uart3_rxbuf, 3);
				uint8_t msg_len = header[1] + 2; // message length is LEN+2
				if(msg_len > 64) {
					printf("INVALID sync %x len %d :(((\n", sync, msg_len);
					
					read_data(&uart3_rxbuf, 1); // discard byte
					continue;
				}
				uint8_t msg_type = header[2];
				if(bytes >= msg_len) {
					// printf("msg! len %d type %x\n", msg_len, msg_type);
					uint8_t* message = read_data(&uart3_rxbuf, msg_len);
					if(msg_type == 0x16) { // RC channels packed
						struct crsf_channels_s* channels = (struct crsf_channels_s*) &message[3];
						printf("Channels %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n", channels->ch0,channels->ch1,channels->ch2,channels->ch3,channels->ch4,channels->ch5,channels->ch6,channels->ch7,channels->ch8,channels->ch9,channels->ch10,channels->ch11,channels->ch12,channels->ch13,channels->ch14,channels->ch15);
						set_duty_pct(&motors[3], (((int32_t)channels->ch0) - 992) / 6);
						// printf("%d\n", adc_state.pot[3]);
					}
				} else {
					break;
				}
			} else {
				// printf(":( %d\n", uart3_rxbuf.overrun);
				read_data(&uart3_rxbuf, 1); // discard byte
			}
		}
	}
}