#include "systick.h"
#include "ch32v20xhw.h"
#include <stdio.h>

// Incremented in the SysTick IRQ - in this example once per millisecond
volatile uint32_t systick_cnt;

/*
 * Initialises the SysTick to trigger an IRQ with auto-reload, using HCLK/1 as
 * its clock source
 */
void systick_init(void)
{
	// Reset any pre-existing configuration
	SysTick->CTLR = 0x0000;
	
	// Set the compare register to trigger once per millisecond
	SysTick->CMP = SYSTICK_PERIOD - 1;

	// Reset the Count Register, and the global millis counter to 0
	SysTick->CNT = 0x00000000;
	systick_cnt = 0x00000000;
	
	// Set the SysTick Configuration
	// NOTE: By not setting SYSTICK_CTLR_STRE, we maintain compatibility with
	// busywait delay funtions used by ch32v003_fun.
	SysTick->CTLR |= SYSTICK_CTLR_STE   |  // Enable Counter
	                 SYSTICK_CTLR_STIE  |  // Enable Interrupts
	                 SYSTICK_CTLR_STCLK ;  // Set Clock Source to HCLK/1
	
	// Enable the SysTick IRQ
	NVIC_EnableIRQ(SysTicK_IRQn);
}

uint32_t systick_remaining_us( void ) {
	uint64_t difference = SysTick->CMP - SysTick->CNT;

	return (uint32_t) (difference / SYSTICK_ONE_MICROSECOND);
}

void SysTick_Handler(void)
{
	// Increment the Compare Register for the next trigger
	// If more than this number of ticks elapse before the trigger is reset,
	// you may miss your next interrupt trigger
	// (Make sure the IQR is lightweight and CMP value is reasonable)
	SysTick->CMP += SYSTICK_PERIOD;

	// Clear the trigger state for the next IRQ
	SysTick->SR = 0x00000000;

	// Increment the tick count
	systick_cnt++;
}