#pragma once
#include "ch32fun.h"

extern volatile uint32_t systick_cnt;

// Number of ticks elapsed per millisecond (48,000 when using 48MHz Clock)
#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)
// Number of ticks elapsed per microsecond (48 when using 48MHz Clock)
#define SYSTICK_ONE_MICROSECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000000)

#define SYSTICK_PERIOD SYSTICK_ONE_MILLISECOND;

// Simple macro functions to give a arduino-like functions to call
// Reads the raw SysTick Count, and divides it by the number of 
// ticks per microsecond/millisecond
#define millis() (SysTick->CNT / SYSTICK_ONE_MILLISECOND)
#define micros() (SysTick->CNT / SYSTICK_ONE_MICROSECOND)

void systick_init(void);

uint32_t systick_remaining_us( void );

/*
 * SysTick ISR - must be lightweight to prevent the CPU from bogging down.
 * Increments Compare Register and systick_millis when triggered (every 1ms)
 * NOTE: the `__attribute__((interrupt))` attribute is very important
 */
void SysTick_Handler(void) __attribute__((interrupt));
