#include "motor.h"
#include "adc.h"
#include "ch32fun.h"
#include "ch32v20xhw.h"
#include "systick.h"
#include <stdlib.h>

MotorCtrlT motors[N_MOTORS];

void motor_init( void ) {
	// init motors struct
	for(uint8_t i = 0; i < N_MOTORS; ++i){
		motors[i].duty = 0;
		motors[i].current_setpoint = 0;
		motors[i].current_integrator = 0;

		motors[i].position_setpoint = 2048; // midpoint
		motors[i].duty_cap = MOTOR_DUTY_MAX;
		motors[i].mode = MOTOR_DISABLED;
	}
	// motor pins/buffers
	motors[0].duty_buffer = &(TIM1->CH1CVR);
	motors[0].dir_pin = MOTOR_D0_DIR;

	motors[1].duty_buffer = &(TIM1->CH2CVR);
	motors[1].dir_pin = MOTOR_D2_DIR;

	motors[2].duty_buffer = &(TIM1->CH3CVR);
	motors[2].dir_pin = MOTOR_D2_DIR;

	motors[3].duty_buffer = &(TIM1->CH4CVR);
	motors[3].dir_pin = MOTOR_D3_DIR;

	// setup TIM1
	// enable TIM1
	RCC->APB2PCENR |= RCC_APB2Periph_TIM1;

	// Reset TIM1 to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

	// CTLR1: default is up, events generated, edge align
	// SMCFGR: default clk input is CK_INT
	
	// Prescaler 
	// Using APB2 Bus x2 (72MHz x2 = 144MHz)
	TIM1->PSC = 0x0000;
	
	// Auto Reload - sets period
	TIM1->ATRLR = MOTOR_DUTY_MAX;
	
	// Reload immediately
	TIM1->SWEVGR |= TIM_UG;

	
	// Enable CH1-4 output, default polarity (High for 0-CCR, low after)
	TIM1->CCER |= TIM_CC1E | TIM_CC2E | TIM_CC3E | TIM_CC4E;
	
	// CH1/2 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC2M_2 | TIM_OC2M_1;
	
	// CH3/4 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM1->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1 | TIM_OC4M_2 | TIM_OC4M_1;
	
	// Set the Capture Compare Register value to 0 initially
	TIM1->CH1CVR = 0;
	TIM1->CH2CVR = 0;
	TIM1->CH3CVR = 0;
	TIM1->CH4CVR = 0;
	
	// Enable TIM1 outputs
	TIM1->BDTR |= TIM_MOE;
	
	// Enable TIM1
	TIM1->CTLR1 |= TIM_CEN;

	// Setup mode pins
	funPinMode( MOTOR_D0_MODE, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );
	funPinMode( MOTOR_D1_MODE, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );
	funPinMode( MOTOR_D2_MODE, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );
	funPinMode( MOTOR_D3_MODE, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );

	// MODE 1 PH/EN
	funDigitalWrite(MOTOR_D0_MODE, 1);
	funDigitalWrite(MOTOR_D1_MODE, 1);
	funDigitalWrite(MOTOR_D2_MODE, 1);
	funDigitalWrite(MOTOR_D3_MODE, 1);


	// set up enable pins (these get PWM'd)
	funPinMode( MOTOR_D0_EN, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF );
	funPinMode( MOTOR_D1_EN, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF );
	funPinMode( MOTOR_D2_EN, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF );
	funPinMode( MOTOR_D3_EN, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF );


	// Dir pins
	funPinMode( MOTOR_D0_DIR, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );
	funPinMode( MOTOR_D1_DIR, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );
	funPinMode( MOTOR_D2_DIR, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );
	funPinMode( MOTOR_D3_DIR, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );

	funDigitalWrite(MOTOR_D3_DIR, 0);
	funDigitalWrite(MOTOR_D3_DIR, 0);
	funDigitalWrite(MOTOR_D3_DIR, 0);
	funDigitalWrite(MOTOR_D3_DIR, 0);


}

uint8_t ditherer = 0;
void set_from_duty(MotorCtrlT * motor) {
	uint32_t pos_duty = abs(motor->duty);

	pos_duty += MOTOR_DUTY_MIN;  // duties less than this will make the DRV8212 fall asleep :'(
	// this means you can't go below 4% duty at 10kHz, 8% at 20kHz

	if(pos_duty > motor->duty_cap) {
		pos_duty = motor->duty_cap;
	}
	
	funDigitalWrite(motor->dir_pin, motor->duty > 0);
	*motor->duty_buffer = pos_duty;
}
void motor_update( void ) {
	ditherer = !ditherer; // toggle each time
	for(uint8_t i = 0; i < N_MOTORS; ++i) {
		if(motors[i].mode == MOTOR_DISABLED) {
			*motors[i].duty_buffer = 0; // will need to wake up again...
		}else if(motors[i].mode == MOTOR_DUTY_CYCLE) {
			set_from_duty(&motors[i]);
		}else if(motors[i].mode == MOTOR_CURRENT_CONTROL) { // TODO Current sensing borken :'(((
			int32_t current_feedback = adc_state.isns_motor[i]; // read & convert to int32

			#ifdef CURRENT_FEEDBACK_INVERTED
			current_feedback = -current_feedback; // invert if necessary
			#endif
			
			int32_t current_error = motors[i].current_setpoint - current_feedback;

			int32_t current_p = current_error * CURRENT_kP; // proportional term
			
			//integral term
			motors[i].current_integrator += current_error;

			int32_t current_i = motors[i].current_integrator * CURRENT_kI;
			if(current_i > CURRENT_GAIN_SCALE) { // anti-windup past limits of control
				motors[i].current_integrator = CURRENT_GAIN_SCALE / CURRENT_kI;
			} else if(current_i < -CURRENT_GAIN_SCALE) {
				motors[i].current_integrator = -(CURRENT_GAIN_SCALE / CURRENT_kI);
			}

			int32_t demand = current_p + current_i;
			motors[i].duty = (demand * MOTOR_DUTY_MAX) / CURRENT_GAIN_SCALE;
			set_from_duty(&motors[i]);
		}else if(motors[i].mode == MOTOR_POSITION_CONTROL) {
			int32_t position = adc_state.pot[i]; // read & convert to int32
			
			int32_t pos_error = motors[i].position_setpoint - position;

			int32_t pos_p = pos_error * POSITION_kP; // proportional term
			
			int32_t delta = pos_error - motors[i].prev_pos_err;

			int32_t pos_d = delta * POSITION_kD;
			//integral term
			motors[i].position_integrator += pos_error;

			int32_t position_i = motors[i].position_integrator * POSITION_kI;
			if(position_i > POSITION_GAIN_SCALE) { // anti-windup past limits of control
				motors[i].position_integrator = POSITION_GAIN_SCALE / POSITION_kI;
			} else if(position_i < -POSITION_GAIN_SCALE) {
				motors[i].position_integrator = -(POSITION_GAIN_SCALE / POSITION_kI);
			}

			int32_t demand = pos_p + position_i - pos_d;
			motors[i].duty = (demand * MOTOR_DUTY_MAX) / POSITION_GAIN_SCALE;
			set_from_duty(&motors[i]);
			motors[i].prev_pos_err = pos_error;
		}

	}
}

void wakeup_motors( void ) {
	for(uint8_t i = 0; i < N_MOTORS; ++i) {
		*motors[i].duty_buffer = MOTOR_DUTY_MAX;
	}
	Delay_Ms(10);
	for(uint8_t i = 0; i < N_MOTORS; ++i) {
		*motors[i].duty_buffer = MOTOR_DUTY_MIN;
	}
}

void set_current_mA(MotorCtrlT* motor, int32_t current_mA) {
	motor->mode = MOTOR_CURRENT_CONTROL;
	motor->current_setpoint = current_mA * 1000 / MOTOR_CURRENT_GAIN;
}

void set_position(MotorCtrlT* motor, int32_t setpoint) {
	motor->mode = MOTOR_POSITION_CONTROL;
	motor->position_setpoint = setpoint;
}

void set_duty_pct(MotorCtrlT* motor, int32_t duty_pct) {
	// duty_pct is -100 to +100
	motor->mode = MOTOR_DUTY_CYCLE;
	motor->duty = (duty_pct * MOTOR_DUTY_MAX) / 100; // convert units
}
