#pragma once
#include "ch32fun.h"
#include "ch32v20xhw.h"

#define CURRENT_FEEDBACK_INVERTED

#define MOTOR_CURRENT_GAIN 1700 // uA per ADC count, divide by 1000. Theoretically 1.611 but calibrated to 1.7

#define MOTOR_MAX_CURRENT_mA 200
#define MOTOR_DUTY_MAX 14400 // sets 20kHz PWM frequency
#define MOTOR_DUTY_MIN 600 // can't go below this or the motors fall alseep :P

#define CURRENT_GAIN_SCALE 32768 // Gains are 1/8192 of full duty per unit
#define CURRENT_kP 1
#define CURRENT_kI 500

#define MOTOR_KV 2750 // units/sec/V

#define POSITION_GAIN_SCALE 32768
#define POSITION_kP 50
#define POSITION_kI 0
#define POSITION_kD 0

#define MOTOR_D0_MODE PB12
#define MOTOR_D1_MODE PB13
#define MOTOR_D2_MODE PB14
#define MOTOR_D3_MODE PB15

#define MOTOR_D0_DIR PA12
#define MOTOR_D1_DIR PA15
#define MOTOR_D2_DIR PB2
#define MOTOR_D3_DIR PB3

#define MOTOR_D0_EN PA8
#define MOTOR_D1_EN PA9
#define MOTOR_D2_EN PA10
#define MOTOR_D3_EN PA11

#define N_MOTORS 4

typedef enum MotorControlMode {
	MOTOR_DISABLED=0,
	MOTOR_DUTY_CYCLE=1,
	MOTOR_CURRENT_CONTROL=2,
	MOTOR_POSITION_CONTROL=3,
} MotorControlMode;

typedef struct MotorCtrlT {
	MotorControlMode mode;

	uint32_t duty_cap;
	int32_t duty; // -MOTOR_DUTY_MAX to +MOTOR_DUTY_MAX
	volatile uint16_t* duty_buffer; // Buffer to write PWM duty
	uint32_t dir_pin;

	int32_t current_setpoint; // motor current setpoint in ADC counts
	int32_t current_integrator; // integrator for current control loop

	int32_t position_setpoint; // in motor sensor units
	int32_t position_integrator; // integrator for current control loop
	int32_t feedforward_vel; // motor sensor units/sec
	int32_t prev_pos_err;

} MotorCtrlT;

void set_duty_pct(MotorCtrlT* motor, int32_t duty_pct);

void set_current_mA(MotorCtrlT* motor, int32_t current_mA);

void set_position(MotorCtrlT* motor, int32_t setpoint);

extern MotorCtrlT motors[N_MOTORS];

void motor_init( void );

void wakeup_motors( void );

void motor_update( void );