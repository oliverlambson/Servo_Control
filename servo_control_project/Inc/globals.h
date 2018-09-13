/*
 * globals.h
 *
 *  Created on: 05 Aug 2018
 *      Author: 18225713
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

/* INCLUDES */
// stm
#include "stm32f3xx_hal.h"

// generic
#include "stdint.h"
#include "stdbool.h"


/* DEFINES & CONSTANTS */
// timers/counters
#define MAX_DEBOUNCE_TIME 100 // ms

// potentiometer
#define POT_ADC_IN_MAX 4095
#define POT_ADC_IN_MIN 0

// servos
#define SERVO_TICK_INTERVAL 6 // us
#define NUMBER_SERVOS 2
#define ANGLE_MAX 120 // deg
#define ANGLE_MIN 0 // deg
#define PULSE_COUNT_MAX 150 // 900-2100 us
#define PULSE_COUNT_MIN 350 // 900-2100 us
#define PULSE_COUNT_MAX_TOTAL 3333
#define SERVO1_MIN 0
#define SERVO1_MAX 120
#define SERVO2_MIN 0
#define SERVO2_MAX 60

/* TYPES */
enum ctrl_state
{
	idle,
	servo1_ctrl,
	servo2_ctrl,
	run
};


/* VARIABLES */
extern enum ctrl_state state;

// flags
extern volatile bool gf_SYSTICK;
extern volatile bool gf_RTC_TICK;
extern volatile bool gf_B1;
extern volatile bool gf_SERVO1_BTN;
extern volatile bool gf_SERVO2_BTN;
extern volatile bool gf_RUN_BTN;
extern GPIO_PinState gf_SERVO1_LED;
extern GPIO_PinState gf_SERVO2_LED;
extern GPIO_PinState gf_RUN_LED;

// timers/counters
extern volatile uint16_t g_msec_count;
volatile uint32_t g_curr_time;
volatile uint32_t g_switch_time;
volatile uint16_t g_prev_btn;

// servos
extern uint16_t servo_angle[NUMBER_SERVOS];
extern uint16_t servo_angle_min[NUMBER_SERVOS];
extern uint16_t servo_angle_max[NUMBER_SERVOS];
extern uint16_t servo_angle_zero[NUMBER_SERVOS];
extern uint32_t servo_pulse_width[NUMBER_SERVOS];


#endif /* GLOBALS_H_ */
