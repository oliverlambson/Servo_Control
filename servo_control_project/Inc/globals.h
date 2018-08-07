/*
 * globals.h
 *
 *  Created on: 05 Aug 2018
 *      Author: 18225713
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

// stm
#include "stm32f3xx_hal.h"

// generic
#include "stdint.h"

#define SERVO_TICK_INTERVAL 6 // us

#define NUMBER_SERVOS 2

//#define PULSE_MAX 2100 // us
//#define PULSE_MIN 900 // us

#define ANGLE_MAX 120 // deg
#define ANGLE_MIN 0 // deg

#define PULSE_COUNT_MAX 150
#define PULSE_COUNT_MIN 350

#define PULSE_COUNT_MAX_TOTAL 3333

extern volatile uint8_t gf_SYSTICK;
extern volatile uint8_t gf_RTC_TICK;
extern volatile uint16_t g_msec_count;

extern volatile uint8_t gf_BTN_PRESS;

extern volatile uint16_t servo_angle[NUMBER_SERVOS];
extern volatile uint32_t servo_pulse_width[NUMBER_SERVOS];

#endif /* GLOBALS_H_ */
