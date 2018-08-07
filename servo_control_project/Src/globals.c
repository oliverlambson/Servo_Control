/*
 * globals.c
 *
 *  Created on: 05 Aug 2018
 *      Author: 18225713
 */

#include "globals.h"

volatile uint8_t gf_SYSTICK = 0;
volatile uint8_t gf_RTC_TICK = 0;
volatile uint16_t g_msec_count = 0;

volatile uint8_t gf_BTN_PRESS = 0;

volatile uint16_t servo_angle[NUMBER_SERVOS];
volatile uint32_t servo_pulse_width[NUMBER_SERVOS];
