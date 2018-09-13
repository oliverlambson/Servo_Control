/*
 * globals.c
 *
 *  Created on: 05 Aug 2018
 *      Author: 18225713
 */

#include "globals.h"

/* VARIABLES */
enum ctrl_state state = idle;

// flags
volatile bool gf_SYSTICK = false;
volatile bool gf_RTC_TICK = false;
volatile bool gf_B1 = false;
volatile bool gf_SERVO1_BTN = false;
volatile bool gf_SERVO2_BTN = false;
volatile bool gf_RUN_BTN = false;
GPIO_PinState gf_SERVO1_LED = GPIO_PIN_RESET;
GPIO_PinState gf_SERVO2_LED = GPIO_PIN_RESET;
GPIO_PinState gf_RUN_LED = GPIO_PIN_RESET;

// timers/counters
volatile uint16_t g_msec_count;
volatile uint32_t g_curr_time;
volatile uint32_t g_switch_time = 0;
volatile uint16_t g_prev_btn;

// servos
uint16_t servo_angle[NUMBER_SERVOS];
uint16_t servo_angle_min[NUMBER_SERVOS] = {SERVO1_MIN, SERVO2_MIN};
uint16_t servo_angle_max[NUMBER_SERVOS] = {SERVO1_MAX, SERVO2_MAX};
uint16_t servo_angle_zero[NUMBER_SERVOS] = {SERVO1_MIN, SERVO2_MIN};
uint32_t servo_pulse_width[NUMBER_SERVOS];
