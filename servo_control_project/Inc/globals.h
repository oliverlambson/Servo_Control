/*
 * globals.h
 *
 *  Created on: 05 Aug 2018
 *      Author: 18225713
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#define PULSE_MAX 2100 // us
#define PULSE_MIN 900 // us

#define ANGLE_MAX 120 // deg
#define ANGLE_MIN 0 // deg

#define COUNT_MAX 150
#define COUNT_MIN 350

#define COUNT_MAX_TOTAL 3333

// generic
#include "stdint.h"

extern volatile uint8_t duty1;

extern volatile uint16_t count2;
extern volatile uint16_t angle2; // 2*desired angle
extern volatile uint16_t pulse_count_duty2;

#endif /* GLOBALS_H_ */
