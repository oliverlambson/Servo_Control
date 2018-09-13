/*
 * usr.h
 *
 *  Created on: 05 Aug 2018
 *      Author: 18225713
 */

#ifndef USR_H_
#define USR_H_

/* INCLUDES */
// stm
#include "stm32f3xx_hal.h"

// generic
#include "stdint.h"

// user
#include "globals.h"

/* DEFINES & CONSTANTS */


/* PUBLIC FUNCTION PROTOTYPES */
void usr_init(TIM_HandleTypeDef *htim);
void usr_process(TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc);


#endif /* USR_H_ */
