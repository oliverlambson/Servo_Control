/*
 * usr.c
 *
 *  Created on: 05 Aug 2018
 *      Author: 18225713
 */
#include "usr.h"

void usr_process(TIM_HandleTypeDef *htim2)
{

}

/* converts desired angle to number of pulses required
 * angle = 2*angle desired
 * i.e. if angle = 15 angle desired is 7.5 degrees
 */
uint16_t get_pulse_count(uint16_t angle)
{
	return (((5*angle)/3) + 150*2)/2;
}
