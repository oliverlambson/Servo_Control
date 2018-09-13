/*
 * usr.c
 *
 *  Created on: 05 Aug 2018
 *      Author: 18225713
 */
#include "usr.h"

/* PRIVATE FUNCTION PROTOTYPES */

void set_servo_angle(uint16_t angle, uint8_t servo_number);
uint32_t get_pulse_width(uint16_t angle);

/* PUBLIC FUNCTIONS */
/*
 * main.c initialisation
 */
void usr_init(TIM_HandleTypeDef *htim2)
{
	uint8_t i;

  //tim2 init
  for (i = 0; i < NUMBER_SERVOS; ++i)
  {
	  set_servo_angle(0, i);
	  __HAL_TIM_SET_COMPARE(htim2, 4*i, servo_pulse_width[i]);
  }
  __HAL_TIM_ENABLE(htim2);
  HAL_TIM_PWM_Init(htim2);
  HAL_TIM_PWM_Start_IT(htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(htim2, TIM_CHANNEL_2);

  return;
}


/*
 * main.c infinite loop
 */
void usr_process(TIM_HandleTypeDef *htim2)
{
	uint8_t i;

	if (gf_BTN_PRESS)
	{
#if(0)
		for (i = 0; i < NUMBER_SERVOS; ++i)
		{
			if (servo_angle[i] < ANGLE_MAX)
			{
				set_servo_angle(servo_angle[i]+30, i);
			} else
			{
				set_servo_angle(0, i);
			}
			__HAL_TIM_SET_COMPARE(htim2, 4*i, servo_pulse_width[i]);
		}
#endif

		if (servo_angle[0] < ANGLE_MAX)
		{
			set_servo_angle(servo_angle[0]+30, 0);
		} else
		{
			set_servo_angle(0, 0);
		}

		if (servo_angle[1] < ANGLE_MAX)
		{
			set_servo_angle(servo_angle[1]+15, 1);
		} else
		{
			set_servo_angle(0, 1);
		}
		__HAL_TIM_SET_COMPARE(htim2, 4*0, servo_pulse_width[0]);
		__HAL_TIM_SET_COMPARE(htim2, 4*1, servo_pulse_width[1]);

		gf_BTN_PRESS = 0;
	}

	return;
}



/* PRIVATE FUNCTIONS */

/*
 * sets pulse width to number of pulses required for desired angle
 * store angle set
 */
void set_servo_angle(uint16_t angle, uint8_t servo_number)
{
	servo_angle[servo_number] = angle;
	servo_pulse_width[servo_number] = get_pulse_width(angle);

	return;
}

/*
 * converts desired angle to pulse width required
 */
uint32_t get_pulse_width(uint16_t angle)
{
	return 10*angle + 900;
}
