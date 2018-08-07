/*
 * usr.c
 *
 *  Created on: 05 Aug 2018
 *      Author: 18225713
 */
#include "usr.h"


/*
 * main.c initialisation
 */
void usr_init(TIM_HandleTypeDef *htim6)
{
	uint8_t i;
  HAL_TIM_Base_Start_IT(htim6);

  servo_pin_port[0] = SERVO_1_CTRL_GPIO_Port;
  servo_pin_number[0] = SERVO_1_CTRL_Pin;
  servo_pin_port[1] = SERVO_2_CTRL_GPIO_Port;
  servo_pin_number[1] = SERVO_2_CTRL_Pin;

  for (i = 0; i < NUMBER_SERVOS; ++i)
  {
	  servo_pulse_width[i] = get_pulse_count(0);
  }

#if(0)
  //tim2 init
  HAL_TIM_PWM_Init(&htim2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 32);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
#endif

  return;
}

/*
 * main.c infinite loop
 */
void usr_process()
{
	uint8_t i;

	if (gf_BTN_PRESS)
	{
		for (i = 0; i < NUMBER_SERVOS; ++i)
		{
			if (servo_angle[i] < ANGLE_MAX)
			{
				set_servo_angle(servo_angle[i]+30, i);
			} else
			{
				set_servo_angle(0, i);
			}
		}

		gf_BTN_PRESS = 0;
	}

	return;
}

/*
 * updates all servo PWM signals every tick
 */
void servo_tick(void )
{
	uint8_t i;

	for (i = 0; i < NUMBER_SERVOS; ++i)
	{
		servo_pulse_count[i]++;
		set_servo_control_pin_state(i);
	}

	return;
}

/*
 * sets servo control pin output state
 */
void set_servo_control_pin_state(uint8_t servo_number)
{
	if (servo_pulse_count[servo_number] >= PULSE_COUNT_MAX_TOTAL-1)
	{
		servo_pulse_count[servo_number] = 0;
	}

	if ((servo_pulse_count[servo_number] < servo_pulse_width[servo_number]) && (servo_pinstate[servo_number] != GPIO_PIN_SET))
	{
		servo_pinstate[servo_number] = GPIO_PIN_SET;
		HAL_GPIO_WritePin(servo_pin_port[servo_number], servo_pin_number[servo_number], GPIO_PIN_SET);

	} else if ((servo_pulse_count[servo_number] >= servo_pulse_width[servo_number]) && (servo_pinstate[servo_number] != GPIO_PIN_RESET))
	{
		servo_pinstate[servo_number] = GPIO_PIN_RESET;
		HAL_GPIO_WritePin(servo_pin_port[servo_number], servo_pin_number[servo_number], GPIO_PIN_RESET);
	}
}


/*
 * sets pulse width to number of pulses required for desired angle
 * store angle set
 */
void set_servo_angle(uint16_t angle, uint8_t servo_number)
{
	servo_angle[servo_number] = angle;
	servo_pulse_width[servo_number] = get_pulse_count(angle);

	return;
}

/*
 * converts desired angle to number of pulses required
 */
uint16_t get_pulse_count(uint16_t angle)
{
	return ((5*angle)/3) + 150;
}
