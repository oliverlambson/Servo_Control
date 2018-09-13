/*
 * usr.c
 *
 *  Created on: 05 Aug 2018
 *      Author: 18225713
 */
#include "usr.h"


/* PRIVATE FUNCTION PROTOTYPES */
void set_LEDS(void );
void set_servo_PWM(TIM_HandleTypeDef *htim, uint32_t* servo_pulse_width, uint16_t no_servos);
void set_servo_angle(uint16_t angle, uint8_t servo_number);
uint32_t get_pulse_width(uint16_t angle);
uint16_t get_map(uint16_t in_val, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);


/* PUBLIC FUNCTIONS */
/*
 * main.c initialisation
 */
void usr_init(TIM_HandleTypeDef *htim)
{
	uint8_t i;

	HAL_GPIO_WritePin(POT_PWR_GPIO_Port, POT_PWR_Pin, GPIO_PIN_SET);

  //tim2 init
  for (i = 0; i < NUMBER_SERVOS; ++i)
  {
	  set_servo_angle(0, i);
  }
  set_servo_PWM(htim, servo_pulse_width, NUMBER_SERVOS);

  __HAL_TIM_ENABLE(htim);
  HAL_TIM_PWM_Init(htim);
  HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_2);

  return;
}


/*
 * main.c infinite loop
 */
void usr_process(TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc)
{
//	uint8_t i;
	uint16_t pot_adc_in_val, servo_angle_val;
//	static uint16_t ADC_min = -1, ADC_max = 0;

	switch (ctrl_state) {
		case idle:
			if (gf_RTC_TICK)
			{
				gf_RTC_TICK = false;
			}

			if (gf_SYSTICK)
			{
				gf_SYSTICK = false;
			}

			if (gf_B1)
			{
				// change servo 0 angle
				if (servo_angle[0] < servo_angle_max[0])
				{
					set_servo_angle(servo_angle[0]+30, 0);
				} else
				{
					set_servo_angle(0, 0);
				}
				// change servo 2 angle
				if (servo_angle[1] < servo_angle_max[1])
				{
					set_servo_angle(servo_angle[1]+30, 1);
				} else
				{
					set_servo_angle(0, 1);
				}
				set_servo_PWM(htim, servo_pulse_width, NUMBER_SERVOS);

				gf_B1 = false;
			}

			if (gf_SERVO1_BTN)
			{
				ctrl_state = servo1_ctrl;
				gf_SERVO1_BTN = false;
				set_LEDS();
			}

			if (gf_SERVO2_BTN)
			{
				ctrl_state = servo2_ctrl;
				gf_SERVO2_BTN = false;
				set_LEDS();
			}

			if (gf_RUN_BTN)
			{
				ctrl_state = run;
				run_state = touch;
				gf_RUN_BTN = false;
				set_LEDS();
			}

			break;
		case servo1_ctrl:
			if (gf_RTC_TICK)
			{
				gf_RTC_TICK = false;
			}

			if (gf_SYSTICK)
			{
				// read ADC
				HAL_ADC_Start(hadc);
				HAL_ADC_PollForConversion(hadc, 10);
				pot_adc_in_val = HAL_ADC_GetValue(hadc);
				HAL_ADC_Stop(hadc);

				// set servo angle
				servo_angle_val = get_map(pot_adc_in_val, POT_ADC_IN_MIN, POT_ADC_IN_MAX, ANGLE_MIN, ANGLE_MAX);
				set_servo_angle(servo_angle_val, 0);
				set_servo_PWM(htim, servo_pulse_width, NUMBER_SERVOS);

				gf_SYSTICK = false;
			}

			if (gf_B1) {
				gf_B1 = false;
			}

			if (gf_SERVO1_BTN)
			{
				servo_angle_zero[0] = servo_angle[0];

				set_servo_angle(servo_angle_min[0], 0);
				set_servo_angle(servo_angle_min[1], 1);
				set_servo_PWM(htim, servo_pulse_width, NUMBER_SERVOS);

				ctrl_state = idle;
				set_LEDS();
				gf_SERVO1_BTN = false;
			}

			if (gf_SERVO2_BTN)
			{
				gf_SERVO2_BTN = false;
			}

			if (gf_RUN_BTN)
			{
				gf_RUN_BTN = false;
			}

			break;
		case servo2_ctrl:
			if (gf_RTC_TICK)
			{
				gf_RTC_TICK = false;
			}

			if (gf_SYSTICK)
			{
				// read ADC
				HAL_ADC_Start(hadc);
				HAL_ADC_PollForConversion(hadc, 10);
				pot_adc_in_val = HAL_ADC_GetValue(hadc);
				HAL_ADC_Stop(hadc);

				// set servo angle
				servo_angle_val = get_map(pot_adc_in_val, POT_ADC_IN_MIN, POT_ADC_IN_MAX, ANGLE_MIN, ANGLE_MAX);
				set_servo_angle(servo_angle_val, 1);
				set_servo_PWM(htim, servo_pulse_width, NUMBER_SERVOS);

				gf_SYSTICK = false;
			}

			if (gf_B1)
			{
				gf_B1 = false;
			}

			if (gf_SERVO1_BTN)
			{
				gf_SERVO1_BTN = false;
			}

			if (gf_SERVO2_BTN)
			{
				servo_angle_zero[1] = servo_angle[1];

				set_servo_angle(servo_angle_min[0], 0);
				set_servo_angle(servo_angle_min[1], 1);
				set_servo_PWM(htim, servo_pulse_width, NUMBER_SERVOS);

				ctrl_state = idle;
				set_LEDS();
				gf_SERVO2_BTN = false;
			}

			if (gf_RUN_BTN)
			{
				gf_RUN_BTN = false;
			}

			break;
		case run:
			if (gf_RTC_TICK)
			{
				gf_RTC_TICK = false;
			}

			if (gf_SYSTICK)
			{
				g_run_time++;
				switch (run_state) {
					case touch:
						set_servo_angle(servo_angle_zero[0], 0);
						set_servo_angle(servo_angle_min[1], 1);

						if (g_run_time >= 2500)
						{
							run_state = pick;
							g_run_time = 0;
						}
						break;
					case pick:
						set_servo_angle(servo_angle_max[0], 0);
						set_servo_angle(servo_angle_min[1], 1);

						if (g_run_time >= 10000)
						{
							run_state = move;
							g_run_time = 0;
						}
						break;
					case move:
						set_servo_angle(servo_angle_max[0], 0);
						set_servo_angle(servo_angle_zero[1], 1);

						if (g_run_time >= 5000)
						{
							run_state = reset;
							g_run_time = 0;
						}
						break;
					case reset:
						set_servo_angle(servo_angle_min[0], 0);
						if (g_run_time == 2500)
						{
							set_servo_angle(servo_angle_min[1], 1);
						}
						if (g_run_time >= 5000)
						{
							run_state = touch;
							g_run_time = 0;

							ctrl_state = idle;
							set_LEDS();
						}

						break;
					default:
						break;
				}
				set_servo_PWM(htim, servo_pulse_width, NUMBER_SERVOS);

				gf_SYSTICK = false;
			}

			if (gf_B1)
			{
				gf_B1 = false;
			}

			if (gf_SERVO1_BTN)
			{
				gf_SERVO1_BTN = false;
			}

			if (gf_SERVO2_BTN)
			{
				gf_SERVO2_BTN = false;
			}

			if (gf_RUN_BTN)
			{
				ctrl_state = idle;
				gf_RUN_BTN = false;
				set_LEDS();
			}

			break;
		default:
			if (gf_RTC_TICK)
			{
				gf_RTC_TICK = false;
			}

			if (gf_SYSTICK)
			{
				gf_SYSTICK = false;
			}

			if (gf_B1)
			{
				gf_B1 = false;
			}

			if (gf_SERVO1_BTN)
			{
				gf_SERVO1_BTN = false;
			}

			if (gf_SERVO2_BTN)
			{
				gf_SERVO2_BTN = false;
			}

			if (gf_RUN_BTN)
			{
				gf_RUN_BTN = false;
			}
			break;
	}

	return;
}



/* PRIVATE FUNCTIONS */
/*
 * sets LED outputs based on state
 */
void set_LEDS(void )
{
	switch (ctrl_state) {
		case idle:
			HAL_GPIO_WritePin(SERVO1_LED_GPIO_Port, SERVO1_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SERVO2_LED_GPIO_Port, SERVO2_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
			break;
		case servo1_ctrl:
			HAL_GPIO_WritePin(SERVO1_LED_GPIO_Port, SERVO1_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SERVO2_LED_GPIO_Port, SERVO2_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
			break;
		case servo2_ctrl:
			HAL_GPIO_WritePin(SERVO1_LED_GPIO_Port, SERVO1_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SERVO2_LED_GPIO_Port, SERVO2_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
			break;
		case run:
			HAL_GPIO_WritePin(SERVO1_LED_GPIO_Port, SERVO1_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SERVO2_LED_GPIO_Port, SERVO2_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
			break;
		default:
			HAL_GPIO_WritePin(SERVO1_LED_GPIO_Port, SERVO1_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SERVO2_LED_GPIO_Port, SERVO2_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
			break;
	}

	return;
}

/*
 * sets timer pulse width to stored pulse width value
 */
void set_servo_PWM(TIM_HandleTypeDef *htim, uint32_t* servo_pulse_width, uint16_t no_servos)
{
	uint16_t i;

	for (i = 0; i < no_servos; ++i) {
		__HAL_TIM_SET_COMPARE(htim, 4*i, servo_pulse_width[i]);
	}
	return;
}


/*
 * sets stored pulse width to number of pulses required for desired angle
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


/*
 * linearly maps a variable in a range to another range
 */
uint16_t get_map(uint16_t in_val, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	return (uint16_t) (((in_val-in_min)*(out_max-out_min))/(in_max-in_min) + out_min);
}
