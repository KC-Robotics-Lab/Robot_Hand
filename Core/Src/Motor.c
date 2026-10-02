#include "Motor.h"
#include "tim.h"
#include "I2CMultiplex.h"
#include "IMU.h"
#include "Force.h"
#include "WIFI.h"
#include "CLI.h"
#include "math.h"

uint16_t pwm0_val, pwm1_val, pwm2_val, pwm3_val, pwm4_val;
extern systick Systime;

// D131MW servo motor pwm initial
void Motor_PWM_init(void)
{
//	HAL_TIM_PWM_Start(&PWM_Tim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&PWM_Tim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&PWM_Tim2, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&PWM_Tim3, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&PWM_Tim3, TIM_CHANNEL_3);

	__HAL_TIM_SetCompare(&PWM_Tim2, TIM_CHANNEL_2, 350);
	__HAL_TIM_SetCompare(&PWM_Tim2, TIM_CHANNEL_1, 350);

//	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);

//	pwm_val = 20;
//	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
//	HAL_Delay(200);
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, STROKE_MAX);
//	TIM2->CCR1 = STROKE_MIN;
}

/*
 * pwm_val = 0,  pwm = 500,  stroke : 0mm
 * pwm_val = 20, pwm = 1000, stroke : 20mm
 * pwm_freq = ((Internal Clock)/(Prescaler + 1)) / (Period)
 * 50Hz = ((64MHz)/(127 + 1)) / (10000)
 * 50Hz = ((90MHz)/(179 + 1)) / (10000)
 * 50Hz = ((100MHz)/(200)) / (10000)
 */

// Not use
//void pq12_r_process(uint16_t pwm_val)
//{
//	uint16_t pwm;
//	pwm = -(pwm_val * 25) + STROKE_MAX;
//	__HAL_TIM_SetCompare(&PWM_Tim2, TIM_CHANNEL_2, pwm);
////	TIM2->CCR1 = pwm;
//}

// Not use
//void Runhand_Motor_Process(void)
//{
//	uint16_t pwm0, pwm1, pwm2, pwm3, pwm4;
//
////	if(force0 < 2)
//	{
//		pwm0_val = map(Roll_buf[0], -90, 90, 20, 0);
//		pwm0 = -(pwm0_val * 25) + 1000;
//		__HAL_TIM_SetCompare(&PWM_Tim3, TIM_CHANNEL_2, pwm0);
//	}
//
////	if(force1 < 2)
//	{
//		pwm1_val = map(Roll_buf[1], -90, 90, 40, 0);
//		pwm1 = -(pwm1_val * 25) + 1250;
//		__HAL_TIM_SetCompare(&PWM_Tim2, TIM_CHANNEL_1, pwm1);
//	}
//
////	if(force2 < 2)
//	{
//		pwm2_val = map(Roll_buf[2], -90, 90, 40, 0);
//		pwm2 = -(pwm2_val * 25) + 1250;
//		__HAL_TIM_SetCompare(&PWM_Tim3, TIM_CHANNEL_1, pwm2);
//	}
//
////	if(force3 < 2)
//	{
//		pwm3_val = map(Roll_buf[3], -90, 90, 20, 0);
//		pwm3 = -(pwm3_val * 25) + 1000;
//		__HAL_TIM_SetCompare(&PWM_Tim3, TIM_CHANNEL_3, pwm3);
//	}
//}

// Not use
//void Realtime_Motor_Process(void)
//{
//	uint16_t pwm0, pwm1, pwm2, pwm3, pwm4;
//
////	if(force0 <= c_Force0)
//	{//D131MW : 350~1000
//		pwm0_val = map(f_Roll0, 90, -90, 26, 0);
//		pwm0 = -(pwm0_val * 25) + 1000;
//		__HAL_TIM_SetCompare(&PWM_Tim3, TIM_CHANNEL_2, pwm0);
//	}
//
////	if(force1 < c_Force1)
//	{//D131MW : 350~1000
//		pwm1_val = map(f_Roll1, 90, -90, 26, 0);
//		pwm1 = -(pwm1_val * 25) + 1000;
//		__HAL_TIM_SetCompare(&PWM_Tim2, TIM_CHANNEL_1, pwm1);
//	}
//
////	if(force2 < c_Force2)
//	{//HS-311 : 250~1250
//		pwm2_val = map(f_Roll2, -90, 90, 40, 0);
//		pwm2 = -(pwm2_val * 25) + 1250;
//		__HAL_TIM_SetCompare(&PWM_Tim3, TIM_CHANNEL_1, pwm2);
//	}
//
////	if(force3 < c_Force3)
//	{//PQ12 : 500~1000
//		pwm3_val = map(f_Roll3, -90, 90, 20, 0);
//		pwm3 = -(pwm3_val * 25) + 1000;
//		__HAL_TIM_SetCompare(&PWM_Tim3, TIM_CHANNEL_3, pwm3);
//	}
////	if(force4 < c_Force4)
//	{
//
//	}
//}

#if 1

#define	max_deg				90
#define min_deg				-90
#define min_val				0
#define max_val				180
#define PWM_offset			4
#define max_pwm				1070
#define offset				100

// Run D131MW 2 servo motor
void One_fingur_2Motor(uint8_t OP)
{
	uint16_t pwm0, pwm1, pwm0_buf, pwm1_buf;
	if(OP == 0)
	{
		if(f_Roll0 < -40)
		{
			pwm0_buf = map(f_Roll0, max_deg, min_deg, max_val, min_val);
			pwm0_val = constrain(pwm0_buf, min_val, max_val);
//			if(pwm0_val <= max_val)
			{
				pwm0 = -(pwm0_val * PWM_offset) + max_pwm;
			}
			TIM2->CCR2 = pwm0-offset;
		}
		else if(f_Roll0 > -40)
		{
			pwm1_buf = map(f_Roll0, max_deg, min_deg, max_val, min_val);
			pwm1_val = constrain(pwm1_buf, min_val, max_val);
//			if(pwm1_val <= max_val)
			{
				pwm1 = -(pwm1_val * PWM_offset) + max_pwm;
			}
			TIM2->CCR1 = pwm1;

			pwm0_buf = map(f_Roll0, max_deg, min_deg, max_val, min_val);
			pwm0_val = constrain(pwm0_buf, min_val, max_val);
//			if(pwm0_val <= max_val)
			{
				pwm0 = -(pwm0_val * PWM_offset) + max_pwm;
			}
			TIM2->CCR2 = pwm0-offset;
		}
	}
	else if(OP == 1)
	{
		if(Roll_buf[0] < -40)
		{
			pwm0_buf = map(Roll_buf[0], max_deg, min_deg, max_val, min_val);
			pwm0_val = constrain(pwm0_buf, min_val, max_val);
//			if(pwm0_val <= max_val)
			{
				pwm0 = -(pwm0_val * PWM_offset) + max_pwm;
			}
			TIM2->CCR2 = pwm0-offset;
		}
		else if(Roll_buf[0] > -40)
		{
			pwm1_buf = map(Roll_buf[0], max_deg, min_deg, max_val, min_val);
			pwm1_val = constrain(pwm1_buf, min_val, max_val);
//			if(pwm1_val <= max_val)
			{
				pwm1 = -(pwm1_val * PWM_offset) + max_pwm;
			}
			TIM2->CCR1 = pwm1;

			pwm0_buf = map(Roll_buf[0], max_deg, min_deg, max_val, min_val);
			pwm0_val = constrain(pwm0_buf, min_val, max_val);
//			if(pwm0_val <= max_val)
			{
				pwm0 = -(pwm0_val * PWM_offset) + max_pwm;
			}
			TIM2->CCR2 = pwm0-offset;
		}
	}
}

#else

#define	max_deg				90
#define min_deg				-90
#define min_val				0
#define max_val				13
#define PWM_offset			50
#define max_pwm				1000
#define offset				100

void One_fingur_2Motor(void)
{
	uint16_t pwm0, pwm1;
#if 1
	if(f_Roll0 < -40)
	{
		pwm0_val = map(f_Roll0, max_deg, min_deg, max_val, min_val);
		if(pwm0_val <= max_val)
		{
			pwm0 = -(pwm0_val * PWM_offset) + max_pwm;
			TIM3->CCR2 = pwm0-offset;
		}
	}
	else if(f_Roll0 > -40)
	{
		pwm1_val = map(f_Roll0, max_deg, min_deg, max_val, min_val);
		if(pwm1_val <= max_val)
		{
			pwm1 = -(pwm1_val * PWM_offset) + max_pwm;
			TIM2->CCR1 = pwm1;
		}

		pwm0_val = map(f_Roll0, max_deg, min_deg, max_val, min_val);
		if(pwm0_val <= max_val)
		{
			pwm0 = -(pwm0_val * PWM_offset) + max_pwm;
			TIM3->CCR2 = pwm0-offset;
		}
	}
#else
	pwm1_val = map(f_Roll0, -90, 90, 40, 0);
	if(pwm1_val <= 40)
	{
		pwm1 = -(pwm1_val * 25) + 1250;
		__HAL_TIM_SetCompare(&PWM_Tim3, TIM_CHANNEL_1, pwm1);
	}
#endif
}
#endif


int conVAR = 0;
int currentVar=0;
int previousVar=0;

#define	max_deg_				98-8
#define min_deg_				-98+8
#define min_val_				0
#define max_val_				180
#define PWM_offset_				4
#define max_pwm_				1070
#define offset_					100

// Not use
//void Motor_Test(void)
//{
//	uint16_t pwm1;
//	conVAR = map(f_Roll0, max_deg_, min_deg_, max_val_, min_val_);
//	currentVar = constrain(conVAR, min_val_, max_val_);
////		if(pwm1_val <= max_val)
//	if((currentVar > previousVar+8) || (currentVar < previousVar-8))
//	{
//		pwm1 = -(currentVar * PWM_offset_) + max_pwm_;
//		HAL_Delay(40);
//		previousVar = currentVar;
//		TIM2->CCR1 = pwm1;
//	}
//
//
////	uint16_t pwm0, pwm1, pwm0_buf, pwm1_buf;
////	if(f_Roll0 < -40)
////	{
////		pwm0_buf = map(f_Roll0, max_deg_, min_deg_, max_val_, min_val_);
////		pwm0_val = constrain(pwm0_buf, min_val_, max_val_);
//////		if(pwm0_val <= max_val)
////		{
////			pwm0 = -(pwm0_val * PWM_offset_) + max_pwm_;
////		}
////		TIM3->CCR2 = pwm0-offset_;
////	}
////	else if(f_Roll0 > -40)
////	{
////		pwm1_buf = map(f_Roll0, max_deg_, min_deg_, max_val_, min_val_);
////		pwm1_val = constrain(pwm1_buf, min_val_, max_val_);
//////		if(pwm1_val <= max_val)
////		{
////			pwm1 = -(pwm1_val * PWM_offset_) + max_pwm_;
////		}
////		TIM2->CCR1 = pwm1;
////
////		pwm0_buf = map(f_Roll0, max_deg_, min_deg_, max_val_, min_val_);
////		pwm0_val = constrain(pwm0_buf, min_val_, max_val_);
//////		if(pwm0_val <= max_val)
////		{
////			pwm0 = -(pwm0_val * PWM_offset_) + max_pwm_;
////		}
////		TIM3->CCR2 = pwm0-offset_;
////	}
//}

//===================================================step motor==================================================

int Target_val0, Target_val1, Target_val2;
int diff_pwm0, diff_pwm1, diff_pwm2;
int Direction0, Direction1, Direction2;
int PWM_cnt0, PWM_cnt1, PWM_cnt2;
int c_stop0, c_stop1, c_stop2;
float fre[ACCELERATED_SPEED_LENGTH];
uint16_t period[ACCELERATED_SPEED_LENGTH];
uint16_t step0 = ACCELERATED_SPEED_LENGTH/2, step1 = ACCELERATED_SPEED_LENGTH/2, step2 = ACCELERATED_SPEED_LENGTH/2;

// S-Curve PWM calculation
// flexible : make flexible S-curve
// fre min / max : Motor freq range
void CalculateSModelLine(float fre[], unsigned short period[], float len, float fre_max, float fre_min, float flexible)
{
	int i=0;
	float deno ;
	float melo ;
	float delt = fre_max-fre_min;
    for(; i<len; i++)
    {
        melo = flexible* (i-len/2) / (len/2);
        deno = 1.0f / (1 + expf(-melo));  //expf is a library function of exponential(e)?
        fre[i] = delt * deno + fre_min;
        period[i] = (unsigned short)(FREQ / fre[i]); // 10000000 is the timer driver frequency
    }
    return ;
}

// PWM Interrupt(A4988 PWM)
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if(Direction2)
			{
				PWM_cnt2++;
			}
			else
			{
				PWM_cnt2--;
			}

			__HAL_TIM_SetAutoreload(&htim3, period[step2*2]);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, period[step2*2]/2);

			if(PWM_cnt2 == Target_val2)
			{
				HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_2);
				c_stop2 = 0;
				step2 = ACCELERATED_SPEED_LENGTH/2;
			}

			if(step2 == 0)
			{
				step2 = ACCELERATED_SPEED_LENGTH/2;
			}
			else step2--;
		}

		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if(Direction1)
			{
				PWM_cnt1++;
			}
			else
			{
				PWM_cnt1--;
			}

			__HAL_TIM_SetAutoreload(&htim3, period[step1*2]);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, period[step1*2]/2);

			if(PWM_cnt1 == Target_val1)
			{
				HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_3);
				c_stop1 = 0;
				step1 = ACCELERATED_SPEED_LENGTH/2;
			}

			if(step1 == 0)
			{
				step1 = ACCELERATED_SPEED_LENGTH/2;
			}
			else step1--;
		}
	}
	else if(htim->Instance == TIM4) // eger kesme kaynagi timer1 den gelmis ise
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if(Direction0)
			{
				PWM_cnt0++;
			}
			else
			{
				PWM_cnt0--;
			}

			__HAL_TIM_SetAutoreload(&htim4, period[step0*2]);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, period[step0*2]/2);

			if(PWM_cnt0 == Target_val0)
			{
				HAL_TIM_PWM_Stop_IT(&htim4,TIM_CHANNEL_1);
				c_stop0 = 0;
				step0 = ACCELERATED_SPEED_LENGTH/2;
			}

			if(step0 == 0)
			{
				step0 = ACCELERATED_SPEED_LENGTH/2;
			}
			else step0--;
		}

//		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//		{
//			if(Direction1)
//			{
//				PWM_cnt1++;
//			}
//			else
//			{
//				PWM_cnt1--;
//			}
//
//			if(PWM_cnt1 == Target_val1)
//			{
//				HAL_TIM_PWM_Stop_IT(&htim4,TIM_CHANNEL_2);
//				c_stop1 = 0;
//			}
//		}
	}
//	if(htim->Instance == TIM5)
//	{
//		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//		{
//			if(Direction2)
//			{
//				PWM_cnt2++;
//			}
//			else
//			{
//				PWM_cnt2--;
//			}
//
//			if(PWM_cnt2 == Target_val2)
//			{
//				HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_1);
//				c_stop2 = 0;
//			}
//		}
//	}
}

// Run stepper motor using Glove IMU data
void Stepper_motor(uint8_t motorID, float degree)
{
	switch(motorID)
	{
		case 0 :
			if (degree > 90)
			{
				Target_val0 = trunc(90 / 0.1525);
			}
			else if(degree < -90)
			{
				Target_val0 = trunc(-90 / 0.1525);
			}
			else
			{
				Target_val0 = trunc(degree / 0.1525);
			}

			diff_pwm0 = Target_val0 - PWM_cnt0;

			if(diff_pwm0 < 0)
			{
				HAL_GPIO_WritePin(Direction0_GPIO_Port,Direction0_Pin,GPIO_PIN_SET);
				Direction0 = 0;
				if(!c_stop0)
				{
					HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
					c_stop0 = 1;
				}
			}

			else if(diff_pwm0 > 0)
			{
				HAL_GPIO_WritePin(Direction0_GPIO_Port,Direction0_Pin,GPIO_PIN_RESET);
				Direction0 = 1;
				if(!c_stop0)
				{
					HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
					c_stop0 = 1;
				}
			}
			break;
		case 1 :
			if (degree > 90)
			{
				Target_val1 = trunc(90 / 0.1525);
			}
			else if(degree < -90)
			{
				Target_val1 = trunc(-90 / 0.1525);
			}
			else
			{
				Target_val1 = trunc(degree / 0.1525);
			}

			diff_pwm1 = Target_val1 - PWM_cnt1;

			if(diff_pwm1 < 0)
			{
				HAL_GPIO_WritePin(Direction1_GPIO_Port,Direction1_Pin,GPIO_PIN_SET);
				Direction1 = 0;
				if(!c_stop1)
				{
					HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_3);
					c_stop1 = 1;
				}
			}

			else if(diff_pwm1 > 0)
			{
				HAL_GPIO_WritePin(Direction1_GPIO_Port,Direction1_Pin,GPIO_PIN_RESET);
				Direction1 = 1;
				if(!c_stop1)
				{
					HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_3);
					c_stop1 = 1;
				}
			}
			break;
		case 2 :
			if (degree > 90)
			{
				Target_val2 = trunc(90 / 0.1525);
			}
			else if(degree < -90)
			{
				Target_val2 = trunc(-90 / 0.1525);
			}
			else
			{
				Target_val2 = trunc(degree / 0.1525);
			}

			diff_pwm2 = Target_val2 - PWM_cnt2;

			if(diff_pwm2 < 0)
			{
				HAL_GPIO_WritePin(Direction2_GPIO_Port,Direction2_Pin,GPIO_PIN_SET);
				Direction2 = 0;
				if(!c_stop2)
				{
					HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);
					c_stop2 = 1;
				}
				else
				{
//					HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_1);
				}
			}

			else if(diff_pwm2 > 0)
			{
				HAL_GPIO_WritePin(Direction2_GPIO_Port,Direction2_Pin,GPIO_PIN_RESET);
				Direction2 = 1;
				if(!c_stop2)
				{
					HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);
					c_stop2 = 1;
				}
				else
				{
//					HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_1);
				}
			}
			break;
	}
}

