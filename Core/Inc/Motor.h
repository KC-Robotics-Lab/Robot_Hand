#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

extern uint16_t pwm0_val, pwm1_val, pwm2_val, pwm3_val, pwm4_val;

#define PWM_Tim2								htim2
#define PWM_Tim3								htim3
#define STROKE_MIN								500		//robot finger max stretch
#define STROKE_MAX								1000	//robot finger max bending

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void Motor_PWM_init(void);
void pq12_r_process(uint16_t pwm_val);
void Runhand_Motor_Process(void);
void Realtime_Motor_Process(void);
void One_fingur_2Motor(uint8_t OP);

void Motor_Test(void);

#define ACCELERATED_SPEED_LENGTH 2360
#define FRE_MIN 1200
#define FRE_MAX 600
#define FREQ	1000000
#define flex	4

extern float fre[ACCELERATED_SPEED_LENGTH];
extern uint16_t period[ACCELERATED_SPEED_LENGTH];

void CalculateSModelLine(float fre[], unsigned short period[], float len, float fre_max, float fre_min, float flexible);
void Stepper_motor(uint8_t motorID, float degree);

#endif /* INC_MOTOR_H_ */
