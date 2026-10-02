#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"

#define IMU_I2C											hi2c1
#define	mpuAddr											0x68 << 1
#define PI 												3.141592
#define alpha 											0.98

#define MPU6050_ADDR									0x68<<1

#define MPU6050_SMPRT_DIV								0X19
#define MPU6050_WHO_AM_I 								0X75
#define MPU6050_CONFIG 									0X1A
#define MPU6050_GYRO_CONFIG 							0X1B
#define MPU6050_ACCEL_CONFIG 							0X1C
#define MPU6050_INT_PIN_CFG 							0X37
#define MPU6050_INT_ENABLE 								0X38
#define MPU6050_INT_STATUS 								0X3A
#define MPU6050_ACCEL_XOUT_H 							0X3B
#define MPU6050_ACCEL_XOUT_L 							0X3C
#define MPU6050_PWR_MGMT_1 								0X6B //most important

extern uint8_t buf[100];

typedef struct _MPU6050{
	short acc_x_raw;
	short acc_y_raw;
	short acc_z_raw;
	short temperature_raw;
	short gyro_x_raw;
	short gyro_y_raw;
	short gyro_z_raw;

	float acc_x;
	float acc_y;
	float acc_z;
	float temperature;
	float gyro_x;
	float gyro_y;
	float gyro_z;
}Struct_MPU6050;

extern Struct_MPU6050 MPU6050;

extern float calibrate0[6], calibrate1[6], calibrate2[6], calibrate3[6], calibrate4[6];
extern float kalFilter0[3], kalFilter1[3], kalFilter2[3], kalFilter3[3], kalFilter4[3];
extern float raw0[6], raw1[6], raw2[6], raw3[6], raw4[6];
extern float dt;
extern float Roll[5], Pitch[5], Yaw[5];
extern float Roll0_Char, Pitch0_Char;
extern float Roll_buf[5], Pitch_buf[5], Yaw_buf[5];
extern float Roll0_deg, Pitch0_deg, Roll1_deg, Pitch1_deg, Roll2_deg, Pitch2_deg, Roll3_deg, Pitch3_deg, Roll4_deg, Pitch4_deg;

void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val);
void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data);
void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU6050_Initialization(void);
void ALL_IMU_Initialization(void);
void MPU6050_Get6AxisRawData(Struct_MPU6050* mpu6050);
void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC);
void MPU6050_DataConvert(Struct_MPU6050* mpu6050);
void MPU6050_ProcessData(Struct_MPU6050* mpu6050);
void calibration(float* cal);
void ALL_IMU_Calibration_Save(void);
void ALL_IMU_Calibration_Read(void);
void readRaw(float *data, float* cal);

void complementary(float* raw, float* filtered, float dt);

typedef struct {
	float Q[2][2];
	float R;
	float X[2];
	float P[2][2];
	float K[2];
} Kalman_t;

void kalmanFilter(Kalman_t* Kalman, float acc, float gyro, float dt);
void kalman(float* raw, float* filtered, float dt);
void kalman1(float* raw, float* filtered, float dt);
void kalman2(float* raw, float* filtered, float dt);
void kalman3(float* raw, float* filtered, float dt);
void kalman4(float* raw, float* filtered, float dt);
void IMU_process(void);

#endif /* INC_IMU_H_ */
