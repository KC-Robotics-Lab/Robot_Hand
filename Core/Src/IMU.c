#include "IMU.h"
#include "i2c.h"
#include "math.h"
#include "I2CMultiplex.h"
#include "CLI.h"
#include "EEPROM.h"

Struct_MPU6050 MPU6050;

static float LSB_Sensitivity_ACC;
static float LSB_Sensitivity_GYRO;

uint8_t buf[100] = {0,};
float dt = 0.05;
float calibrate0[6]={0,}, calibrate1[6]={0,}, calibrate2[6]={0,}, calibrate3[6]={0,}, calibrate4[6]={0,};
float kalFilter0[3]={0,}, kalFilter1[3]={0,}, kalFilter2[3]={0,}, kalFilter3[3]={0,}, kalFilter4[3]={0,};
float raw0[6]={0,}, raw1[6]={0,}, raw2[6]={0,}, raw3[6]={0,}, raw4[6]={0,};
float Roll[5]={0,}, Pitch[5]={0,}, Yaw[5]={0,};
float Roll0_Char, Pitch0_Char;
float Roll_buf[5]={0,}, Pitch_buf[5]={0,}, Yaw_buf[5]={0,};
float Roll0_deg, Pitch0_deg, Roll1_deg, Pitch1_deg, Roll2_deg, Pitch2_deg, Roll3_deg, Pitch3_deg, Roll4_deg, Pitch4_deg;

void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
	HAL_I2C_Mem_Write(&IMU_I2C, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Write(&IMU_I2C, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data)
{
	HAL_I2C_Mem_Read(&IMU_I2C, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Read(&IMU_I2C, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

void MPU6050_Initialization(void)
{
	uint8_t who_am_i = 0;
#if (Debug_IMU0 == 1) || (Debug_IMU1 == 1) || (Debug_IMU2 == 1) || (Debug_IMU3 == 1) || (Debug_IMU4 == 1)
	printf("Checking MPU6050...\n");
#endif
	HAL_Delay(100);
	MPU6050_Readbyte(MPU6050_WHO_AM_I, &who_am_i);
#if (Debug_IMU0 == 1) || (Debug_IMU1 == 1) || (Debug_IMU2 == 1) || (Debug_IMU3 == 1) || (Debug_IMU4 == 1)
	if(who_am_i == 0x68)
	{
		printf("MPU6050 who_am_i = 0x%02x...OK\n", who_am_i);
	}
	else
	{
		printf("ERROR!\n");
		printf("MPU6050 who_am_i : 0x%02x should be 0x68\n", who_am_i);
		while(1)
		{
			printf("who am i error. Can not recognize mpu6050\n");
			HAL_Delay(100);
		}
	}
#endif
	//Reset the whole module before initialization
	MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x1<<7);
	HAL_Delay(100);

	//Power Management setting
	/* Default is sleep mode
	 * necessary to wake up MPU6050*/
	MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x00);
	HAL_Delay(50);

	//Sample rate divider
	/*Sample Rate = Gyroscope Output Rate / (1 + SMPRT_DIV) */
	//	MPU6050_Writebyte(MPU6050_SMPRT_DIV, 0x00); // ACC output rate is 1kHz, GYRO output rate is 8kHz
	MPU6050_Writebyte(MPU6050_SMPRT_DIV, 39); // Sample Rate = 200Hz
	HAL_Delay(50);

	//FSYNC and DLPF setting
	/*DLPF is set to 0*/
	MPU6050_Writebyte(MPU6050_CONFIG, 0x00);
	HAL_Delay(50);

	//GYRO FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-250 degree/s
	  1		+-500 degree/s
	  2		+-1000 degree/s
	  3		+-2000 degree/s	*/
	uint8_t FS_SCALE_GYRO = 0x0;
	MPU6050_Writebyte(MPU6050_GYRO_CONFIG, FS_SCALE_GYRO<<3);
	HAL_Delay(50);

	//ACCEL FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-2g
	  1		+-4g
	  2		+-8g
	  3		+-16g	*/
	uint8_t FS_SCALE_ACC = 0x0;
	MPU6050_Writebyte(MPU6050_ACCEL_CONFIG, FS_SCALE_ACC<<3);
	HAL_Delay(50);

	MPU6050_Get_LSB_Sensitivity(FS_SCALE_GYRO, FS_SCALE_ACC);
	//printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\n",LSB_Sensitivity_GYRO, LSB_Sensitivity_ACC);

	//Interrupt PIN setting
	uint8_t INT_LEVEL = 0x0; //0 - active high, 1 - active low
	uint8_t LATCH_INT_EN = 0x0; //0 - INT 50us pulse, 1 - interrupt clear required
	uint8_t INT_RD_CLEAR = 0x1; //0 - INT flag cleared by reading INT_STATUS, 1 - INT flag cleared by any read operation
	MPU6050_Writebyte(MPU6050_INT_PIN_CFG, (INT_LEVEL<<7)|(LATCH_INT_EN<<5)|(INT_RD_CLEAR<<4)); //
	HAL_Delay(50);

	//Interrupt enable setting
//	uint8_t DATA_RDY_EN = 0x1; // 1 - enable, 0 - disable
//	MPU6050_Writebyte(MPU6050_INT_ENABLE, DATA_RDY_EN);
//	HAL_Delay(50);

	//printf("MPU6050 setting is finished\n");
}
/*Get Raw Data from sensor*/

void ALL_IMU_Initialization(void)
{
	Select_I2CPort(0, 0);
	MPU6050_Initialization();
	Select_I2CPort(0, 1);
	MPU6050_Initialization();
	Select_I2CPort(0, 2);
	MPU6050_Initialization();
	Select_I2CPort(0, 3);
	MPU6050_Initialization();
	Select_I2CPort(0, 4);
	MPU6050_Initialization();

	ALL_IMU_Calibration_Read();
}

void MPU6050_Get6AxisRawData(Struct_MPU6050* mpu6050)
{
	uint8_t data[14];
	MPU6050_Readbytes(MPU6050_ACCEL_XOUT_H, 14, data);

	mpu6050->acc_x_raw = (data[0] << 8) | data[1];
	mpu6050->acc_y_raw = (data[2] << 8) | data[3];
	mpu6050->acc_z_raw = (data[4] << 8) | data[5];

	mpu6050->temperature_raw = (data[6] << 8) | data[7];

	mpu6050->gyro_x_raw = ((data[8] << 8) | data[9]);
	mpu6050->gyro_y_raw = ((data[10] << 8) | data[11]);
	mpu6050->gyro_z_raw = ((data[12] << 8) | data[13]);
}

void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
	switch(FS_SCALE_GYRO)
	{
	case 0:
		LSB_Sensitivity_GYRO = 131.f;
		break;
	case 1:
		LSB_Sensitivity_GYRO = 65.5f;
		break;
	case 2:
		LSB_Sensitivity_GYRO = 32.8f;
		break;
	case 3:
		LSB_Sensitivity_GYRO = 16.4f;
		break;
	}
	switch(FS_SCALE_ACC)
	{
	case 0:
		LSB_Sensitivity_ACC = 16384.f;
		break;
	case 1:
		LSB_Sensitivity_ACC = 8192.f;
		break;
	case 2:
		LSB_Sensitivity_ACC = 4096.f;
		break;
	case 3:
		LSB_Sensitivity_ACC = 2048.f;
		break;
	}
}

/*Convert Unit. acc_raw -> g, gyro_raw -> degree per second*/
void MPU6050_DataConvert(Struct_MPU6050* mpu6050)
{
	//printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\n",LSB_Sensitivity_GYRO,LSB_Sensitivity_ACC);
	mpu6050->acc_x = mpu6050->acc_x_raw / LSB_Sensitivity_ACC;
	mpu6050->acc_y = mpu6050->acc_y_raw / LSB_Sensitivity_ACC;
	mpu6050->acc_z = mpu6050->acc_z_raw / LSB_Sensitivity_ACC;

	mpu6050->temperature = (float)(mpu6050->temperature_raw)/340+36.53;

	mpu6050->gyro_x = mpu6050->gyro_x_raw / LSB_Sensitivity_GYRO;
	mpu6050->gyro_y = mpu6050->gyro_y_raw / LSB_Sensitivity_GYRO;
	mpu6050->gyro_z = mpu6050->gyro_z_raw / LSB_Sensitivity_GYRO;

//	printf("acc_x:%f, acc_y:%f, acc_z:%f    ", mpu6050->acc_x, mpu6050->acc_y, mpu6050->acc_z);
//	printf("gyro_x:%f, gyro_y:%f, gyro_z:%f\n", mpu6050->gyro_x, mpu6050->gyro_y, mpu6050->gyro_z);
}

void MPU6050_ProcessData(Struct_MPU6050* mpu6050)
{
	MPU6050_Get6AxisRawData(mpu6050);
	MPU6050_DataConvert(mpu6050);
}

void calibration(float* cal)
{
	uint8_t rawData[6]; uint16_t cnt;

	cnt = 100;

	for(int i = 0; i < cnt; i++)
	{
		uint8_t accStart = 0x3B;
		HAL_I2C_Mem_Read(&IMU_I2C, mpuAddr, accStart, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
		HAL_Delay(3);

		cal[0] += ((int16_t)(rawData[0] << 8 | rawData[1])) / LSB_Sensitivity_ACC; // x
		cal[1] += ((int16_t)(rawData[2] << 8 | rawData[3])) / LSB_Sensitivity_ACC; // y
		cal[2] += ((int16_t)(rawData[4] << 8 | rawData[5])) / LSB_Sensitivity_ACC; // z

		uint8_t gyroStart = 0x43;
		HAL_I2C_Mem_Read(&IMU_I2C, mpuAddr, gyroStart, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
		HAL_Delay(3);

		//GYRO
		cal[3] += ((int16_t)(rawData[0] << 8 | rawData[1])) / LSB_Sensitivity_GYRO; // x
		cal[4] += ((int16_t)(rawData[2] << 8 | rawData[3])) / LSB_Sensitivity_GYRO; // y
		cal[5] += ((int16_t)(rawData[4] << 8 | rawData[5])) / LSB_Sensitivity_GYRO; // z
	}

	cal[0] /= cnt;
	cal[1] /= cnt;
	cal[2] /= cnt;
	cal[3] /= cnt;
	cal[4] /= cnt;
	cal[5] /= cnt;

//	printf("%f, %f %f, %f %f, %f\n\r", cal[0], cal[1], cal[2], cal[3], cal[4], cal[5]);
}

void ALL_IMU_Calibration_Save(void)
{
	uint8_t cal0_temp[24], cal1_temp[24], cal2_temp[24], cal3_temp[24], cal4_temp[24];

	Select_I2CPort(0, 0);
	calibration(calibrate0);
	float2byte(calibrate0, cal0);
	Select_I2CPort(0, 1);
	calibration(calibrate1);
	float2byte(calibrate1, cal1);
	Select_I2CPort(0, 2);
	calibration(calibrate2);
	float2byte(calibrate2, cal2);
	Select_I2CPort(0, 3);
	calibration(calibrate3);
	float2byte(calibrate3, cal3);
	Select_I2CPort(0, 4);
	calibration(calibrate4);
	float2byte(calibrate4, cal4);

	EEPROM_Writebytes(IMUCAL0_ax, 24, cal0);
	HAL_Delay(10);
	EEPROM_Writebytes(IMUCAL1_ax, 24, cal1);
	HAL_Delay(10);
	EEPROM_Writebytes(IMUCAL2_ax, 24, cal2);
	HAL_Delay(10);
	EEPROM_Writebytes(IMUCAL3_ax, 24, cal3);
	HAL_Delay(10);
	EEPROM_Writebytes(IMUCAL4_ax, 24, cal4);
	HAL_Delay(10);

	EEPROM_Readbytes(IMUCAL0_ax, 24, cal0_temp);
	if(memcmp(cal0, cal0_temp, sizeof(cal0)) != 0)
	{
		printf("IMU0 Calibration ERROR\n");
		printf("IMU0 Calibration Value save again!\n");
		EEPROM_Writebytes(IMUCAL0_ax, 24, cal0);
	}
	EEPROM_Readbytes(IMUCAL1_ax, 24, cal1_temp);
	if(memcmp(cal1, cal1_temp, sizeof(cal1)) != 0)
	{
		printf("IMU1 Calibration ERROR\n");
		printf("IMU1 Calibration Value save again!\n");
		EEPROM_Writebytes(IMUCAL1_ax, 24, cal1);
	}
	EEPROM_Readbytes(IMUCAL2_ax, 24, cal2_temp);
	if(memcmp(cal2, cal2_temp, sizeof(cal2)) != 0)
	{
		printf("IMU2 Calibration ERROR\n");
		printf("IMU2 Calibration Value save again!\n");
		EEPROM_Writebytes(IMUCAL2_ax, 24, cal2);
	}
	EEPROM_Readbytes(IMUCAL3_ax, 24, cal3_temp);
	if(memcmp(cal3, cal3_temp, sizeof(cal3)) != 0)
	{
		printf("IMU3 Calibration ERROR\n");
		printf("IMU3 Calibration Value save again!\n");
		EEPROM_Writebytes(IMUCAL3_ax, 24, cal3);
	}
	EEPROM_Readbytes(IMUCAL4_ax, 24, cal4_temp);
	if(memcmp(cal4, cal4_temp, sizeof(cal4)) != 0)
	{
		printf("IMU4 Calibration ERROR\n");
		printf("IMU4 Calibration Value save again!\n");
		EEPROM_Writebytes(IMUCAL4_ax, 24, cal4);
	}

	ALL_IMU_Calibration_Read();
}

void ALL_IMU_Calibration_Read(void)
{
	EEPROM_Readbytes(IMUCAL0_ax, 24, cal0);
	HAL_Delay(100);
	byte2float(cal_f0, cal0);

	EEPROM_Readbytes(IMUCAL1_ax, 24, cal1);
	HAL_Delay(100);
	byte2float(cal_f1, cal1);

	EEPROM_Readbytes(IMUCAL2_ax, 24, cal2);
	HAL_Delay(100);
	byte2float(cal_f2, cal2);

	EEPROM_Readbytes(IMUCAL3_ax, 24, cal3);
	HAL_Delay(100);
	byte2float(cal_f3, cal3);

	EEPROM_Readbytes(IMUCAL4_ax, 24, cal4);
	HAL_Delay(100);
	byte2float(cal_f4, cal4);

	for(uint8_t i=0; i<6; i++)
	{
		calibrate0[i] = cal_f0[i];
		calibrate1[i] = cal_f1[i];
		calibrate2[i] = cal_f2[i];
		calibrate3[i] = cal_f3[i];
		calibrate4[i] = cal_f4[i];
	}

	printf("IMU0 calibrate -> AccX: %f, AccY: %f AccZ: %f, ZyX: %f ZyY: %f, ZyZ: %f\n\r", calibrate0[0], calibrate0[1], calibrate0[2], calibrate0[3], calibrate0[4], calibrate0[5]);
	printf("IMU1 calibrate -> AccX: %f, AccY: %f AccZ: %f, ZyX: %f ZyY: %f, ZyZ: %f\n\r", calibrate1[0], calibrate1[1], calibrate1[2], calibrate1[3], calibrate1[4], calibrate1[5]);
	printf("IMU2 calibrate -> AccX: %f, AccY: %f AccZ: %f, ZyX: %f ZyY: %f, ZyZ: %f\n\r", calibrate2[0], calibrate2[1], calibrate2[2], calibrate2[3], calibrate2[4], calibrate2[5]);
	printf("IMU3 calibrate -> AccX: %f, AccY: %f AccZ: %f, ZyX: %f ZyY: %f, ZyZ: %f\n\r", calibrate3[0], calibrate3[1], calibrate3[2], calibrate3[3], calibrate3[4], calibrate3[5]);
	printf("IMU4 calibrate -> AccX: %f, AccY: %f AccZ: %f, ZyX: %f ZyY: %f, ZyZ: %f\n\r", calibrate4[0], calibrate4[1], calibrate4[2], calibrate4[3], calibrate4[4], calibrate4[5]);
}

void readRaw(float *data, float* cal)
{
	uint8_t rawData[6];

	uint8_t accStart = 0x3B;
	HAL_I2C_Mem_Read(&IMU_I2C, mpuAddr, accStart, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);

	// ACC
	data[0] = ((int16_t)(rawData[0] << 8 | rawData[1])) / 16384.0 - cal[0]; // x
	data[1] = ((int16_t)(rawData[2] << 8 | rawData[3])) / 16384.0 - cal[1]; // y
	data[2] = ((int16_t)(rawData[4] << 8 | rawData[5])) / 16384.0 - cal[2] + 1; // z
	uint8_t gyroStart = 0x43;
	HAL_I2C_Mem_Read(&IMU_I2C, mpuAddr, gyroStart, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);

	//GYRO
	data[3] = ((int16_t)(rawData[0] << 8 | rawData[1])) / 131.0 - cal[3]; // x
	data[4] = ((int16_t)(rawData[2] << 8 | rawData[3])) / 131.0 - cal[4]; // y
	data[5] = ((int16_t)(rawData[4] << 8 | rawData[5])) / 131.0 - cal[5]; // z

//#if (Debug_IMU0 == 1) || (Debug_IMU1 == 1) || (Debug_IMU2 == 1) || (Debug_IMU3 == 1) || (Debug_IMU4 == 1)
//	printf("ax = %f  ay = %f  gx = %f  gy = %f  \n\r", data[0], data[1], data[3], data[4]);
//#endif
}

void complementary(float* raw, float* filtered, float dt)
{
	float acc[2];
	acc[0] = (180.0 / PI) * atan(raw[1] / sqrt(pow(raw[0], 2) + pow(raw[2], 2)));
	acc[1] = (-180.0 / PI) * atan(raw[0] / sqrt(pow(raw[1], 2) + pow(raw[2], 2)));

	float gyro[3];
	gyro[0] = filtered[0] + (raw[3] * dt);
	gyro[1] = filtered[1] + (raw[4] * dt);
	gyro[2] = filtered[2] + (raw[5] * dt);

	filtered[0] = (alpha * gyro[0]) + ((1-alpha) * acc[0]);
	filtered[1] = (alpha * gyro[1]) + ((1-alpha) * acc[1]);
	filtered[2] = gyro[2];
}

Kalman_t kalmanX = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

Kalman_t kalmanY = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

Kalman_t kalmanX1 = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

Kalman_t kalmanY1 = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

Kalman_t kalmanX2 = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

Kalman_t kalmanY2 = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

Kalman_t kalmanX3 = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

Kalman_t kalmanY3 = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

Kalman_t kalmanX4 = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

Kalman_t kalmanY4 = {
	.Q = {
		{0.001f, 0.0f},
		{0.0f, 0.003f}
	},
	.R = 0.03f,
	.P = {
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	},
	.X = {0.0f, 0.0f}
};

void kalmanFilter(Kalman_t* Kalman, float acc, float gyro, float dt)
{
	Kalman->X[0] += (gyro - Kalman->X[1]) * dt;

	Kalman->P[0][0] += (Kalman->P[1][1] * dt - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q[0][0]) * dt;
	Kalman->P[0][1] -= Kalman->P[1][1] * dt;
	Kalman->P[1][0] -= Kalman->P[1][1] * dt;
	Kalman->P[1][1] += Kalman->Q[1][1] * dt;

	float y = acc - Kalman->X[0];
	float s = Kalman->P[0][0] + Kalman->R;
	Kalman->K[0] = Kalman->P[0][0] / s;
	Kalman->K[1] = Kalman->P[1][0] / s;

	Kalman->X[0] += Kalman->K[0] * y;
	Kalman->X[1] += Kalman->K[1] * y;

	float PZZ = Kalman->P[0][0];
	float PZO = Kalman->P[0][1];
	Kalman->P[0][0] -= Kalman->K[0] * PZZ;
	Kalman->P[0][1] -= Kalman->K[0] * PZO;
	Kalman->P[1][0] -= Kalman->K[1] * PZZ;
	Kalman->P[1][1] -= Kalman->K[1] * PZO;
}

void kalman(float* raw, float* filtered, float dt)
{
	float acc[2], gyro[3];
	acc[0] = (180.0 / PI) * atan(raw[1] / sqrt(pow(raw[0], 2) + pow(raw[2], 2)));
	acc[1] = (-180.0 / PI) * atan(raw[0] / sqrt(pow(raw[1], 2) + pow(raw[2], 2)));
	gyro[0] = raw[3];
	gyro[1] = raw[4];
	gyro[2] = raw[5];

	kalmanFilter(&kalmanX, acc[0], gyro[0], dt);
	kalmanFilter(&kalmanY, acc[1], gyro[1], dt);

	filtered[0] = kalmanX.X[0];
	filtered[1] = kalmanY.X[0];
	filtered[2] += (raw[5] * dt);
}

void kalman1(float* raw, float* filtered, float dt)
{
	float acc[2], gyro[3];
	acc[0] = (180.0 / PI) * atan(raw[1] / sqrt(pow(raw[0], 2) + pow(raw[2], 2)));
	acc[1] = (-180.0 / PI) * atan(raw[0] / sqrt(pow(raw[1], 2) + pow(raw[2], 2)));
	gyro[0] = raw[3];
	gyro[1] = raw[4];
	gyro[2] = raw[5];

	kalmanFilter(&kalmanX1, acc[0], gyro[0], dt);
	kalmanFilter(&kalmanY1, acc[1], gyro[1], dt);

	filtered[0] = kalmanX1.X[0];
	filtered[1] = kalmanY1.X[0];
	filtered[2] += (raw[5] * dt);
}

void kalman2(float* raw, float* filtered, float dt)
{
	float acc[2], gyro[3];
	acc[0] = (180.0 / PI) * atan(raw[1] / sqrt(pow(raw[0], 2) + pow(raw[2], 2)));
	acc[1] = (-180.0 / PI) * atan(raw[0] / sqrt(pow(raw[1], 2) + pow(raw[2], 2)));
	gyro[0] = raw[3];
	gyro[1] = raw[4];
	gyro[2] = raw[5];

	kalmanFilter(&kalmanX2, acc[0], gyro[0], dt);
	kalmanFilter(&kalmanY2, acc[1], gyro[1], dt);

	filtered[0] = kalmanX2.X[0];
	filtered[1] = kalmanY2.X[0];
	filtered[2] += (raw[5] * dt);
}

void kalman3(float* raw, float* filtered, float dt)
{
	float acc[2], gyro[3];
	acc[0] = (180.0 / PI) * atan(raw[1] / sqrt(pow(raw[0], 2) + pow(raw[2], 2)));
	acc[1] = (-180.0 / PI) * atan(raw[0] / sqrt(pow(raw[1], 2) + pow(raw[2], 2)));
	gyro[0] = raw[3];
	gyro[1] = raw[4];
	gyro[2] = raw[5];

	kalmanFilter(&kalmanX3, acc[0], gyro[0], dt);
	kalmanFilter(&kalmanY3, acc[1], gyro[1], dt);

	filtered[0] = kalmanX3.X[0];
	filtered[1] = kalmanY3.X[0];
	filtered[2] += (raw[5] * dt);
}

void kalman4(float* raw, float* filtered, float dt)
{
	float acc[2], gyro[3];
	acc[0] = (180.0 / PI) * atan(raw[1] / sqrt(pow(raw[0], 2) + pow(raw[2], 2)));
	acc[1] = (-180.0 / PI) * atan(raw[0] / sqrt(pow(raw[1], 2) + pow(raw[2], 2)));
	gyro[0] = raw[3];
	gyro[1] = raw[4];
	gyro[2] = raw[5];

	kalmanFilter(&kalmanX4, acc[0], gyro[0], dt);
	kalmanFilter(&kalmanY4, acc[1], gyro[1], dt);

	filtered[0] = kalmanX4.X[0];
	filtered[1] = kalmanY4.X[0];
	filtered[2] += (raw[5] * dt);
}

void IMU_process(void)
{
	Select_I2CPort(0, 0);
	readRaw(raw0, calibrate0);
	kalman(raw0, kalFilter0, dt);
	Roll[0] = kalFilter0[0];
	Roll0_Char = trunc(Roll[0]);
	Pitch[0] = kalFilter0[1];
	Pitch0_Char = trunc(Pitch[0]);
#if 0
	Select_I2CPort(0, 1);
	readRaw(raw1, calibrate1);
	kalman1(raw1, kalFilter1, dt);
	Roll[1] = kalFilter1[0];
	Pitch[1] = kalFilter1[1];

	Select_I2CPort(0, 2);
	readRaw(raw2, calibrate2);
	kalman2(raw2, kalFilter2, dt);
	Roll[2] = kalFilter2[0];
	Pitch[2] = kalFilter2[1];

	Select_I2CPort(0, 3);
	readRaw(raw3, calibrate3);
	kalman3(raw3, kalFilter3, dt);
	Roll[3] = kalFilter3[0];
	Pitch[3] = kalFilter3[1];

	Select_I2CPort(0, 4);
	readRaw(raw4, calibrate4);
	kalman4(raw4, kalFilter4, dt);
	Roll[4] = kalFilter4[0];
	Pitch[4] = kalFilter4[1];
#endif

//#if (Debug_IMU0 == 1) || (Debug_IMU1 == 1) || (Debug_IMU2 == 1) || (Debug_IMU3 == 1) || (Debug_IMU4 == 1)
//	strcpy((char *)buf, "Roll0 : ");
//	SendToUART(buf);
//	sprintf((char *)buf, "%f", Roll[0]);			//roll
//	SendToUART(buf);
//	strcpy((char *)buf, "      ");
//	SendToUART(buf);
//	strcpy((char *)buf, "Pitch0 : ");
//	SendToUART(buf);
//	sprintf((char *)buf, "%f\n", Pitch[0]);			//pitch
//	SendToUART(buf);
//
//	strcpy((char *)buf, "Roll1 : ");
//	SendToUART(buf);
//	sprintf((char *)buf, "%f", Roll[1]);			//roll
//	SendToUART(buf);
//	strcpy((char *)buf, "      ");
//	SendToUART(buf);
//	strcpy((char *)buf, "Pitch1 : ");
//	SendToUART(buf);
//	sprintf((char *)buf, "%f\n", Pitch[1]);			//pitch
//	SendToUART(buf);
//#endif
}

