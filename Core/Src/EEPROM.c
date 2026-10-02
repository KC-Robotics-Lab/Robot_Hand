#include "EEPROM.h"
#include "i2c.h"
#include "I2CMultiplex.h"

uint8_t Rdata;
uint8_t cal0[24], cal1[24], cal2[24], cal3[24], cal4[24];
float cal_f0[6], cal_f1[6], cal_f2[6], cal_f3[6], cal_f4[6];

void EEPROM_Writebyte(uint16_t reg_addr, uint8_t val)
{
#if Use_Multiplex
	HAL_I2C_Mem_Write(&I2CEEP, EEPROM_ADDR, reg_addr, I2C_MEMADD_SIZE_16BIT, &val, 1, HAL_MAX_DELAY);
#else
	Select_I2CPort(1, 5);
	HAL_I2C_Mem_Write(&I2CNUM1, EEPROM_ADDR, reg_addr, I2C_MEMADD_SIZE_16BIT, &val, 1, HAL_MAX_DELAY);
#endif
}

void EEPROM_Writebytes(uint16_t reg_addr, uint8_t len, uint8_t* data)
{
#if Use_Multiplex
	HAL_I2C_Mem_Write(&I2CEEP, EEPROM_ADDR, reg_addr, I2C_MEMADD_SIZE_16BIT, data, len, HAL_MAX_DELAY);
#else
	Select_I2CPort(1, 5);
	HAL_I2C_Mem_Write(&I2CNUM1, EEPROM_ADDR, reg_addr, I2C_MEMADD_SIZE_16BIT, data, len, HAL_MAX_DELAY);
#endif
}

void EEPROM_Readbyte(uint16_t reg_addr, uint8_t* data)
{
#if Use_Multiplex
	HAL_I2C_Mem_Read(&I2CEEP, EEPROM_ADDR, reg_addr, I2C_MEMADD_SIZE_16BIT, data, 1, HAL_MAX_DELAY);
#else
	Select_I2CPort(1, 5);
	HAL_I2C_Mem_Read(&I2CNUM1, EEPROM_ADDR, reg_addr, I2C_MEMADD_SIZE_16BIT, data, 1, HAL_MAX_DELAY);
#endif
}

void EEPROM_Readbytes(uint16_t reg_addr, uint8_t len, uint8_t* data)
{
#if Use_Multiplex
	HAL_I2C_Mem_Read(&I2CEEP, EEPROM_ADDR, reg_addr, I2C_MEMADD_SIZE_16BIT, data, len, HAL_MAX_DELAY);
#else
	Select_I2CPort(1, 5);
	HAL_I2C_Mem_Read(&I2CNUM1, EEPROM_ADDR, reg_addr, I2C_MEMADD_SIZE_16BIT, data, len, HAL_MAX_DELAY);
#endif
}

typedef union
{
  unsigned char a[4];
  float b;
} union_data;

union_data data_s,data_r;

typedef union
{
  unsigned long a;
  float b;
} union_data1;

union_data1 data_sum;

void float2byte_onedata(float val, uint8_t* outdata)
{
	data_s.b = val;
	memcpy(&data_r.a, &data_s.a,sizeof(data_s.a));
	outdata[0] = data_r.a[0];
	outdata[1] = data_r.a[1];
	outdata[2] = data_r.a[2];
	outdata[3] = data_r.a[3];

}

void float2byte(float* val, uint8_t* outdata)
{
	data_s.b = val[0];
	memcpy(&data_r.a, &data_s.a,sizeof(data_s.a));
	outdata[0] = data_r.a[0];
	outdata[1] = data_r.a[1];
	outdata[2] = data_r.a[2];
	outdata[3] = data_r.a[3];

	data_s.b = val[1];
	memcpy(&data_r.a, &data_s.a,sizeof(data_s.a));
	outdata[4] = data_r.a[0];
	outdata[5] = data_r.a[1];
	outdata[6] = data_r.a[2];
	outdata[7] = data_r.a[3];

	data_s.b = val[2];
	memcpy(&data_r.a, &data_s.a,sizeof(data_s.a));
	outdata[8] = data_r.a[0];
	outdata[9] = data_r.a[1];
	outdata[10] = data_r.a[2];
	outdata[11] = data_r.a[3];

	data_s.b = val[3];
	memcpy(&data_r.a, &data_s.a,sizeof(data_s.a));
	outdata[12] = data_r.a[0];
	outdata[13] = data_r.a[1];
	outdata[14] = data_r.a[2];
	outdata[15] = data_r.a[3];

	data_s.b = val[4];
	memcpy(&data_r.a, &data_s.a,sizeof(data_s.a));
	outdata[16] = data_r.a[0];
	outdata[17] = data_r.a[1];
	outdata[18] = data_r.a[2];
	outdata[19] = data_r.a[3];

	data_s.b = val[5];
	memcpy(&data_r.a, &data_s.a,sizeof(data_s.a));
	outdata[20] = data_r.a[0];
	outdata[21] = data_r.a[1];
	outdata[22] = data_r.a[2];
	outdata[23] = data_r.a[3];
}

void byte2float_onedata(float *val, uint8_t* outdata)
{
	data_sum.a = (outdata[3] << 24) + (outdata[2] << 16) + (outdata[1] << 8) + (outdata[0] & 0xFF);
	*val = data_sum.b;
}

void byte2float(float* val, uint8_t* outdata)
{
	data_sum.a = (outdata[3] << 24) + (outdata[2] << 16) + (outdata[1] << 8) + (outdata[0] & 0xFF);
	val[0] = data_sum.b;

	data_sum.a = (outdata[7] << 24) + (outdata[6] << 16) + (outdata[5] << 8) + (outdata[4] & 0xFF);
	val[1] = data_sum.b;

	data_sum.a = (outdata[11] << 24) + (outdata[10] << 16) + (outdata[9] << 8) + (outdata[8] & 0xFF);
	val[2] = data_sum.b;

	data_sum.a = (outdata[15] << 24) + (outdata[14] << 16) + (outdata[13] << 8) + (outdata[12] & 0xFF);
	val[3] = data_sum.b;

	data_sum.a = (outdata[19] << 24) + (outdata[18] << 16) + (outdata[17] << 8) + (outdata[16] & 0xFF);
	val[4] = data_sum.b;

	data_sum.a = (outdata[23] << 24) + (outdata[22] << 16) + (outdata[21] << 8) + (outdata[20] & 0xFF);
	val[5] = data_sum.b;
}

