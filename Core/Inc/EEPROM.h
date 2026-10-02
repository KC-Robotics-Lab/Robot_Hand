#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"

#define I2CEEP											hi2c3
#define EEPROM_ADDR										0x50<<1

#define IMUCal_Confirm_Addr								0x7FFF
#define IMUCal_Confirm_Data								0xAA

#define	IMUCAL0_ax										0x0000
#define	IMUCAL1_ax										0x0020
#define	IMUCAL2_ax										0x0040
#define	IMUCAL3_ax										0x0060
#define	IMUCAL4_ax										0x0080 //~0x007c

//one index per 50byte
//name : 5byte, IMU : 4byte*2*5ea, Force : 1byte*5
#define Sensor_Val0										256
#define Index_jump										52

//0x7FFF

extern uint8_t Rdata;
extern uint8_t cal0[24], cal1[24], cal2[24], cal3[24], cal4[24];
extern float cal_f0[6], cal_f1[6], cal_f2[6], cal_f3[6], cal_f4[6];

void EEPROM_Writebyte(uint16_t reg_addr, uint8_t val);
void EEPROM_Writebytes(uint16_t reg_addr, uint8_t len, uint8_t* data);
void EEPROM_Readbyte(uint16_t reg_addr, uint8_t* data);
void EEPROM_Readbytes(uint16_t reg_addr, uint8_t len, uint8_t* data);

void float2byte_onedata(float val, uint8_t* output);
void float2byte(float* val, uint8_t* outdata);
void byte2float_onedata(float *val, uint8_t* outdata);
void byte2float(float* val, uint8_t* outdata);

#endif /* INC_EEPROM_H_ */
