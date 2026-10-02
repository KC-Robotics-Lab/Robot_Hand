#ifndef INC_I2CMULTIPLEX_H_
#define INC_I2CMULTIPLEX_H_

#include "main.h"

#define I2CNUM0											hi2c1
#define I2CNUM1											hi2c2

#define I2CMADDR0										0x70<<1
#define I2CMADDR1										0x71<<1
#define I2CMADDR2										0x72<<1
#define I2CMADDR3										0x73<<1
#define I2CMADDR4										0x74<<1
#define I2CMADDR5										0x75<<1
#define I2CMADDR6										0x76<<1
#define I2CMADDR7										0x77<<1

void I2C_Find_Process(void);
//void IMU_I2CMultiplexer0_RST(void);
void Force_I2CMultiplexer1_RST(void);
void Select_I2CPort(uint8_t I2Cnum, uint8_t port);

#endif /* INC_I2CMULTIPLEX_H_ */
