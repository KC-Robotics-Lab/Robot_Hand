#include "I2CMultiplex.h"
#include "i2c.h"
#include "usart.h"

uint8_t Buffer[25] = {0};
uint8_t Space[] = " - ";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";

void I2C_Find_Process(void)
{
#if 0
	for(uint8_t i=1; i<128; i++)
	{
		HAL_UART_Transmit(&huart1, StartMSG, sizeof(StartMSG), HAL_MAX_DELAY);
		uint8_t ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
		if (ret != HAL_OK) /* No ACK Received At That Address */
		{
			HAL_UART_Transmit(&huart1, Space, sizeof(Space), HAL_MAX_DELAY);
		}
		else if(ret == HAL_OK)
		{
			sprintf(Buffer, "0x%X", i);
			HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), HAL_MAX_DELAY);
		}
	}
	HAL_UART_Transmit(&huart1, EndMSG, sizeof(EndMSG), HAL_MAX_DELAY);
	HAL_Delay(10);
#endif
}

// Not use
//void IMU_I2CMultiplexer0_RST(void)
//{
//	HAL_GPIO_WritePin(I2CMP_RST_GPIO_Port, I2CMP_RST_Pin, GPIO_PIN_SET);
//}

// I2C Multiplexer Reset
void Force_I2CMultiplexer1_RST(void)
{
	HAL_GPIO_WritePin(I2CMP1_RST_GPIO_Port, I2CMP1_RST_Pin, GPIO_PIN_SET);
}

// I2C port selection in I2C Multiplexer
void Select_I2CPort(uint8_t I2Cnum, uint8_t port)
{
	uint8_t port_TX;

	port_TX = (1 << port) & 0xFF;
	if(I2Cnum== 0)
	{
		HAL_I2C_Master_Transmit(&I2CNUM0, I2CMADDR0, &port_TX, 1, HAL_MAX_DELAY);
	}
	else if(I2Cnum == 1)
	{
		HAL_I2C_Master_Transmit(&I2CNUM1, I2CMADDR0, &port_TX, 1, HAL_MAX_DELAY);
	}
	//HAL_Delay(10);
#if (I2C_Multiplexer == 1)
	uint8_t port_RX;
	HAL_I2C_Master_Receive(&I2CNUM0, I2CMADDR0, &port_RX, 1, HAL_MAX_DELAY);
	printf("port = %x\n", port_RX);
#endif
}



