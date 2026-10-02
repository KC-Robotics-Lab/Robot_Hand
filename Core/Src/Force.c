#include "Force.h"
#include "i2c.h"
#include "I2CMultiplex.h"
#include <math.h>

uint8_t Fstatus;
float force0, force1, force2, force3, force4;
float Fdata0, Fdata1, Fdata2, Fdata3, Fdata4;
float Temperature0, Temperature1, Temperature2;

// read I2C data and calculate force value(N)
void Force_Process(void)
{
	uint8_t force_data[4];
	float temp;
#if 0
	Select_I2CPort(1, 6);
	HAL_I2C_Master_Receive(&hi2c2, (0x28<<1), force_data, 4, HAL_MAX_DELAY);

	Fstatus = ((force_data[0] & 0xc0) >> 6);

	Fdata0 = (uint16_t)((force_data[0] & 0x3f) << 8) + force_data[1];
	force0 = ((Fdata0 - OUTPUT_MIN) * FORCE_MAX) / (OUTPUT_MAX - OUTPUT_MIN);

	Select_I2CPort(1, 7);
	HAL_I2C_Master_Receive(&hi2c2, (0x28<<1), force_data, 4, HAL_MAX_DELAY);

	Fstatus = ((force_data[0] & 0xc0) >> 6);

	Fdata1 = (uint16_t)((force_data[0] & 0x3f) << 8) + force_data[1];
	force1 = (Fdata1 - OUTPUT_MIN) * FORCE_MAX / (OUTPUT_MAX - OUTPUT_MIN);

#else

	Select_I2CPort(1, 0);
//	HAL_I2C_Master_Receive(&hi2c2, (0x28<<1), force_data, 4, HAL_MAX_DELAY);
	while(HAL_I2C_Master_Receive(&hi2c2, (0x28<<1), force_data, 4, HAL_MAX_DELAY) != HAL_OK)
	{
		break;
	}

	Fstatus = ((force_data[0] & 0xc0) >> 6);

	Fdata0 = (uint16_t)((force_data[0] & 0x3f) << 8) + force_data[1];
	force0 = ((Fdata0 - OUTPUT_MIN) * FORCE_MAX) / (OUTPUT_MAX - OUTPUT_MIN);

#if 0
	printf("Fdata = %d        ", Fdata);
	printf("force0 = %f N\n", force0);
#endif

	temp = (force_data[2] << 3) + (force_data[3] >> 5);
	Temperature0 = (((temp * 200) / 2047) - 50);

#if 0
	printf("temp = %f         ", temp);
	printf("Temperature0 = %f\n", Temperature0);
	printf("\n\r");
#endif

	Select_I2CPort(1, 1);
//	HAL_I2C_Master_Receive(&hi2c2, (0x28<<1), force_data, 4, HAL_MAX_DELAY);
	while(HAL_I2C_Master_Receive(&hi2c2, (0x28<<1), force_data, 4, HAL_MAX_DELAY) != HAL_OK)
	{
		break;
	}

	Fstatus = ((force_data[0] & 0xc0) >> 6);

	Fdata1 = (uint16_t)((force_data[0] & 0x3f) << 8) + force_data[1];
	force1 = (Fdata1 - OUTPUT_MIN) * FORCE_MAX / (OUTPUT_MAX - OUTPUT_MIN);

#if 0
	printf("Fdata = %d        ", Fdata);
	printf("force1 = %d N\n", force1);
#endif

	temp = (force_data[2] << 3) + (force_data[3] >> 5);
	Temperature1 = (((temp * 200) / 2047) - 50);

#if 0
	printf("temp = %f         ", temp);
	printf("Temperature1 = %f\n", Temperature1);
	printf("\n\r");
#endif

	Select_I2CPort(1, 2);
//	HAL_I2C_Master_Receive(&hi2c2, (0x28<<1), force_data, 4, HAL_MAX_DELAY);
	while(HAL_I2C_Master_Receive(&hi2c2, (0x28<<1), force_data, 4, HAL_MAX_DELAY) != HAL_OK)
	{
		break;
	}

	Fstatus = ((force_data[0] & 0xc0) >> 6);

	Fdata2 = (uint16_t)((force_data[0] & 0x3f) << 8) + force_data[1];
	force2 = (Fdata2 - OUTPUT_MIN) * FORCE_MAX / (OUTPUT_MAX - OUTPUT_MIN);

#if 0
	printf("Fdata = %d        ", Fdata);
	printf("force2 = %d N\n", force2);
#endif

	temp = (force_data[2] << 3) + (force_data[3] >> 5);
	Temperature2 = (((temp * 200) / 2047) - 50);

#if 0
	printf("temp = %f         ", temp);
	printf("Temperature2 = %f\n", Temperature2);
	printf("\n\r");
#endif

#endif
}

