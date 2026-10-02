#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#include "main.h"

#define WiFi_UART							huart1
#define UART_RX_BUFFER_SIZE					1024

typedef struct
{
	char buffer[UART_RX_BUFFER_SIZE];
	char buffer_temp[UART_RX_BUFFER_SIZE];
	uint8_t rxd;
	volatile uint16_t input_p;
	volatile uint16_t output_p;
	volatile uint16_t buf_p;
	uint8_t temp;
} uart_hal_rx_type;

extern uart_hal_rx_type uart_hal_rx;

extern char r_Index[10], r_str[5];
extern char r_Roll0[10], r_Pitch0[10], r_Roll1[10], r_Pitch1[10], r_Roll2[10], r_Pitch2[10], r_Roll3[10], r_Pitch3[10], r_Roll4[10], r_Pitch4[10];
extern char r_Force0[2], r_Force1[2], r_Force2[2], r_Force3[2], r_Force4[2];
extern float f_Roll0, f_Pitch0, f_Roll1, f_Pitch1, f_Roll2, f_Pitch2, f_Roll3, f_Pitch3, f_Roll4, f_Pitch4;
extern char c_Force0, c_Force1, c_Force2, c_Force3, c_Force4;
extern uint8_t Onetime_flag, Realtime_recv, Realtime_flag, Onetime_Error, Realtime_Error;
extern int Wifi_status;

void Wifi_HW_Reset(void);
void uart_hal_rx_buffer_init(void);
uint8_t uart_hal_getchar(void);
void uart_hal_rx_monitor(void);
uint8_t Recieve_data(void);
uint8_t Realtime_Recieve_data(void);

#endif /* INC_WIFI_H_ */
