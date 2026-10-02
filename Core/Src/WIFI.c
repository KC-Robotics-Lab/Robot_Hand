#include "WIFI.h"
#include "usart.h"
#include "I2CMultiplex.h"
#include "IMU.h"
#include "Force.h"
#include "EEPROM.h"
#include "CLI.h"
#include "math.h"

uart_hal_rx_type uart_hal_rx;
uint8_t cnt;
//char Data_cpy[1024] = {0,};
char r_Index[10], r_str[5];
char r_Roll0[10] = {0,}, r_Pitch0[10] = {0,}, r_Roll1[10] = {0,}, r_Pitch1[10] = {0,}, r_Roll2[10] = {0,}, r_Pitch2[10] = {0,}, r_Roll3[10] = {0,}, r_Pitch3[10] = {0,}, r_Roll4[10] = {0,}, r_Pitch4[10] = {0,};
char r_Force0[2] = {0,}, r_Force1[2] = {0,}, r_Force2[2] = {0,}, r_Force3[2] = {0,}, r_Force4[2] = {0,};
float f_Roll0, f_Pitch0, f_Roll1, f_Pitch1, f_Roll2, f_Pitch2, f_Roll3, f_Pitch3, f_Roll4, f_Pitch4;
char c_Force0, c_Force1, c_Force2, c_Force3, c_Force4;
uint8_t Onetime_flag = 0, Realtime_recv = 0, Realtime_flag = 0, Onetime_Error = 0, Realtime_Error = 0;
int Wifi_status;

void Wifi_HW_Reset(void)
{
//	HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
//	HAL_Delay(10);
//	HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
}

// ESP-01 Wifi buffer initial
void uart_hal_rx_buffer_init(void)
{
	uart_hal_rx.input_p = 0;
	uart_hal_rx.output_p = 0;
	uart_hal_rx.buf_p = 0;
	uart_hal_rx.temp = 0;
}

// ESP-01 receive string get and put buffer
uint8_t uart_hal_getchar(void)
{
	uint32_t reg = READ_REG(WiFi_UART.Instance->CR1);

	__HAL_UART_DISABLE_IT(&WiFi_UART, UART_IT_RXNE);
	if(uart_hal_rx.input_p == uart_hal_rx.output_p)
	{
		WRITE_REG(WiFi_UART.Instance->CR1, reg);
		return 0;
	}

	WRITE_REG(WiFi_UART.Instance->CR1, reg);
	uart_hal_rx.rxd = uart_hal_rx.buffer[uart_hal_rx.output_p++];
	if(uart_hal_rx.rxd != '\0')
	{
		uart_hal_rx.buffer_temp[uart_hal_rx.buf_p++] = uart_hal_rx.rxd;
	}
	if(uart_hal_rx.output_p >= UART_RX_BUFFER_SIZE)
	{
		uart_hal_rx.output_p = 0;
	}

	return 1;
}

// ESP-01 wifi receive data processing
void uart_hal_rx_monitor(void)
{
	while(uart_hal_getchar() != 0)
	{
//		HAL_UART_Transmit(&CLI_UART, (uint8_t*) uart_hal_rx.buffer_temp, uart_hal_rx.buf_p, HAL_MAX_DELAY);
		if(uart_hal_rx.rxd == '\n')
		{
			//OK
			if((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-4] == 'O') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'K'))
			{
				if(strstr(uart_hal_rx.buffer_temp, "AT+CIPSTATUS") != NULL)
				{
					for(uint8_t i=0; i<uart_hal_rx.buf_p; i++)
					{
						if(uart_hal_rx.buffer_temp[i] == ':')
						{
							Wifi_status = atoi(&uart_hal_rx.buffer_temp[i+1]);
							break;
						}
					}

					if((Wifi_status == 2) || (Wifi_status == 3) || (Wifi_status == 4))
					{
//						printf("Wifi Connected!\n");
					}
					else if(Wifi_status == 5)
					{
						printf("Wifi Disconnected!\n");
					}
					uart_hal_rx.buf_p = 0;
					memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
				}
//				HAL_UART_Transmit(&huart3, (uint8_t*) uart_hal_rx.buffer_temp, uart_hal_rx.buf_p, HAL_MAX_DELAY);	//Debugging
				else if(AT_Flag)
				{
					printf("\r\n");
					printf("AT OK\n");
					printf("\r\n");
					AT_Flag = 0;
					Menu_Tree(menu_cnt);
				}
				else if(AT_MODE_Flag)
				{
					printf("\r\n");
					printf("CW Mode Change OK\n");
					printf("\r\n");
					AT_MODE_Flag = 0;
					Menu_Tree(menu_cnt);
				}
				else if(AT_AP_Conn_Flag)
				{
					printf("\r\n");
					printf("AP Connection OK\n");
					printf("\r\n");
					AT_AP_Conn_Flag = 0;
					Menu_Tree(menu_cnt);
				}
				else if(Get_ip_flag)
				{
					uint8_t temp[50]={0,}, i, num;

					for(i=0; i<uart_hal_rx.buf_p;i++)
					{
						if(uart_hal_rx.buffer_temp[i] == '\n')
						{
							num = i;
							break;
						}
					}

					for(i=num+1; i<uart_hal_rx.buf_p-8; i++)
					{
						temp[i-(num+1)] = uart_hal_rx.buffer_temp[i];
					}
					printf("\r\n");
					printf("%s\n", temp);
					printf("\r\n");

					Get_ip_flag = 0;

					Menu_Tree(menu_cnt);
				}
				else if(AT_MUX_Flag)
				{
					printf("\r\n");
					printf("OK\n");
					printf("\r\n");
					AT_MUX_Flag = 0;
					Menu_Tree(menu_cnt);
				}
				else if(AT_Server_mod_Flag)
				{
					printf("\r\n");
					printf("OK\n");
					printf("\r\n");
					AT_Server_mod_Flag = 0;
					Menu_Tree(menu_cnt);
				}
				else if(AT_Close_Flag)
				{
					printf("\r\n");
					printf("Server Closed!\n");
					printf("\r\n");
					HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
					AT_Close_Flag= 0;
					Menu_Tree(menu_cnt);
				}
				uart_hal_rx.buf_p = 0;
			}
			//SW RESET
			else if((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-35] == 'R') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-31] == 'y'))
			{
//				HAL_UART_Transmit(&huart3, (uint8_t*) uart_hal_rx.buffer_temp, uart_hal_rx.buf_p, HAL_MAX_DELAY);	//Debugging
				if(AT_RST_Flag)
				{
					printf("\r\n");
					printf("AT RST OK\n");
					printf("\r\n");
					AT_RST_Flag = 0;
					Menu_Tree(menu_cnt);
				}
				uart_hal_rx.buf_p = 0;
				memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			}
			//Connection OK
			else if(((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-11] == 'Y') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-16] == 'A')
					&& (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'T') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-9] == 'C'))
				|| ((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'd') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-8] == 'L')))	//Linked
			{
//				HAL_UART_Transmit(&huart3, (uint8_t*) uart_hal_rx.buffer_temp, uart_hal_rx.buf_p, HAL_MAX_DELAY);	//Debugging
				if(AT_Conn_Flag)
				{
					printf("\r\n");
					printf("Server Connection OK\n");
					printf("\r\n");
					AT_Conn_Flag = 0;
					Menu_Tree(menu_cnt);
				}
				uart_hal_rx.buf_p = 0;
			}
			//when server mode, connection to client
			else if(strstr(uart_hal_rx.buffer_temp, "Link") != NULL)
			{
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				uart_hal_rx.buf_p = 0;
			}
			//No Connection
//			else if((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'k')	&& (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-8] == 'U'))	//Unlink
			else if(strstr(uart_hal_rx.buffer_temp, "Unlink") != NULL)
			{
//				HAL_UART_Transmit(&huart3, (uint8_t*) uart_hal_rx.buffer_temp, uart_hal_rx.buf_p, HAL_MAX_DELAY);	//Debugging
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				if(AT_Conn_Flag)
				{
					printf("\r\n");
					printf("Unlink! Try to connect to Server\n");
					printf("\r\n");
					Menu_Tree(menu_cnt);
				}
				else
				{
					printf("\r\n");
					printf("Unlink!\n");
					printf("\r\n");
					Menu_Tree(menu_cnt);
				}
				uart_hal_rx.buf_p = 0;
			}
			//no ip / busy error
//			else if(((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'p')	&& (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-7] == 'n')) || ((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-8] == 'y') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-11] == 'b') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-6] == 'p')))
			else if(strstr(uart_hal_rx.buffer_temp, "no ip") != NULL)
			{
//				HAL_UART_Transmit(&huart3, (uint8_t*) uart_hal_rx.buffer_temp, uart_hal_rx.buf_p, HAL_MAX_DELAY);	//Debugging

				printf("\r\n");
				printf("No IP!!!!\n");
				printf("\r\n");
				Menu_Tree(menu_cnt);
				uart_hal_rx.buf_p = 0;
				memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			}
			//no change(CW mode)
			else if(((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'e')	&& (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-11] == 'n')))
//			else if(strstr(uart_hal_rx.buffer_temp, "no change") != NULL)
			{
//				HAL_UART_Transmit(&huart3, (uint8_t*) uart_hal_rx.buffer_temp, uart_hal_rx.buf_p, HAL_MAX_DELAY);	//Debugging
				if(AT_MODE_Flag)
				{
					printf("\r\n");
					printf("Current CW Mode is same! No Change!!\n");
					printf("\r\n");
					AT_MODE_Flag = 0;
					Menu_Tree(menu_cnt);
				}
				else if(AT_Server_mod_Flag)
				{
					printf("\r\n");
					printf("Current Server Mode is same! No Change!!\n");
					printf("\r\n");
					HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
					AT_Server_mod_Flag = 0;
					Menu_Tree(menu_cnt);
				}
				uart_hal_rx.buf_p = 0;
			}
//			else if(((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'd')	&& (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-9] == 'b')))
//			{
////				HAL_UART_Transmit(&huart3, (uint8_t*) uart_hal_rx.buffer_temp, uart_hal_rx.buf_p, HAL_MAX_DELAY);	//Debugging
//
//				Menu_Tree(menu_cnt);
//				memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
//				uart_hal_rx.buf_p = 0;
//			}
			//Fail
//			else if((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-6] == 'F') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'L'))
			else if(strstr(uart_hal_rx.buffer_temp, "FAIL") != NULL)
			{
				if(AT_AP_Conn_Flag)
				{
					printf("\r\n");
					printf("AP Connection Fail\n");
					printf("\r\n");
					AT_AP_Conn_Flag = 0;
					Menu_Tree(menu_cnt);
				}
				uart_hal_rx.buf_p = 0;
				memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			}
			//DNS Fail
			else if((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-10] == 'D') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'l'))
			{
				printf("\r\n");
				printf("DNS Fail\n");
				printf("\r\n");
				Menu_Tree(menu_cnt);
				uart_hal_rx.buf_p = 0;
			}
			//Error
			else if((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-7] == 'E') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'R'))
			{
				if(AT_Conn_Flag)
				{
					printf("\r\n");
					printf("Connection ERROR\n");
					printf("\r\n");
				}
				else
				{
					printf("\r\n");
					printf("ERROR\n");
					printf("\r\n");
					Menu_Tree(menu_cnt);
				}
				uart_hal_rx.buf_p = 0;
				memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			}
			//when MUX=1
			else if(((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-7] == 'M') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == '0')))
			{
				if(AT_Close_Flag)
				{
					printf("\r\n");
					printf("Set MUX=1\n");
					printf("\r\n");
					AT_Close_Flag= 0;
					Menu_Tree(menu_cnt);
				}
				uart_hal_rx.buf_p = 0;
				memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			}
			//link is not
			else if(((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-13] == 'l') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 't')))
			{
				if(AT_Close_Flag)
				{
					printf("\r\n");
					printf("link is not\n");
					printf("\r\n");
					AT_Close_Flag= 0;
					Menu_Tree(menu_cnt);
				}
				uart_hal_rx.buf_p = 0;
				memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			}
			//Server mode and client connection
			else if(((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-6] == 'l') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'k')))
			{
				printf("\n");
				printf("link to Client\n");
				printf("\n");
				uart_hal_rx.buf_p = 0;
			}
//			//Server mode and client no connection
//			else if(((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-8] == 'U') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == 'k')))
//			{
//				printf("\r\n");
//				printf("Unlink to Client\n");
//				printf("\r\n");
//				Menu_Tree(menu_cnt);
//				uart_hal_rx.buf_p = 0;
//			}
			//not connection to server(busy s...)
//			else if((uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-11] == 'b') && (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-6] == 's')
//					&& (uart_hal_rx.buffer_temp[uart_hal_rx.buf_p-3] == '.'))
			else if(strstr(uart_hal_rx.buffer_temp, "busy s...") != NULL)
			{
				uart_hal_rx.buf_p = 0;
				memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			}
			else if(strstr(uart_hal_rx.buffer_temp, "link is builded") != NULL)
			{
				if(AT_MUX_Flag)
				{
					printf("\r\n");
					printf("link is builded\n");
					printf("\r\n");
					Menu_Tree(menu_cnt);
				}
				uart_hal_rx.buf_p = 0;
			}
			else if(strstr(uart_hal_rx.buffer_temp, "+IPD") != NULL)
			{
				if(strstr(uart_hal_rx.buffer_temp, "KCHAND+ECHO") != NULL)
				{
					Ready_Send_Echo();
				}
				else if(Realtime_Start)
				{
					if(strstr(uart_hal_rx.buffer_temp, "Done") != NULL)
					{
						Realtime_recv = 1;
					}

//					HAL_GPIO_WritePin(I2CMP_RST_GPIO_Port, I2CMP_RST_Pin, GPIO_PIN_SET);
//					HAL_Delay(1);
//					HAL_GPIO_WritePin(I2CMP_RST_GPIO_Port, I2CMP_RST_Pin, GPIO_PIN_RESET);
				}
				else if(Onetime_Start)
				{
					Onetime_Error = Recieve_data();
					if(Onetime_Error)
					{
						printf("Overflow1 Error\r\n");
						Ready_Onetime_Error_Send();
					}
					else
					{
						Ready_Onetime_rx_Send();
					}
					Onetime_flag = 1;
//					Menu_Tree(menu_cnt);
//					printf("\r\n");
//					printf("Complete Save data!\n");
//					printf("\r\n");
				}
				else
				{
					memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
				}
				uart_hal_rx.buf_p = 0;
			}
			else if(strstr(uart_hal_rx.buffer_temp, "we must restart") != NULL)
			{
				printf("\r\n");
				printf("we must restart!\n");
				printf("\r\n");
				Menu_Tree(menu_cnt);
				uart_hal_rx.buf_p = 0;
				memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			}
		}
		else if(uart_hal_rx.rxd == '>')
		{
			if(AT_ECHO_Flag)
			{
				ECHO();
				AT_ECHO_Flag = 0;
				uart_hal_rx.buf_p = 0;
			}
			else if(AT_SensorOP_Flag)
			{
				Hand_Sensor_OP();
				if(cnt%2 == 1) Sensor_Start = 0;
				else 		   Sensor_Start = 1;
				if(cnt == 2)   cnt = 0;
				cnt++;
				AT_SensorOP_Flag = 0;
				uart_hal_rx.buf_p = 0;
			}
			else if(Onetime_flag)
			{
				if(Onetime_Error)
				{
					Onetime_Error_Send();
				}
				else
				{
					Onetime_rx_Send();
				}

				uart_hal_rx.buf_p = 0;
				Onetime_flag = 0;
			}
			else if(Realtime_flag)
			{
				Realtime_rx_Send();
				uart_hal_rx.buf_p = 0;
				Realtime_flag = 0;
			}
		}
		else
		{

		}
	}
}

// Analyze data received from Hand and save in variables and EEPROM(One time)
uint8_t Recieve_data(void)
{
	uint16_t i, i_buf;
	uint8_t Roll_val_0[4] = {0,}, Pitch_val_0[4] = {0,}, Roll_val_1[4] = {0,}, Pitch_val_1[4] = {0,};
	uint8_t Roll_val_2[4] = {0,}, Pitch_val_2[4] = {0,}, Roll_val_3[4] = {0,}, Pitch_val_3[4] = {0,};
	uint8_t Roll_val_4[4] = {0,}, Pitch_val_4[4] = {0,};
	uint8_t Save_Index[2] = {0,}, Save_str[5] = {0,};
	uint16_t index_addr, Save_Index_val;

//	memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));

	HAL_Delay(5);

	for(i=0; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] == ':')
		{
			i_buf = i;
			break;
		}

		if(i > 20)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}

	memset(r_Index, 0, sizeof(r_Index));

	//Index
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Index[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}

	//STR
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_str[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Roll0
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Roll0[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Pitch0
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Pitch0[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Roll1
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Roll1[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}
	}
	//Pitch1
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Pitch1[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Roll2
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Roll2[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Pitch2
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Pitch2[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Roll3
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Roll3[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}
	}
	//Pitch3
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Pitch3[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Roll4
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Roll4[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Pitch4
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Pitch4[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//force0
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Force0[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}
	}
	//force1
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Force1[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//force2
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Force2[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//force3
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Force3[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//force4
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Force4[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}

	f_Roll0 = atof(r_Roll0);
	f_Pitch0 = atof(r_Pitch0);
	f_Roll1 = atof(r_Roll1);
	f_Pitch1 = atof(r_Pitch1);
	f_Roll2 = atof(r_Roll2);
	f_Pitch2 = atof(r_Pitch2);
	f_Roll3 = atof(r_Roll3);
	f_Pitch3 = atof(r_Pitch3);
	f_Roll4 = atof(r_Roll4);
	f_Pitch4 = atof(r_Pitch4);

	c_Force0 = atoi(r_Force0);
	c_Force1 = atoi(r_Force1);
	c_Force2 = atoi(r_Force2);
	c_Force3 = atoi(r_Force3);
	c_Force4 = atoi(r_Force4);

	Save_Index_val = ascii_to_hex(r_Index);
	index_addr = Start_Index_Addr_cal(Save_Index_val);

	float aaaaa;

	Save_Index[0] = Save_Index_val >> 8;
	Save_Index[1] = Save_Index_val & 0xff;

	EEPROM_Writebyte(index_addr, Save_Index[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+1, Save_Index[1]);
	HAL_Delay(10);

	printf("\r\n");
	printf("\r\n");
	printf("Index  : %d\r\n", Save_Index_val);
	printf("\r\n");

	Save_str[0] = r_str[0];
	Save_str[1] = r_str[1];
	Save_str[2] = r_str[2];
	Save_str[3] = r_str[3];
	Save_str[4] = r_str[4];
	printf("String : %c%c%c%c%c\r\n", Save_str[0], Save_str[1], Save_str[2], Save_str[3], Save_str[4]);
	printf("\r\n");

	EEPROM_Writebyte(index_addr+2, r_str[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+3, r_str[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+4, r_str[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+5, r_str[3]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+6, r_str[4]);
	HAL_Delay(10);

	f_Roll0 = floorf(f_Roll0 * 10) / 10;
	float2byte_onedata(f_Roll0, Roll_val_0);
//	EEPROM_Writebytes(index_addr+7, sizeof(Roll_val_0), Roll_val_0);
//	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+7, Roll_val_0[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+8, Roll_val_0[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+9, Roll_val_0[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+10, Roll_val_0[3]);
	HAL_Delay(10);
	byte2float_onedata(&aaaaa, Roll_val_0);
	printf("Roll0  : %f\n", aaaaa);

	f_Pitch0 = floorf(f_Pitch0 * 10) / 10;
	float2byte_onedata(f_Pitch0, Pitch_val_0);
//	EEPROM_Writebytes(index_addr+11, sizeof(Pitch_val_0), Pitch_val_0);
//	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+11, Pitch_val_0[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+12, Pitch_val_0[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+13, Pitch_val_0[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+14, Pitch_val_0[3]);
	HAL_Delay(10);
	byte2float_onedata(&aaaaa, Pitch_val_0);
	printf("Pitch0 : %f\n", aaaaa);

	f_Roll1 = floorf(f_Roll1 * 10) / 10;
	float2byte_onedata(f_Roll1, Roll_val_1);
//	EEPROM_Writebytes(index_addr+15, sizeof(Roll_val_1), Roll_val_1);
//	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+15, Roll_val_1[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+16, Roll_val_1[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+17, Roll_val_1[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+18, Roll_val_1[3]);
	HAL_Delay(10);
	byte2float_onedata(&aaaaa, Roll_val_1);
	printf("Roll1  : %f\n", aaaaa);

	f_Pitch1 = floorf(f_Pitch1 * 10) / 10;
	float2byte_onedata(f_Pitch1, Pitch_val_1);
//	EEPROM_Writebytes(index_addr+19, sizeof(Pitch_val_1), Pitch_val_1);
//	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+19, Pitch_val_1[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+20, Pitch_val_1[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+21, Pitch_val_1[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+22, Pitch_val_1[3]);
	HAL_Delay(10);
	byte2float_onedata(&aaaaa, Pitch_val_1);
	printf("Pitch1 : %f\n", aaaaa);

	f_Roll2 = floorf(f_Roll2 * 10) / 10;
	float2byte_onedata(f_Roll2, Roll_val_2);
//	EEPROM_Writebytes(index_addr+23, sizeof(Roll_val_2), Roll_val_2);
//	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+23, Roll_val_2[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+24, Roll_val_2[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+25, Roll_val_2[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+26, Roll_val_2[3]);
	HAL_Delay(10);
	byte2float_onedata(&aaaaa, Roll_val_2);
	printf("Roll2  : %f\n", aaaaa);

	f_Pitch2 = floorf(f_Pitch2 * 10) / 10;
	float2byte_onedata(f_Pitch2, Pitch_val_2);
//	EEPROM_Writebytes(index_addr+27, sizeof(Pitch_val_2), Pitch_val_2);
//	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+27, Pitch_val_2[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+28, Pitch_val_2[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+29, Pitch_val_2[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+30, Pitch_val_2[3]);
	HAL_Delay(10);
	byte2float_onedata(&aaaaa, Pitch_val_2);
	printf("Pitch2 : %f\n", aaaaa);

	f_Roll3 = floorf(f_Roll3 * 10) / 10;
	float2byte_onedata(f_Roll3, Roll_val_3);
//	EEPROM_Writebytes(index_addr+31, sizeof(Roll_val_3), Roll_val_3);
//	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+31, Roll_val_3[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+32, Roll_val_3[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+33, Roll_val_3[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+34, Roll_val_3[3]);
	HAL_Delay(10);
	byte2float_onedata(&aaaaa, Roll_val_3);
	printf("Roll3  : %f\n", aaaaa);

	f_Pitch3 = floorf(f_Pitch3 * 10) / 10;
	float2byte_onedata(f_Pitch3, Pitch_val_3);
//	EEPROM_Writebytes(index_addr+35, sizeof(Pitch_val_3), Pitch_val_3);
//	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+35, Pitch_val_3[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+36, Pitch_val_3[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+37, Pitch_val_3[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+38, Pitch_val_3[3]);
	HAL_Delay(10);
	byte2float_onedata(&aaaaa, Pitch_val_3);
	printf("Pitch3 : %f\n", aaaaa);

	f_Roll4 = floorf(f_Roll4 * 10) / 10;
	float2byte_onedata(f_Roll4, Roll_val_4);
//	EEPROM_Writebytes(index_addr+39, sizeof(Roll_val_4), Roll_val_4);
//	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+39, Roll_val_4[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+40, Roll_val_4[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+41, Roll_val_4[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+42, Roll_val_4[3]);
	HAL_Delay(10);
	byte2float_onedata(&aaaaa, Roll_val_4);
	printf("Roll4  : %f\n", aaaaa);

	f_Pitch4 = floorf(f_Pitch4 * 10) / 10;
	float2byte_onedata(f_Pitch4, Pitch_val_4);
//	EEPROM_Writebytes(index_addr+43, sizeof(Pitch_val_4), Pitch_val_4);
//	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+43, Pitch_val_4[0]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+44, Pitch_val_4[1]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+45, Pitch_val_4[2]);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+46, Pitch_val_4[3]);
	HAL_Delay(10);
	byte2float_onedata(&aaaaa, Pitch_val_4);
	printf("Pitch4 : %f\n", aaaaa);
	printf("\r\n");

	EEPROM_Writebyte(index_addr+47, (uint8_t)force0);
	printf("Force0 : %d\n", (uint8_t)force0);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+48, (uint8_t)force1);
	printf("Force1 : %d\n", (uint8_t)force1);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+49, (uint8_t)force2);
	printf("force2 : %d\n", (uint8_t)force2);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+50, (uint8_t)force3);
	printf("force3 : %d\n", (uint8_t)force3);
	HAL_Delay(10);
	EEPROM_Writebyte(index_addr+51, (uint8_t)force4);
	printf("force4 : %d\n", (uint8_t)force4);
	printf("\r\n");
	HAL_Delay(10);

	memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));

	Index_Clear_Sub_Menu_Tree();
	return 0;
}

// Analyze data received from Hand and save in variables(realtime)
uint8_t Realtime_Recieve_data(void)
{
	uint16_t i, i_buf;

//	char n, data[100];
//	n = sprintf(data, uart_hal_rx.buffer_temp);
//	for(i=0; i<n; i++)
//	{
//		printf("%c", data[i]);
//	}
//	printf("\r\n");

//	HAL_Delay(5);//HAL_Delay(10);//HAL_Delay(20);

	for(i=0; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] == ':')
		{
			i_buf = i;
			break;
		}

		if(i > 20)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}

//	//STR
//	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
//	{
//		if(uart_hal_rx.buffer_temp[i] != ',')
//		{
//			r_str[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
//		}
//		else if(uart_hal_rx.buffer_temp[i] == ',')
//		{
//			i_buf = i;
//			break;
//		}
//
//		if(i > 20+i_buf)
//		{
//			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
//			return 1;
//		}
//	}
	//Roll0
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Roll0[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Pitch0
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Pitch0[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Roll1
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Roll1[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Pitch1
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Pitch1[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Roll2
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Roll2[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Pitch2
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Pitch2[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Roll3
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Roll3[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Pitch3
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Pitch3[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Roll4
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Roll4[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//Pitch4
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Pitch4[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//force0
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Force0[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//force1
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Force1[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//force2
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Force2[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//force3
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Force3[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}
	//force4
	for(i=i_buf+1; i<sizeof(uart_hal_rx.buffer_temp); i++)
	{
		if(uart_hal_rx.buffer_temp[i] != ',')
		{
			r_Force4[i-i_buf-1] = uart_hal_rx.buffer_temp[i];
		}
		else if(uart_hal_rx.buffer_temp[i] == ',')
		{
			i_buf = i;
			break;
		}

		if(i > 20+i_buf)
		{
			memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			return 1;
		}
	}

	f_Roll0 = atof(r_Roll0);
	f_Pitch0 = atof(r_Pitch0);
	f_Roll1 = atof(r_Roll1);
	f_Pitch1 = atof(r_Pitch1);
	f_Roll2 = atof(r_Roll2);
	f_Pitch2 = atof(r_Pitch2);
	f_Roll3 = atof(r_Roll3);
	f_Pitch3 = atof(r_Pitch3);
	f_Roll4 = atof(r_Roll4);
	f_Pitch4 = atof(r_Pitch4);

	c_Force0 = atoi(r_Force0);
	c_Force1 = atoi(r_Force1);
	c_Force2 = atoi(r_Force2);
	c_Force3 = atoi(r_Force3);
	c_Force4 = atoi(r_Force4);

	f_Roll0 = trunc(f_Roll0);
	f_Pitch0 = trunc(f_Pitch0);
	f_Roll1 = trunc(f_Roll1);
	f_Pitch1 = trunc(f_Pitch1);
	f_Roll2 = trunc(f_Roll2);
	f_Pitch2 = trunc(f_Pitch2);
	f_Roll3 = trunc(f_Roll3);
	f_Pitch3 = trunc(f_Pitch3);
	f_Roll4 = trunc(f_Roll4);
	f_Pitch4 = trunc(f_Pitch4);

//	printf("%1f  %1f  %1f  %1f  %1f  %1f  %1f  %1f\r\n", f_Roll0, f_Pitch0, f_Roll1, f_Pitch1,  f_Roll2, f_Pitch2, f_Roll3, f_Pitch3);

	memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
	return 0;
}


