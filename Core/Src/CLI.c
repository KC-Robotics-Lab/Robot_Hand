#include "CLI.h"
#include "usart.h"
#include "I2CMultiplex.h"
#include "IMU.h"
#include "Force.h"
#include "EEPROM.h"
#include "WIFI.h"
#include "Motor.h"
#include "math.h"

uart_rx_CLI uart_hal_rx_CLI;
main_menu_t main_menu;

uint8_t menu_cnt = 0, submenu_cnt = 0;
uint8_t	AP_Conn_submenu = 0, Milti_Conn_submenu = 0, Client_Conn_submenu = 0;
uint8_t CWmod_submenu = 0, Server_mod_submenu = 0, Close_submenu = 0;
uint8_t Index_name_submenu = 0, Hand_OP_submenu = 0, Index_Read_submenu = 0, Index_clear_submenu = 0, Sensor_OP_submenu = 0, Realtime_OP_submenu = 0;

uint16_t CWMode_val = 0, Multi_Conn_val = 0, WifiClient_Port = 0, Server_mod_val = 0, Server_port_val = 0, close_val = 0, Save_Index_val = 0, Index_val = 0;
char CW_Mode[50] = {0,}, Wifi_SSID[50] = {0,}, Wifi_PW[50] = {0,};
char Client_IP[50] = {0,}, Client_port[50] = {0,}, Multi_Conn[50] = {0,};
char Server_mod[50] = {0,}, Server_Port[50] = {0,}, WifiClose[50] = {0,};
char Hand_OP[50] = {0, }, Index_buf[50], Sensor_OP[50] = {0,}, Realtime_OP[50] = {0,};
uint8_t Sensor_Start = 0, Motor_Start = 0, Realtime_Start = 0, Onetime_Start = 0;

uint8_t EEPROM_STR_buf[5], EEPROM_IMU_buf[40], EEPROM_Force_buf[5];
uint16_t EEPROM_Index;

#define PRJ_24LC256_PAGE_SIZE								(64U)
#define PRJ_24LC256_MAX_MEM_SIZE							(32768U)
#define PRJ_24LC256_MAX_MEM_ADDRESS							(PRJ_24LC256_MAX_MEM_SIZE - 1U)

uint8_t a2h[5] = {0,};

int __io_getchar(void)
{
	unsigned char ch;
	HAL_UART_Receive(&CLI_UART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

int __io_putchar(int ch)
{
     (void) HAL_UART_Transmit(&CLI_UART, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
     return ch;
}

// CLI Buffer initial
void uart_hal_rx_CLI_buffer_init(void)
{
	uart_hal_rx_CLI.cli_input_p = 0;
	uart_hal_rx_CLI.cli_output_p = 0;
	uart_hal_rx_CLI.cli_buf_p = 0;
	uart_hal_rx_CLI.cli_temp = 0;
}

// CLI string get and put buffer
uint8_t uart_hal_CLI_getchar(void)
{
	uint32_t reg = READ_REG(CLI_UART.Instance->CR1);

	__HAL_UART_DISABLE_IT(&CLI_UART, UART_IT_RXNE);
	if(uart_hal_rx_CLI.cli_input_p == uart_hal_rx_CLI.cli_output_p)
	{
		WRITE_REG(CLI_UART.Instance->CR1, reg);
		return 0;
	}

	WRITE_REG(CLI_UART.Instance->CR1, reg);
	uart_hal_rx_CLI.cli_rxd = uart_hal_rx_CLI.cli_buffer[uart_hal_rx_CLI.cli_output_p++];
	if(uart_hal_rx_CLI.cli_rxd != 0x08)
	{
		uart_hal_rx_CLI.cli_buffer_temp[uart_hal_rx_CLI.cli_buf_p] = uart_hal_rx_CLI.cli_rxd;
		uart_hal_rx_CLI.cli_buf_p++;
	}
	if(uart_hal_rx_CLI.cli_output_p >= UART_RX_CLI_BUFFER_SIZE)
	{
		uart_hal_rx_CLI.cli_output_p = 0;
	}

	return 1;
}

// CLI receive data processing
void uart_hal_rx_CLI_monitor(void)
{
	while(uart_hal_CLI_getchar() != 0)
	{
		if(uart_hal_rx_CLI.cli_rxd == '\n')
		{
			uart_hal_rx_CLI.cli_buffer_temp[uart_hal_rx_CLI.cli_buf_p-1] = 0;
			uart_hal_rx_CLI.cli_buffer_temp[uart_hal_rx_CLI.cli_buf_p-2] = 0;
			uart_hal_rx_CLI.cli_buf_p = 0;

			if(menu_cnt == Sub1_menu0)
			{
				if(CWmod_submenu == 1)
				{
					sprintf(CW_Mode, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					if((strcmp(CW_Mode, "q") == 0) || (strcmp(CW_Mode, "Q") == 0))
					{
						menu_cnt = Main_menu1;
						CWmod_submenu = 0;
						Menu_Tree(menu_cnt);
					}
					else if((strcmp(CW_Mode, "1") == 0) || (strcmp(CW_Mode, "3") == 0))
					{
						CWMode_val = ascii_to_hex(CW_Mode);
						CWMode_func(CWMode_val);
						CWmod_submenu = 0;
						menu_cnt = Main_menu1;
					}
					else
					{
						printf("\r\n");
						printf("Please write value again!!\n");
						printf("\r\n");
						CWmod_submenu = 0;
						menu_cnt = Main_menu1;
						Menu_Tree(menu_cnt);
					}
				}
				else if(AP_Conn_submenu == 1)
				{
					sprintf(Wifi_SSID, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					AP_Conn_submenu = 2;
					AP_Sub_Menu_Tree(AP_Conn_submenu);
				}
				else if(AP_Conn_submenu == 2)
				{
					sprintf(Wifi_PW, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					AP_Connection_func(Wifi_SSID, Wifi_PW);
					AP_Conn_submenu = 0;
					menu_cnt = Main_menu1;
				}
				else if(Milti_Conn_submenu == 1)
				{
					sprintf(Multi_Conn, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					if((strcmp(Multi_Conn, "q") == 0) || (strcmp(Multi_Conn, "Q") == 0))
					{
						menu_cnt = Main_menu1;
						Milti_Conn_submenu = 0;
						Menu_Tree(menu_cnt);
					}
					else if((strcmp(Multi_Conn, "0") == 0) || (strcmp(Multi_Conn, "1") == 0))
					{
						Multi_Conn_val = ascii_to_hex(Multi_Conn);
						Multi_connection_mode(Multi_Conn_val);
						Milti_Conn_submenu = 0;
						menu_cnt = Main_menu1;
					}
					else
					{
						printf("\r\n");
						printf("Please write value again!!\n");
						printf("\r\n");
						Milti_Conn_submenu = 0;
						menu_cnt = Main_menu1;
						Menu_Tree(menu_cnt);
					}
				}
				else if(Client_Conn_submenu == 1)
				{
					sprintf(Client_IP, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					Client_Conn_submenu = 2;
					Server_connection_Sub_Menu_Tree(Client_Conn_submenu);
				}
				else if(Client_Conn_submenu == 2)
				{
					sprintf(Client_port, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					WifiClient_Port = ascii_to_hex(Client_port);
					Client_to_Server_Connection(Client_IP, WifiClient_Port);
					Client_Conn_submenu = 0;
					menu_cnt = Main_menu1;
				}
				else if(Server_mod_submenu == 1)
				{
					sprintf(Server_mod, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					if((strcmp(Server_mod, "q") == 0) || (strcmp(Server_mod, "Q") == 0))
					{
						menu_cnt = Main_menu1;
						Server_mod_submenu = 0;
						Menu_Tree(menu_cnt);
					}
					else if((strcmp(Server_mod, "0") == 0) || (strcmp(Server_mod, "1") == 0))
					{
						Server_mod_val = ascii_to_hex(Server_mod);
						if(Server_mod_val == 1) Server_mod_submenu = 2;
						else
						{
							Server_mod_submenu = 0;
							menu_cnt = Main_menu1;
							Server_Mode_Setting(Server_mod_val, Server_port_val);
						}
						Server_mode_Sub_Menu_Tree(Server_mod_submenu);
					}
					else
					{
						printf("\r\n");
						printf("Please write value again!!\n");
						printf("\r\n");
						Server_mod_submenu = 0;
						menu_cnt = Main_menu1;
						Menu_Tree(menu_cnt);
					}
				}
				else if(Server_mod_submenu == 2)
				{
					sprintf(Server_Port, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					Server_port_val = ascii_to_hex(Server_Port);
					menu_cnt = Main_menu1;
					Server_mod_submenu = 0;
					Server_Mode_Setting(Server_mod_val, Server_port_val);
				}
				else if(Close_submenu == 1)
				{
					sprintf(WifiClose, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					if((strcmp(WifiClose, "q") == 0) || (strcmp(WifiClose, "Q") == 0))
					{
						menu_cnt = Main_menu1;
						Close_submenu = 0;
						Menu_Tree(menu_cnt);
					}
					else if((strcmp(WifiClose, "0") == 0) || (strcmp(WifiClose, "1") == 0) ||
							(strcmp(WifiClose, "2") == 0) || (strcmp(WifiClose, "3") == 0) ||
							(strcmp(WifiClose, "4") == 0) || (strcmp(WifiClose, "5") == 0))
					{
						close_val = ascii_to_hex(WifiClose);
						Connection_Close(close_val);
						Close_submenu = 0;
						menu_cnt = Main_menu1;
					}
					else
					{
						printf("\r\n");
						printf("Please write value again!!\n");
						printf("\r\n");
						Close_submenu = 0;
						menu_cnt = Main_menu1;
						Menu_Tree(menu_cnt);
					}
				}
			}
			else if(menu_cnt == Sub2_menu0)
			{
				if(Hand_OP_submenu == 1)
				{
					sprintf(Index_buf, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					Index_val = ascii_to_hex(Index_buf);
					if((strcmp(Index_buf, "q") == 0) || (strcmp(Index_buf, "Q") == 0))
					{
						menu_cnt = Main_menu2;
						Hand_OP_submenu = 0;
						Onetime_Start = 0;
						Sensor_Start = 0;
						Motor_Start = 0;
						Menu_Tree(menu_cnt);
					}
//					else if((Index_val >= 1) && (Index_val <= 649))
					else if((Index_val >= 1) && (Index_val <= 620))
					{
						Hand_OP_EEPROM_Index_Read(Index_val);
						Motor_Start = 1;
						Hand_OP_submenu = 1;
						Index_Clear_Sub_Menu_Tree();
					}
					else
					{
						printf("retry to write Index number!!\n");
					}
				}
				else if(Index_Read_submenu == 1)
				{
					sprintf(Index_buf, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					Index_val = ascii_to_hex(Index_buf);
					if((strcmp(Index_buf, "q") == 0) || (strcmp(Index_buf, "Q") == 0))
					{
						menu_cnt = Main_menu2;
						Index_Read_submenu = 0;
						Menu_Tree(menu_cnt);
					}
					else if((Index_val >= 1) && (Index_val <= 620))
					{

						EEPROM_Index_Read(Index_val);
						Index_Read_submenu = 1;
						Index_Clear_Sub_Menu_Tree();
					}
					else
					{
						printf("retry to write Index number!!\n");
					}
				}
				else if(Index_clear_submenu == 1)
				{
					sprintf(Index_buf, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					Index_val = ascii_to_hex(Index_buf);
					if((strcmp(Index_buf, "q") == 0) || (strcmp(Index_buf, "Q") == 0))
					{
						menu_cnt = Main_menu2;
						Index_clear_submenu = 0;
						Menu_Tree(menu_cnt);
					}
					else if((Index_val >= 1) && (Index_val <= 620))
					{
						EEPROM_Index_Clear(Index_val);
						Index_clear_submenu = 1;
						Index_Clear_Sub_Menu_Tree();
					}
					else
					{
						printf("retry to write Index number!!\n");
					}
				}
				else if(Sensor_OP_submenu == 1)
				{
					sprintf(Sensor_OP, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					if((strcmp(Sensor_OP, "0") == 0) || (strcmp(Sensor_OP, "1") == 0))
					{
						if(Sensor_OP[0] == '1')
						{
							Sensor_Start = 1;
						}
						else
						{
							Sensor_Start = 0;
						}
						Sensor_OP_submenu = 0;
						menu_cnt = Main_menu2;
						Menu_Tree(menu_cnt);
					}
					else
					{
						printf("\r\n");
						printf("Please write value again!!\n");
						printf("\r\n");
						Sensor_OP_submenu = 0;
						menu_cnt = Main_menu2;
						Menu_Tree(menu_cnt);
					}
				}
				else if(Realtime_OP_submenu == 1)
				{
					sprintf(Realtime_OP, "%s", uart_hal_rx_CLI.cli_buffer_temp);
					if((strcmp(Realtime_OP, "q") == 0) || (strcmp(Realtime_OP, "Q") == 0))
					{
						menu_cnt = Main_menu2;
						memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
						Realtime_OP_submenu = 0;
						Menu_Tree(menu_cnt);
					}
					else if((strcmp(Realtime_OP, "0") == 0) || (strcmp(Realtime_OP, "1") == 0))
					{
						memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
						if(strcmp(Realtime_OP, "0") == 0)
						{
							Realtime_Start = 0;
							Sensor_Start = 0;
							Realtime_flag = 0;
						}
						else
						{
							Realtime_Start= 1;
							Sensor_Start= 1;
						}
						Realtime_OP_submenu = 0;
						menu_cnt = Main_menu2;
						Menu_Tree(menu_cnt);
					}
					else
					{
						printf("\r\n");
						printf("Please write value again!!\n");
						printf("\r\n");
						Realtime_OP_submenu = 0;
						menu_cnt = Main_menu2;
						Menu_Tree(menu_cnt);
					}
				}
			}
			else
			{
				if((strcmp(uart_hal_rx_CLI.cli_buffer_temp, "q") == 0) || (strcmp(uart_hal_rx_CLI.cli_buffer_temp, "Q") == 0))
				{
					switch(menu_cnt)
					{
						case Main_menu1 :
							menu_cnt = Main_menu0;
							CWmod_submenu = 0;
							AP_Conn_submenu = 0;
							Milti_Conn_submenu = 0;
							Client_Conn_submenu = 0;
							Server_mod_submenu = 0;
							Close_submenu = 0;
							break;
						case Main_menu2 :
							menu_cnt = Main_menu0;
							Hand_OP_submenu = 0;
							Index_Read_submenu = 0;
							Index_clear_submenu = 0;
							Sensor_OP_submenu = 0;
							Realtime_OP_submenu = 0;
							break;
						case Sub1_menu0 :
							menu_cnt = Main_menu1;
							memset(uart_hal_rx_CLI.cli_buffer_temp, 0, sizeof(uart_hal_rx_CLI.cli_buffer_temp));
							CWmod_submenu = 0;
							AP_Conn_submenu = 0;
							Milti_Conn_submenu = 0;
							Client_Conn_submenu = 0;
							Server_mod_submenu = 0;
							Close_submenu = 0;
							break;
						case Sub2_menu0 :
							menu_cnt = Main_menu2;
							Motor_Start = 0;
							Sensor_Start = 0;
							Onetime_Start = 0;
							Realtime_Start = 0;
							Index_clear_submenu = 0;
							break;
						default : break;
					}
					Menu_Tree(menu_cnt);
					uart_hal_rx_CLI.cli_buf_p = 0;
				}
				else if(strcasecmp(uart_hal_rx_CLI.cli_buffer_temp, "1") == 0)
				{
					switch(menu_cnt)
					{
						case Main_menu0 :
							menu_cnt = Main_menu1;
							Menu_Tree(menu_cnt);
							break;
						case Main_menu1 :
							AT_Test();
							break;
						case Main_menu2 :
							Hand_Name_list();
							Menu_Tree(menu_cnt);
							break;
						case Sub1_menu0 :
							break;
						case Sub2_menu0 :
							break;
						default : break;
					}
				}
				else if(strcasecmp(uart_hal_rx_CLI.cli_buffer_temp, "2") == 0)
				{
					switch(menu_cnt)
					{
						case Main_menu0 :
							menu_cnt = Main_menu2;
							Menu_Tree(menu_cnt);
							break;
						case Main_menu1 :
							Wifi_Reset_func();
							break;
						case Main_menu2 :
							menu_cnt = Sub2_menu0;
							Hand_OP_submenu = 1;
							Sensor_Start = 1;
							Onetime_Start = 1;
							Index_Clear_Sub_Menu_Tree();
							break;
						case Sub1_menu0 :
							break;
						case Sub2_menu0 :
							break;
						default : break;
					}
				}
				else if(strcasecmp(uart_hal_rx_CLI.cli_buffer_temp, "3") == 0)
				{
					switch(menu_cnt)
					{
						case Main_menu0 :
							break;
						case Main_menu1 :
							menu_cnt = Sub1_menu0;
							CWmod_submenu = 1;
							CWMode_Sub_Menu_Tree();
							break;
						case Main_menu2 :
//							Run_Hand_Calibration();
							Menu_Tree(menu_cnt);
							break;
						case Sub1_menu0 :
							break;
						case Sub2_menu0 :
							break;
						default : break;
					}
				}
				else if(strcasecmp(uart_hal_rx_CLI.cli_buffer_temp, "4") == 0)
				{
					switch(menu_cnt)
					{
						case Main_menu0 :
							break;
						case Main_menu1 :
							menu_cnt = Sub1_menu0;
							AP_Conn_submenu = 1;
							AP_Sub_Menu_Tree(AP_Conn_submenu);
							break;
						case Main_menu2 :
							menu_cnt = Sub2_menu0;
							Index_Read_submenu = 1;
							Index_Clear_Sub_Menu_Tree();
							break;
						case Sub1_menu0 :
							break;
						case Sub2_menu0 :
							break;
						default : break;
					}
				}
				else if(strcasecmp(uart_hal_rx_CLI.cli_buffer_temp, "5") == 0)
				{
					switch(menu_cnt)
					{
						case Main_menu0 :
							break;
						case Main_menu1 :
							Wifi_Get_IP();
							break;
						case Main_menu2 :
							menu_cnt = Sub2_menu0;
							Index_clear_submenu = 1;
							Index_Clear_Sub_Menu_Tree();
							break;
						case Sub1_menu0 :
							break;
						case Sub2_menu0 :
							break;
						default : break;
					}
				}
				else if(strcasecmp(uart_hal_rx_CLI.cli_buffer_temp, "6") == 0)
				{
					switch(menu_cnt)
					{
						case Main_menu0 :
							break;
						case Main_menu1 :
							menu_cnt = Sub1_menu0;
							Milti_Conn_submenu = 1;
							Multi_Sub_Menu_Tree();
							break;
						case Main_menu2 :
							EEPROM_ALL_Clear();
							Menu_Tree(menu_cnt);
							break;
						case Sub1_menu0 :
							break;
						case Sub2_menu0 :
							break;
						default : break;
					}
				}
				else if(strcasecmp(uart_hal_rx_CLI.cli_buffer_temp, "7") == 0)
				{
					switch(menu_cnt)
					{
						case Main_menu0 :
							break;
						case Main_menu1 :
							menu_cnt = Sub1_menu0;
							Client_Conn_submenu = 1;
							Server_connection_Sub_Menu_Tree(Client_Conn_submenu);
							break;
						case Main_menu2 :
							EEPROM_Read();
							Menu_Tree(menu_cnt);
							break;
						case Sub1_menu0 :
							break;
						case Sub2_menu0 :
							break;
						default : break;
					}
				}
				else if(strcasecmp(uart_hal_rx_CLI.cli_buffer_temp, "8") == 0)
				{
					switch(menu_cnt)
					{
						case Main_menu0 :
							break;
						case Main_menu1 :
							menu_cnt = Sub1_menu0;
							Server_mod_submenu = 1;
							Server_mode_Sub_Menu_Tree(Server_mod_submenu);
							break;
						case Main_menu2 :
							menu_cnt = Sub2_menu0;
							Sensor_OP_submenu = 1;
							Sensor_OP_Sub_Menu_Tree();
							break;
						case Sub1_menu0 :
							break;
						case Sub2_menu0 :
							break;
						default : break;
					}
				}
				else if(strcasecmp(uart_hal_rx_CLI.cli_buffer_temp, "9") == 0)
				{
					switch(menu_cnt)
					{
						case Main_menu0 :
							break;
						case Main_menu1 :
							menu_cnt = Sub1_menu0;
							Close_submenu = 1;
							Server_Close_Sub_Menu_Tree();
							break;
						case Main_menu2 :
							menu_cnt = Sub2_menu0;
							Realtime_OP_submenu = 1;
							Sensor_OP_Sub_Menu_Tree();
							break;
						case Sub1_menu0 :
							break;
						case Sub2_menu0 :
							break;
						default : break;
					}
				}
				else
				{
					Menu_Tree(menu_cnt);
				}
			}
		}
		else if(uart_hal_rx_CLI.cli_rxd == 0x08)
		{
			if(uart_hal_rx_CLI.cli_buf_p > 0)
			{
				uart_hal_rx_CLI.cli_buf_p--;
				uart_hal_rx_CLI.cli_buffer_temp[uart_hal_rx_CLI.cli_buf_p] = 0;
			}
		}
	}
}

uint8_t AT_Flag = 0, AT_RST_Flag = 0, AT_MODE_Flag = 0, AT_AP_Conn_Flag = 0, Get_ip_flag = 0, AT_MUX_Flag = 0, AT_Conn_Flag = 0, AT_Server_mod_Flag = 0, AT_Close_Flag = 0;
uint8_t AT_ECHO_Flag = 0, AT_SensorOP_Flag = 0;

// ESP-01 AT Command check operation
void AT_Test(void)
{
	uint8_t AT[] = "AT\r\n";
	HAL_UART_Transmit(&WiFi_UART, AT, sizeof(AT), HAL_MAX_DELAY);
	AT_Flag = 1;
}

// ESP-01 S/W Reset
void Wifi_Reset_func(void)
{
	uint8_t AT_RST[] = "AT+RST\r\n";
	HAL_UART_Transmit(&WiFi_UART, AT_RST, sizeof(AT_RST), HAL_MAX_DELAY);
	AT_RST_Flag = 1;
}

// CWMode Setting(AP/STA)
void CWMode_func(uint8_t mode)
{
	char data[80] = {0,};
	uint8_t n;
	n = sprintf(data, "AT+CWMODE=%d\r\n", mode);
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	AT_MODE_Flag = 1;
}

// AP SSID / Password Setting
void AP_Connection_func(char *SSID, char *PASSWD)
{
	char data[80] = {0,};
	uint8_t n;
	n = sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	AT_AP_Conn_Flag = 1;
}

// ESP-01 check IP
void Wifi_Get_IP(void)
{
	uint8_t AT_CIFSR[] = "AT+CIFSR\r\n";
	HAL_UART_Transmit(&WiFi_UART, AT_CIFSR, sizeof(AT_CIFSR), HAL_MAX_DELAY);
	Get_ip_flag = 1;
}

// Mux Setting(Client/Server)
void Multi_connection_mode(uint8_t mux)
{
	char data[80] = {0,};
	uint8_t n;
	n = sprintf(data, "AT+CIPMUX=%d\r\n", mux);
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	AT_MUX_Flag = 1;
}

// Connection Client to Server
void Client_to_Server_Connection(char *ip, uint16_t port)
{
	char data[80] = {0,};
	uint8_t n;
	n = sprintf (data, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ip, port);
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	AT_Conn_Flag = 1;
}

// if Mux is Server, Server start
void Server_Mode_Setting(uint8_t mode, uint16_t port)
{
	char data[80] = {0,};
	uint8_t n;
	if(mode == 0)
	{
		n = sprintf (data, "AT+CIPSERVER=%d\r\n", mode);
		HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	}
	else
	{
		n = sprintf (data, "AT+CIPSERVER=%d,%d\r\n", mode, port);
		HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	}
	AT_Server_mod_Flag = 1;
}

// Server and Client connection close
void Connection_Close(uint8_t LinkID)
{
	char data[80] = {0,};
	uint8_t n;
	n = sprintf (data, "AT+CIPCLOSE=%d\r\n", LinkID);
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	AT_Close_Flag = 1;
}

// ESP-01 read status
void WifiStatus(void)
{
	char data[80] = {0,};
	uint8_t n;
	n = sprintf (data, "AT+CIPSTATUS\r\n");
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
}

// find start address of EEPROM
uint16_t Start_Index_Addr_cal(uint16_t Index)
{
	uint16_t Index_start_addr;

	Index_start_addr = ((Index*Index_jump)+Sensor_Val0);

	return Index_start_addr;
}


// Generate data to check connected to Hand
void Ready_Send_Echo(void)
{
	char data[80] = {0,};
	uint8_t n, linkID = 0;
	uint16_t len = 33;
	n = sprintf(data, "AT+CIPSEND=%d,%d\r\n", linkID, len);
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	AT_ECHO_Flag = 1;
}

// Send data to check connected to Hand
void ECHO(void)
{
	uint8_t Echo_Connect[] = "Connection with the Robot hand!\r\n";
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)Echo_Connect, sizeof(Echo_Connect), HAL_MAX_DELAY);
}

// Not use
//void Ready_Send_Sensor_OP(void)
//{
//	char data[80] = {0,};
//	uint8_t n, linkID = 0;
//	uint16_t len = 24;
//	n = sprintf(data, "AT+CIPSEND=%d,%d\r\n", linkID, len);
//	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
//	AT_SensorOP_Flag = 1;
//}

// Send Sensor operation response
void Hand_Sensor_OP(void)
{
	uint8_t Sensor_Operation[] = "Hand Sensor Operation!\r\n";
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)Sensor_Operation, sizeof(Sensor_Operation), HAL_MAX_DELAY);
}

// Show saved name list in EEPROM
void Hand_Name_list(void)
{
	uint16_t i, Index_start_addr;
	uint8_t buffer[5];

	printf("\r\n");

	for(i=1; i<=620; i++)
	{
		Index_start_addr = ((i*Index_jump)+Sensor_Val0);

		EEPROM_Readbytes(Index_start_addr+2, 5, buffer);
		HAL_Delay(10);
		printf("Index %d : %c%c%c%c%c\n", i, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	}

	printf("\r\n");
}

uint8_t imu0_roll[4], imu0_pitch[4], imu01_roll[4], imu1_pitch[4], imu2_roll[4], imu2_pitch[4], imu3_roll[4], imu3_pitch[4], imu4_roll[4], imu4_pitch[4];

// Read saved data packet in EEPROM for running motor
void Hand_OP_EEPROM_Index_Read(uint16_t Index)
{
	uint8_t m_page_buffer[52] = {0}, buffer, Clear_val = 0x00, Index_buf[2];
	uint8_t EEP_Roll[4], EEP_Pitch[4];
	uint16_t Index_start_addr, i;
//	uint8_t imu0_roll[4], imu0_pitch[4], imu01_roll[4], imu1_pitch[4], imu2_roll[4], imu2_pitch[4], imu3_roll[4], imu3_pitch[4], imu4_roll[4], imu4_pitch[4];

	memset(m_page_buffer, Clear_val, sizeof(m_page_buffer));

	Index_start_addr = ((Index*Index_jump)+Sensor_Val0);

	printf("\r\n");

	for(i=0; i<52; i++)
	{
		EEPROM_Readbyte(Index_start_addr+i, &buffer);
		HAL_Delay(5);

		if(i < 2)
		{
			Index_buf[i] = buffer;
		}
		else if((i >= 2) && (i <= 6))
		{
			EEPROM_STR_buf[i-2] = buffer;
		}
		else if((i >= 7) && (i <= 46))
		{
			EEPROM_IMU_buf[i-7] = buffer;
		}
		else
		{
			EEPROM_Force_buf[i] = buffer;
		}

		printf("0x%02x  ", buffer);
		if(i == 1)					//Index
		{
			printf("\r\n");
		}
		else if(i == 6)				//STR
		{
			printf("\r\n");
		}
		else if(i == 14)			//Roll[0], Pitch[0]
		{
			printf("\r\n");
		}
		else if(i == 22)			//Roll[1], Pitch[1]
		{
			printf("\r\n");
		}
		else if(i == 30)			//Roll[2], Pitch[2]
		{
			printf("\r\n");
		}
		else if(i == 38)			//Roll[3], Pitch[3]
		{
			printf("\r\n");
		}
		else if(i == 46)			//Roll[4], Pitch[4]
		{
			printf("\r\n");
		}
		else if(i ==51)
		{
			printf("\r\n");			//Force0 ~ 4
		}

	}

	EEPROM_Index = (uint16_t)((Index_buf[0] << 8) + Index_buf[1]);

	for(i=7; i<11; i++) EEP_Roll[i-7] = EEPROM_IMU_buf[i-7];
	byte2float_onedata(&Roll0_deg, EEP_Roll);
	Roll_buf[0] = trunc(Roll0_deg);
	for(i=11; i<15; i++) EEP_Pitch[i-11] = EEPROM_IMU_buf[i-7];
	byte2float_onedata(&Pitch0_deg, EEP_Pitch);
	Pitch_buf[0] = trunc(Pitch0_deg);
	for(i=15; i<19; i++) EEP_Roll[i-15] = EEPROM_IMU_buf[i-7];
	byte2float_onedata(&Roll1_deg, EEP_Roll);
	Roll_buf[1] = trunc(Roll1_deg);
	for(i=19; i<23; i++) EEP_Pitch[i-19] = EEPROM_IMU_buf[i-7];
	byte2float_onedata(&Pitch1_deg, EEP_Pitch);
	Pitch_buf[1] = trunc(Pitch1_deg);
	for(i=23; i<27; i++) EEP_Roll[i-23] = EEPROM_IMU_buf[i-7];
	byte2float_onedata(&Roll2_deg, EEP_Roll);
	Roll_buf[2] = trunc(Roll2_deg);
	for(i=27; i<31; i++) EEP_Pitch[i-27] = EEPROM_IMU_buf[i-7];
	byte2float_onedata(&Pitch2_deg, EEP_Pitch);
	Pitch_buf[2] = trunc(Pitch2_deg);
	for(i=31; i<35; i++) EEP_Roll[i-31] = EEPROM_IMU_buf[i-7];
	byte2float_onedata(&Roll3_deg, EEP_Roll);
	Roll_buf[3] = trunc(Roll3_deg);
	for(i=35; i<39; i++) EEP_Pitch[i-35] = EEPROM_IMU_buf[i-7];
	byte2float_onedata(&Pitch3_deg, EEP_Pitch);
	Pitch_buf[3] = trunc(Pitch3_deg);
	for(i=39; i<43; i++) EEP_Roll[i-39] = EEPROM_IMU_buf[i-7];
	byte2float_onedata(&Roll4_deg, EEP_Roll);
	Roll_buf[4] = trunc(Roll4_deg);
	for(i=43; i<47; i++) EEP_Pitch[i-43] = EEPROM_IMU_buf[i-7];
	byte2float_onedata(&Pitch4_deg, EEP_Pitch);
	Pitch_buf[4] = trunc(Pitch4_deg);
}

// IMU Calibration(Not use)
//void Run_Hand_Calibration(void)
//{
//	printf("\r\n");
//	printf("START Calibration\n");
//	printf("\r\n");
////	ALL_IMU_Calibration_Save();
//	printf("\r\n");
//	printf("END Calibration\n");
//	printf("\r\n");
//}

// Erase all data in EEPROM
void EEPROM_ALL_Clear(void)
{
	uint16_t i;
	uint8_t Wbuffer[PRJ_24LC256_PAGE_SIZE] = {0,};
	uint8_t Clear_val = 0x00;

	memset(Wbuffer, Clear_val, PRJ_24LC256_PAGE_SIZE);

	printf("\r\n");
	printf("Cleaning\n");

	for(i=0; i<PRJ_24LC256_MAX_MEM_SIZE/PRJ_24LC256_PAGE_SIZE; i++)
	{
		EEPROM_Writebytes(i*PRJ_24LC256_PAGE_SIZE, PRJ_24LC256_PAGE_SIZE, Wbuffer);
		HAL_Delay(10);
	}
	printf("\r\n");
	printf("ALL Data Clear\n");
	printf("\r\n");
}

// Read saved data in the selected index
void EEPROM_Index_Read(uint16_t Index)
{
	uint8_t m_page_buffer[52] = {0}, buffer, Clear_val = 0x00;
	uint16_t Index_start_addr, i;

	memset(m_page_buffer, Clear_val, sizeof(m_page_buffer));

	Index_start_addr = ((Index*Index_jump)+Sensor_Val0);

	printf("\r\n");

	for(i=0; i<52; i++)
	{
		EEPROM_Readbyte(Index_start_addr+i, &buffer);
		HAL_Delay(3);
		printf("0x%02x  ", buffer);
		if(i == 1)					//Index
		{
			printf("\r\n");
		}
		else if(i == 6)				//STR
		{
			printf("\r\n");
		}
		else if(i == 14)			//Roll[0], Pitch[0]
		{
			printf("\r\n");
		}
		else if(i == 22)			//Roll[1], Pitch[1]
		{
			printf("\r\n");
		}
		else if(i == 30)			//Roll[2], Pitch[2]
		{
			printf("\r\n");
		}
		else if(i == 38)			//Roll[3], Pitch[3]
		{
			printf("\r\n");
		}
		else if(i == 46)			//Roll[4], Pitch[4]
		{
			printf("\r\n");
		}
		else if(i ==51)
		{
			printf("\r\n");			//Force0 ~ 4
		}
	}
}

// Erase Index data in EEPROM
void EEPROM_Index_Clear(uint16_t Index)
{
	uint8_t m_page_buffer[52] = {0}, buffer, Clear_val = 0x00;
	uint16_t Index_start_addr, i;

	memset(m_page_buffer, Clear_val, sizeof(m_page_buffer));

	Index_start_addr = ((Index*Index_jump)+Sensor_Val0);

	for(i=0; i<52; i++)
	{
		EEPROM_Writebyte(Index_start_addr+i, m_page_buffer[i]);
		HAL_Delay(10);
	}

	printf("\r\n");

	for(i=0; i<52; i++)
	{
		EEPROM_Readbyte(Index_start_addr+i, &buffer);
		HAL_Delay(3);
		printf("0x%02x  ", buffer);
		if(i == 1)					//Index
		{
			printf("\r\n");
		}
		else if(i == 6)				//STR
		{
			printf("\r\n");
		}
		else if(i == 14)			//Roll[0], Pitch[0]
		{
			printf("\r\n");
		}
		else if(i == 22)			//Roll[1], Pitch[1]
		{
			printf("\r\n");
		}
		else if(i == 30)			//Roll[2], Pitch[2]
		{
			printf("\r\n");
		}
		else if(i == 38)			//Roll[3], Pitch[3]
		{
			printf("\r\n");
		}
		else if(i == 46)			//Roll[4], Pitch[4]
		{
			printf("\r\n");
		}
		else if(i ==51)
		{
			printf("\r\n");			//Force0 ~ 4
		}
	}
}

// Read all data in EEPROM
void EEPROM_Read(void)
{
	uint16_t i, j;
	uint8_t Rbuffer[PRJ_24LC256_PAGE_SIZE] = {0,};
	printf("\r\n");

	for(i=0; i<PRJ_24LC256_MAX_MEM_SIZE/PRJ_24LC256_PAGE_SIZE; i++)
	{
		EEPROM_Readbytes(i*PRJ_24LC256_PAGE_SIZE, PRJ_24LC256_PAGE_SIZE, Rbuffer);
		HAL_Delay(3);
		for(j=0; j<PRJ_24LC256_PAGE_SIZE; j++)
		{
			printf("0x%02x  ", Rbuffer[j]);
			if(j%16 == 15) printf("\r\n");
		}
	}
}

// generate response data receiving data from glove(onetime)
void Ready_Onetime_rx_Send(void)
{
	char data[80] = {0,};
	uint8_t n, linkID = 0;
	uint16_t len = 24;
	n = sprintf(data, "AT+CIPSEND=%d,%d\r\n", linkID, len);
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	HAL_Delay(20);
}

// send response data receiving data from glove(onetime)
void Onetime_rx_Send(void)
{
	uint8_t Success[] = "Success one time data!\r\n";
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)Success, sizeof(Success), HAL_MAX_DELAY);
}

// generate error response data receiving data from glove(onetime)
void Ready_Onetime_Error_Send(void)
{
	char data[80] = {0,};
	uint8_t n, linkID = 0;
	uint16_t len = 22;
	n = sprintf(data, "AT+CIPSEND=%d,%d\r\n", linkID, len);
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	HAL_Delay(20);
}

// send error response data receiving data from glove(onetime)
void Onetime_Error_Send(void)
{
	uint8_t Success[] = "Error one time data!\r\n";
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)Success, sizeof(Success), HAL_MAX_DELAY);
}

// generate response data receiving data from glove(realtime)
void Ready_Realtime_rx_Send(void)
{
	char data[80] = {0,};
	uint8_t n, linkID = 0;
	uint16_t len = 25;
	n = sprintf(data, "AT+CIPSEND=%d,%d\r\n", linkID, len);
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)data, n, HAL_MAX_DELAY);
	HAL_Delay(20);
}

// send response data receiving data from glove(realtime)
void Realtime_rx_Send(void)
{
	uint8_t Success[] = "Success Real time data!\r\n";
	HAL_UART_Transmit(&WiFi_UART, (uint8_t*)Success, sizeof(Success), HAL_MAX_DELAY);
}

// show main Menu
void UART_Menu(void)
{
	printf("\r\n");
	printf("*****************************************************\n");
	printf("\r\n");
	printf("1. Wifi -------------------------------------> \r\n");
	printf("\r\n");
	printf("2. KC HAND ----------------------------------> \r\n");
	printf("\r\n");
	printf("*****************************************************\n");
	printf("\r\n");
}

// show submenu in menu 1
void UART_Menu1(void)
{
	printf("\r\n");
	printf("*****************************************************\n");
	printf("\r\n");
	printf("1. AT Test                                     \r\n");
	printf("\r\n");
	printf("2. Wifi Reset                                  \r\n");
	printf("\r\n");
	printf("3. CW Mode ----------------------------------> \r\n");
	printf("\r\n");
	printf("4. AP Connection ----------------------------> \r\n");
	printf("\r\n");
	printf("5. Wifi get IP                                 \r\n");
	printf("\r\n");
	printf("6. Multi Connection mode(Mux) ---------------> \r\n");
	printf("\r\n");
	printf("7. Client->Server Connection(Only Client) ---> \r\n");
	printf("\r\n");
	printf("8. Server mode setting(Only Server) ---------> \r\n");
	printf("\r\n");
	printf("9. Connection Close(Only Client) ------------> \r\n");
	printf("\r\n");
	printf("*****************************************************\n");
	printf("\r\n");
}

// show submenu in menu 2
void UART_Menu2(void)
{
	printf("\r\n");
	printf("*****************************************************\n");
	printf("\r\n");
	printf("1. Read Index name list ---------------------> \r\n");
	printf("\r\n");
	printf("2. Run Hand ---------------------------------> \r\n");
	printf("\r\n");
	printf("3. Run Hand Calibration                        \r\n");
	printf("\r\n");
	printf("4. EEPROM Index Read ------------------------> \r\n");
	printf("\r\n");
	printf("5. EEPROM Index Clear -----------------------> \r\n");
	printf("\r\n");
	printf("6. EEPROM All Clear                            \r\n");
	printf("\r\n");
	printf("7. EEPROM All Read                             \r\n");
	printf("\r\n");
	printf("8. Sensor ON/OFF ----------------------------> \r\n");
	printf("\r\n");
	printf("9. Run Real time ----------------------------> \r\n");
	printf("\r\n");
	printf("*****************************************************\n");
	printf("\r\n");
}

// show selected menu
void Menu_Tree(uint8_t menu)
{
	switch(menu)
	{
		case Main_menu0 : UART_Menu(); 	break;
		case Main_menu1 : UART_Menu1(); break;
		case Main_menu2 : UART_Menu2(); break;
		default : 				break;
	}
	uint8_t CMD_str[] = "CMD>";
	HAL_UART_Transmit(&CLI_UART, (uint8_t*) CMD_str, sizeof(CMD_str), HAL_MAX_DELAY);
}

// Show Selectable values in CWmode
void CWMode_Sub_Menu_Tree(void)
{
	uint8_t CWMode_str[] = "Value(q / Q : Quit or 1:AP, 3:Wifi-Direct):";

	printf("\r\n");
	HAL_UART_Transmit(&CLI_UART, (uint8_t*) CWMode_str, sizeof(CWMode_str), HAL_MAX_DELAY);
}

// Show menu to set up AP Connection
void AP_Sub_Menu_Tree(uint8_t sub_menu)
{
	uint8_t SSID_str[] = "SSID:";
	uint8_t PW_str[] = "PW:";

	printf("\r\n");

	switch(sub_menu)
	{
		case 0 : break;
		case 1 :
			HAL_UART_Transmit(&CLI_UART, (uint8_t*) SSID_str, sizeof(SSID_str), HAL_MAX_DELAY);
			break;
		case 2 :
			HAL_UART_Transmit(&CLI_UART, (uint8_t*) PW_str, sizeof(PW_str), HAL_MAX_DELAY);
			break;
		default : break;
	}
}

// Show menu to connect from client to server
void Server_connection_Sub_Menu_Tree(uint8_t sub_menu)
{
	uint8_t Mode_str[] = "Server IP:";
	uint8_t Port_str[] = "Server Port:";

	printf("\r\n");

	switch(sub_menu)
	{
		case 0 : break;
		case 1 :
			HAL_UART_Transmit(&CLI_UART, (uint8_t*) Mode_str, sizeof(Mode_str), HAL_MAX_DELAY);
			break;
		case 2 :
			HAL_UART_Transmit(&CLI_UART, (uint8_t*) Port_str, sizeof(Port_str), HAL_MAX_DELAY);
			break;
		default : break;
	}
}

// Show menu to start and stop server and port
void Server_mode_Sub_Menu_Tree(uint8_t sub_menu)
{
	uint8_t IP_str[] = "Server mode(q / Q : Quit or 0:Off, 1:Start):";
	uint8_t Port_str[] = "Server Port:";

	printf("\r\n");
	switch(sub_menu)
	{
		case 0 : break;
		case 1 :
			HAL_UART_Transmit(&CLI_UART, (uint8_t*) IP_str, sizeof(IP_str), HAL_MAX_DELAY);
			break;
		case 2 :
			HAL_UART_Transmit(&CLI_UART, (uint8_t*) Port_str, sizeof(Port_str), HAL_MAX_DELAY);
			break;
		default : break;
	}
}

// Show server/client selection menu
void Multi_Sub_Menu_Tree(void)
{
	uint8_t Milti_str[] = "Value(q / Q : Quit or 0:Client, 1:Server):";

	printf("\r\n");
	HAL_UART_Transmit(&CLI_UART, (uint8_t*) Milti_str, sizeof(Milti_str), HAL_MAX_DELAY);
}

// Show menu to disconnect
void Server_Close_Sub_Menu_Tree(void)
{
	uint8_t Close_str[] = "Value(q / Q : Quit or 0~4(LinkID), 5(All Close):";

	printf("\r\n");
	HAL_UART_Transmit(&CLI_UART, (uint8_t*) Close_str, sizeof(Close_str), HAL_MAX_DELAY);
}

// Show the index range to be able to erase data
void Index_Clear_Sub_Menu_Tree(void)
{
	uint8_t Index_str[] = "Index(q / Q : Quit or 1~620):";
	printf("\r\n");
	HAL_UART_Transmit(&CLI_UART, (uint8_t*) Index_str, sizeof(Index_str), HAL_MAX_DELAY);
}

// Show sensor ON/OFF menu
void Sensor_OP_Sub_Menu_Tree(void)
{
	uint8_t SensorOP_str[] = "Index(q / Q : Quit or 0:OFF/1:ON):";
	printf("\r\n");
	HAL_UART_Transmit(&CLI_UART, (uint8_t*) SensorOP_str, sizeof(SensorOP_str), HAL_MAX_DELAY);
}

// Convert one data from ascii to heximal
uint8_t ascii2hex(char asc)
{
	uint8_t hex;
	if( asc >= '0' && asc <= '9')
	{
		hex = asc - 0x30;
	}
	else if( asc >= 'A' && asc <= 'Z' )
	{
		hex = asc - 0x37;
		return 0;
	}
	else
	{
		hex = asc - 0x57;
		return 0;
	}
	return hex;
}

// Convert array data from ascii to decimal
unsigned short ascii_to_hex(char* buf)
{
	unsigned short tenthousand=0, thousand=0, hundred=0, ten=0, unit=0, value=0;
	int len = strlen(buf);
	uint8_t temp[5];

	switch(len)
	{
		case 1 :
			temp[0] = ascii2hex(buf[0]);
			unit = temp[0];
			value = unit;
			break;
		case 2 :
			temp[0] = ascii2hex(buf[0]);
			ten = temp[0]*10;
			temp[1] = ascii2hex(buf[1]);
			unit = temp[1];
			value = (ten + unit);
			break;
		case 3 :
			temp[0] = ascii2hex(buf[0]);
			hundred = temp[0]*100;
			temp[1] = ascii2hex(buf[1]);
			ten = temp[1]*10;
			temp[2] = ascii2hex(buf[2]);
			unit = temp[2];
			value = (hundred + ten + unit);
			break;
		case 4 :
			temp[0] = ascii2hex(buf[0]);
			thousand = temp[0]*1000;
			temp[1] = ascii2hex(buf[1]);
			hundred = temp[1]*100;
			temp[2] = ascii2hex(buf[2]);
			ten = temp[2]*10;
			temp[3] = ascii2hex(buf[3]);
			unit = temp[3];
			value = (thousand + hundred + ten + unit);
			break;
		case 5 :
			temp[0] = ascii2hex(buf[0]);
			tenthousand = temp[0]*10000;
			temp[1] = ascii2hex(buf[1]);
			thousand = temp[1]*1000;
			temp[2] = ascii2hex(buf[2]);
			hundred = temp[2]*100;
			temp[3] = ascii2hex(buf[3]);
			ten = temp[3]*10;
			temp[4] = ascii2hex(buf[4]);
			unit = temp[4];
			value = (tenthousand + thousand + hundred + ten + unit);
			break;
		default :
			break;
	}

	return value;
}

// not use
//void str2hex(char* str)
//{
//	char asc;
//	uint8_t hex, i;
//	int len;
//	len = strlen (str);
//
//	for(i=0; i<len; i++)
//	{
//		asc = str[i];
//		if( asc >= '0' && asc <= '9')
//		{
//			hex = asc - 0x30;
//		}
//		else if( asc >= 'A' && asc <= 'Z' )
//		{
//			hex = asc - 0x37;
//		}
//		else
//		{
//			hex = asc - 0x57;
//		}
//		a2h[i] = hex;
//	}
//}

// not use
//void hex2str(char* hex)
//{
//	uint8_t i, size;
//	char asc;
//
//	size = sizeof(hex);
//
//	for(i=0; i<size+1; i++)
//	{
//		if(hex[i] > 0x09)
//		{
//			asc = hex[i] + 0x57;
//		}
//		else
//		{
//			asc = hex[i] + 0x30;
//		}
//		printf("%c\n", asc);
//	}
//}

// change value range
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// not use
//float mapArduino(float val, float I_Min, float I_Max, float O_Min, float O_Max)
//{
//    return (((val - I_Min) * ((O_Max - O_Min) / (I_Max - I_Min))) + O_Min);
//}

