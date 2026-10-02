#ifndef INC_CLI_H_
#define INC_CLI_H_

#include "main.h"

#define CLI_UART						huart6
#define UART_RX_CLI_BUFFER_SIZE			50

typedef enum main_menu_e
{
    Main_menu0 = 0,
	Main_menu1,
	Sub1_menu0,
	Main_menu2,
	Sub2_menu0,
} main_menu_t;

extern main_menu_t main_menu;

typedef struct
{
	char cli_buffer[UART_RX_CLI_BUFFER_SIZE];
	char cli_buffer_temp[UART_RX_CLI_BUFFER_SIZE];
	uint8_t cli_rxd;
	volatile uint16_t cli_input_p;
	volatile uint16_t cli_output_p;
	volatile uint16_t cli_buf_p;
	uint8_t cli_temp;
} uart_rx_CLI;

extern uart_rx_CLI uart_hal_rx_CLI;

int __io_getchar(void);
int __io_putchar(int ch);

void uart_hal_rx_CLI_buffer_init(void);
uint8_t uart_hal_CLI_getchar(void);
void uart_hal_rx_CLI_monitor(void);

extern uint8_t menu_cnt, submenu_cnt;
extern uint8_t AP_Conn_submenu, Milti_Conn_submenu, Client_Conn_submenu, CWmod_submenu, Server_mod_submenu, Close_submenu;
extern uint8_t Index_name_submenu, Hand_OP_submenu, Index_Read_submenu, Index_clear_submenu, Sensor_OP_submenu, Realtime_OP_submenu;

extern uint8_t AT_Flag, AT_RST_Flag, AT_MODE_Flag, AT_AP_Conn_Flag, Get_ip_flag, AT_MUX_Flag, AT_Conn_Flag, AT_Server_mod_Flag, AT_Close_Flag;
extern uint8_t AT_ECHO_Flag, AT_SensorOP_Flag;
extern uint8_t Sensor_Start, Motor_Start, Realtime_Start, Onetime_Start;
extern uint8_t EEPROM_STR_buf[5], EEPROM_IMU_buf[40], EEPROM_Force_buf[5];
extern uint16_t EEPROM_Index;

void AT_Test(void);
void Wifi_Reset_func(void);
void CWMode_Sub_Menu_Tree(void);
void CWMode_func(uint8_t mode);
void AP_Connection_func(char *SSID, char *PASSWD);
void Wifi_Get_IP(void);
void Multi_connection_mode(uint8_t mux);
void Client_to_Server_Connection(char *ip, uint16_t port);
void Server_Mode_Setting(uint8_t mode, uint16_t port);
void Connection_Close(uint8_t LinkID);
void WifiStatus(void);
uint16_t Start_Index_Addr_cal(uint16_t Index);
void Ready_Send_Echo(void);
void ECHO(void);
//void Ready_Send_Sensor_OP(void);
void Hand_Sensor_OP(void);

void Hand_Name_list(void);
void Hand_OP_EEPROM_Index_Read(uint16_t Index);
//void Run_Hand_Calibration(void);
void EEPROM_Index_Read(uint16_t Index);
void EEPROM_ALL_Clear(void);
void EEPROM_Index_Clear(uint16_t Index);
void EEPROM_Read(void);
void Ready_Onetime_rx_Send(void);
void Onetime_rx_Send(void);
void Ready_Onetime_Error_Send(void);
void Onetime_Error_Send(void);
void Ready_Realtime_rx_Send(void);
void Realtime_rx_Send(void);

void UART_Menu(void);
void UART_Menu1(void);
void UART_Menu2(void);
void Menu_Tree(uint8_t menu);
void AP_Sub_Menu_Tree(uint8_t sub_menu);
void Server_connection_Sub_Menu_Tree(uint8_t sub_menu);
void Server_mode_Sub_Menu_Tree(uint8_t sub_menu);
void Multi_Sub_Menu_Tree(void);
void Server_Close_Sub_Menu_Tree(void);
void Glove_data_Menu_Tree(uint8_t sub_menu);
void Index_Clear_Sub_Menu_Tree(void);
void Sensor_OP_Sub_Menu_Tree(void);

uint8_t ascii2hex(char asc);
unsigned short ascii_to_hex(char* buf);
void str2hex(char* str);
void hex2str(char* hex);
long map(long x, long in_min, long in_max, long out_min, long out_max);
float mapArduino(float val, float I_Min, float I_Max, float O_Min, float O_Max);

#endif /* INC_CLI_H_ */
