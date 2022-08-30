#ifndef  _UART_WIN32_H_
#define _UART_WIN32_H_

/**

 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved

 * File name:      uart_win32.h

 * Author:  	   *
     
 * Version:        1.0  
   
 * Date:		   2022.3.25

 * Description:    The Windows platform calls the function interface of the hardware    Serial port/Virtual Serial port
 */


#include "data.h"
//public 
HANDLE open_serial_port(RunConfig & cfg);
DWORD  WINAPI  lidar_thread_proc_uart(void* param);
//uart  Port Com
int setup_lidar(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int rpm, char* version);
bool uart_talk(HANDLE hCom, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);

//Virtual Port Com(VPC)
int setup_lidar2(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int rpm, char* version);
bool uart_talk2(HANDLE hCom, int mode, int sn, int len, const char* cmd, int nfetch, char* fetch);
void send_cmd_uart(int hCom, int mode, int sn, int len, const char* cmd);
int setup_lidar_extre(int fd, DevData& data);


#endif // ! UART_WIN32_H_
