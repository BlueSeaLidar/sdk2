#ifndef __UART_DEMO_H_
#define __UART_DEMO_H_
/**

 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved

 * File name:      uart_linux.h

 * Author:  	   *
     
 * Version:        1.0  
   
 * Date:		   2022.3.28

 * Description:    The Linux(Ubuntu) platform calls the function interface of the hardware    Serial/USB
 */


#include"../data.h"
//public
extern "C"  int change_baud(int fd, int baud);
int open_serial_port(RunConfig &cfg);
void *lidar_thread_proc_uart(void *param);
int strip(const char* s, char* buf);

//uart
bool uart_talk(int fd, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);
int setup_lidar(int fd_uart, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm,char* version);

//Virtual serial communication
void send_cmd_uart(int fd, int mode, int sn, int len, const char* cmd);
bool uart_talk2(int hCom, int mode, int sn, int len, const char* cmd, int nfetch, char* fetch);
bool uart_talk3(int  hCom, int mode, int sn, int len, const char* cmd, int result_len,void *result);
int setup_lidar_extre(int fd, DevData& data);
int setup_lidar2(int hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char* version);
#endif