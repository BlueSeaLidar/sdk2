#ifndef _UDP_WIN32_H_
#define _UDP_WIN32_H_


/**

 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved

 * File name:      udp_win32.h

 * Author:  	   *
     
 * Version:        1.0  
   
 * Date:		   2022.3.25

 * Description:    The Windows platform calls the function interface of the hardware    udp
 */

#include"../data.h"

int open_socket_port(RunConfig& cfg);
int setup_lidar(int fd_udp, const char* ip, int port, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, char* version);
int setup_lidar_extre(int fd_udp, const char* ip, int port, DevData& data);
DWORD  WINAPI  lidar_thread_proc_udp(void* param);

unsigned int stm32crc(unsigned int* ptr, unsigned int len);

bool udp_talk_GS_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result);
bool udp_talk_S_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result);
bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);

void send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf);
void send_cmd_udp_f(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf, bool bpr);

#endif // _UDP_WIN32_H_


