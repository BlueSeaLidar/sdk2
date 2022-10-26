#ifndef __UDP_DEMO_H_
#define __UDP_DEMO_H_

/**

 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved

 * File name:      udp_linux.h

 * Author:  	   *
     
 * Version:        1.0  
   
 * Date:		   2022.3.29

 * Description:    The Linux platform calls the function interface of the hardware    udp
 */

#include <pthread.h>
#include <stdint.h> 
#include "../data.h"

int open_socket_port(RunConfig &cfg);
unsigned int stm32crc(unsigned int *ptr, unsigned int len);
void send_cmd_udp_f(int fd_udp, const char* dev_ip, int dev_port,int cmd, int sn, int len, const void* snd_buf, bool bpr);
void send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port,int cmd, int sn, int len, const void* snd_buf);

void* lidar_thread_proc_udp(void* param);
RunConfig* openDev(const RunConfig& cfg);

bool udp_talk_GS_PACK(int fd_udp,const char* ip, int port, int n, const char* cmd,void *result);
bool udp_talk_S_PACK(int fd_udp, const char* ip, int port,int n, const char* cmd,void *result);
bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port,int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);

bool setup_lidar(int fd_udp, const char* ip, int port,int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth,int init_rpm,int should_post,char* version);
int setup_lidar_extre(int fd_udp, const char *ip, int port,  DevData &data);
#endif