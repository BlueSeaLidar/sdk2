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
struct CmdHeader
{
	unsigned short sign;//CN:与硬件约定的标志位						   EN:Flags consistent with hardware
	unsigned short cmd;	//CN:命令									  EN：command
	unsigned short sn;	//CN:随机数，发送报文和接收报文时验证是否一致	EN:Random numbers, verify consistency when sending and receiving messages
	unsigned short len;//CN:命令长度								  EN:command length
};

int open_socket_port(RunConfig &cfg);
/************************************************
* @functionName:  stm32crc
* @date:          2022-03-25
* @description:   CN:数据校验  EN:Data validation
* @Parameter:
				  1.ptr[unsigned int*,OUT]			CN:缓冲区指针			 EN:buffer pointer
				  2.len[unsigned int,IN]			CN:缓冲区长度			 EN:buffer length
* @return:        int  (useless)
* @others:        Null
*************************************************/
unsigned int stm32crc(unsigned int *ptr, unsigned int len);

/************************************************
* @functionName:  send_cmd_udp_f
* @date:          2022-03-25
* @description:   CN:发送UDP命令  EN:send UDP command
* @Parameter:
				  1.fd_udp[int,IN]			CN:socket初始化返回句柄			 EN:socket initialization return handle
				  2.dev_ip[const char*,IN]	CN:设备IP						 EN:device IP
				  3.dev_port[int,IN]		CN:设备端口号					 EN:device port
				  4.cmd[int,IN]				CN:硬件指令						 EN:hardware instructions
				  5.sn[int,IN]				CN:随机数						 EN:rand
				  6.len[int,IN]				CN:udp发送的字符串长度			 EN:Length of string sent by udp
				  7.snd_buf[const void*,OUT]CN:udp发送的字符串				 EN:string sent by udp
				  8.bpr						CN:打印数据标志位				 EN:print data flag
* @return:        true/false
* @others:        Null
*************************************************/
bool send_cmd_udp_f(int fd_udp, const char* dev_ip, int dev_port,int cmd, int sn, int len, const void* snd_buf, bool bpr);


/************************************************
* @functionName:  send_cmd_udp
* @date:          2022-03-25
* @description:   CN:发送UDP命令  EN:send UDP command 
* @Parameter:
				  1.fd_udp[int,IN]			CN:socket初始化返回句柄			 EN:socket initialization return handle
				  2.dev_ip[const char*,IN]	CN:设备IP						 EN:device IP
				  3.dev_port[int,IN]		CN:设备端口号					 EN:device port
				  4.cmd[int,IN]				CN:硬件指令						 EN:hardware instructions
				  5.sn[int,IN]				CN:随机数						 EN:rand
				  6.len[int,IN]				CN:udp发送的字符串长度			 EN:Length of string sent by udp
				  7.snd_buf[const void*,OUT]CN:udp发送的字符串				 EN: string sent by udp
* @return:        true/false
* @others:        Null
*************************************************/
bool send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port,int cmd, int sn, int len, const void* snd_buf);



/************************************************
* @functionName:  lidar_thread_proc
* @date:          2022-03-28
* @description:   CN:控制雷达运行的子程序(无须手动调用)  EN:Subroutines that control the operation of the radar(no need to call manually)
* @Parameter:
				  1.param[void*,OUT]			CN:配置文件数据			 EN:config file data

* @return:        NULL
* @others:        Null
*************************************************/
void* lidar_thread_proc_udp(void* param);

/************************************************
* @functionName:  openDev
* @date:          2022-03-28
* @description:   CN:启动雷达  EN:start radar
* @Parameter:
				  1.cfg[const RunConfig,OUT]			CN:配置文件数据			 EN:config file data

* @return:        config data
* @others:        Null
*************************************************/
RunConfig* openDev(const RunConfig& cfg);


bool udp_talk_GS_PACK(int fd_udp,const char* ip, int port, int n, const char* cmd,void *result);
bool udp_talk_S_PACK(int fd_udp, const char* ip, int port,int n, const char* cmd,void *result);
bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port,int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);

//设置设备的配置数据
int setup_lidar(int fd_udp, const char* ip, int port,int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth,printfMsg callback);
int setup_lidar_extre(int fd_udp, const char *ip, int port,  DevData &data);
#endif