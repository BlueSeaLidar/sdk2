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

extern char lidar_ip[256];   //CN:雷达的网络地址    EN:Radar's network address
extern int lidar_port;		 //CN:雷达的端口号		EN:Radar's  port


int open_socket_port(RunConfig& cfg);

//CN:UDP设置雷达参数时接收使用报文头	EN:UDP uses the header when setting radar parameters
struct CmdHeader
{
	unsigned short sign; //CN:与硬件约定的标志位						EN:Flags consistent with hardware
	unsigned short cmd;	 //CN:命令										EN：command
	unsigned short sn;	//CN:随机数，发送报文和接收报文时验证是否一致	EN:Random numbers, verify consistency when sending and receiving messages
	unsigned short len;	//CN:命令长度									EN:command length
};

/************************************************
* @functionName:  setup_lidar
* @date:          2022-03-25
* @description:   CN:设置雷达的各项参数		EN:Set the parameters of the radar (the second step)
* @Parameter:
				  1.fd_udp[int,IN]			CN:socket初始化返回句柄												EN:	socket initialization return handle
				  2.unit_is_mm[int,IN]		CN:传入的数据单位名称 (0为厘米  1为毫米)							EN:Incoming data unit name (0 is cm and 1 is mm)
				  3.with_confidence[int,IN] CN:数据中是否有带强度  (0为否，1为是)								EN:Is there any band intensity in the data (0 is no, 1 is yes)
				  4.resample[int,IN]		CN:分辨率 （0：原始数据，1：角度修正数据，200：0.2°,333：0.3°)	EN:Resolution (0: Original data, 1: Angle correction data, 200: 0.2°, 333: 0.3°)
				  5.with_deshadow[int,IN]   CN:去拖点(0：关闭，1：开启)											EN:go to drag point (0: off, 1: on)
				  6.with_smooth[int,IN]		CN:数据平滑(0：关闭， 1：开启)										EN:Data smoothing (0: off, 1: on)
* @return:        int  0  success    !=0 false
* @others:        Null
*************************************************/
int setup_lidar(int fd_udp, const char* ip, int port, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, char* version);


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
unsigned int stm32crc(unsigned int* ptr, unsigned int len);
int setup_lidar_extre(int fd_udp, const char* ip, int port, DevData &data);


DWORD  WINAPI  lidar_thread_proc_udp(void*  param);


bool udp_talk_GS_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result);
bool udp_talk_S_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result);
bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);
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
bool send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf);

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
bool send_cmd_udp_f(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf, bool bpr);

#endif // _UDP_WIN32_H_


