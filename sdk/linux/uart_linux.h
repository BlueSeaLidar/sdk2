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

//CN:设备句柄   EN:Device handle

#include"../data.h"

extern int g_port;


/************************************************
* @functionName:  change_baud
* @date:          2022-03-28
* @description:   CN:修改非标准的波特率  EN:Modify non-standard baud rate
* @Parameter:	  
				  1.fd[int,IN]	            CN:设备句柄	EN:Device handle
				  2.baud[int,IN]			CN:波特率	EN:baud rate

* @return:        0  success   !=0 fail
* @others:        Null
*************************************************/
 extern "C"  int change_baud(int fd, int baud);


/************************************************
* @functionName:  open_serial_port
* @date:          2022-03-28
* @description:   CN:打开设备端口(第一步操作) EN:Open the device port (the first step)
* @Parameter:	  
				  1.port[const char*,IN]	CN:串口名称	EN:Serial port name
				  2.baudrate[int,IN]		CN:波特率	EN:baud rate

* @return:        int  >0 success   <=0 fail
* @others:        Null
*************************************************/
int open_serial_port(RunConfig &cfg);

/************************************************
* @functionName:  uart_talk
* @date:          2022-03-23
* @description:   CN:串口命令交互（子函数，一般不需要调用）  EN:Serial port command interaction (sub-functions, generally do not need to be called)
* @Parameter:
				  1.fd[int,IN]			CN:设备句柄（第一步操作后返回）	 EN:	Device handle (returned after the first step)
				  2.n[int,IN]				CN:传入交互指令长度				 EN:incoming command length
				  3.cmd[const char*,IN]		CN:传入交互指令					 EN:incoming command
				  4.nhdr[int,IN]			CN:保存的数据文件名称长度		 EN:Saved data file name length
				  5.hdr_str[const char*,IN] CN:保存的数据文件名称			 EN:Saved data file name
				  6.nfetch[int,IN]			CN:数据结果长度					 EN:data result length
				  7.fetch[char*,OUT]		CN:  数据结果					 EN:data result
* @return:        int  >0返回句柄编号   <=0 失败
* @others:        Null
*************************************************/

int uart_talk(int fd, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);
/************************************************
* @functionName:  strip
* @date:          2022-03-28
* @description:   CN:从缓存区提取设备编号  EN:从缓冲区中提取设备号
* @Parameter:	  
				  1.s[const char*,IN]	            CN:缓冲区	EN:buffer
				  2.buf[char*,OUT]			        CN:设备编号	EN:device  id

* @return:        int    CN:设备编号长度  EN:device id length
* @others:        Null
*************************************************/
int strip(const char* s, char* buf);

/************************************************
* @functionName:  setup_lidar
* @date:          2022-03-23
* @description:   CN:设置雷达的各项参数(第二步操作)	EN:Set the parameters of the radar (the second step)
* @Parameter:	  
				  1.hCom[HANDLE,IN]			CN:设备句柄（第一步操作后返回）										EN:	Device handle (returned after the first step)
				  2.unit_is_mm[int,IN]		CN:传入的数据单位名称 (0为厘米  1为毫米)							EN:Incoming data unit name (0 is cm and 1 is mm)	
				  3.with_confidence[int,IN] CN:数据中是否有带强度  (0为否，1为是)								EN:Is there any band intensity in the data (0 is no, 1 is yes)
				  4.resample[int,IN]		CN:分辨率 （0：原始数据，1：角度修正数据，200：0.2°,333：0.3°)	EN:Resolution (0: Original data, 1: Angle correction data, 200: 0.2°, 333: 0.3°)
				  5.with_deshadow[int,IN]   CN:去拖点(0：关闭，1：开启)											EN:go to drag point (0: off, 1: on)
				  6.with_smooth[int,IN]		CN:数据平滑(0：关闭， 1：开启)										EN:Data smoothing (0: off, 1: on)
* @return:        int  0  success    !=0 false
* @others:        Null
*************************************************/
int setup_lidar(int fd_uart, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm,printfMsg callback);

void *lidar_thread_proc_uart(void *param);

#endif