#ifndef  _UART_WIN32_H_
#define _UART_WIN32_H_

/**

 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved

 * File name:      uart_win32.h

 * Author:  	   *
     
 * Version:        1.0  
   
 * Date:		   2022.3.25

 * Description:    The Windows platform calls the function interface of the hardware    Serial/USB
 */


#include "data.h"
/************************************************
* @functionName:  OpenPort
* @date:          2022-03-23
* @description:   CN:打开设备端口(第一步操作) EN:Open the device port (the first step)
* @Parameter:	  
				  1.name[const char*,IN]	CN:串口名称	EN:Serial port name
				  2.speed[int,IN]			CN:波特率	EN:baud rate

* @return:        HANDLE   !=NULL   success    =NULL   false
* @others:        Null
*************************************************/
HANDLE open_serial_port(RunConfig & cfg);

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
int setup_lidar(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int rpm, char* version);

/************************************************
* @functionName:  uart_talk
* @date:          2022-03-23
* @description:   CN:串口命令交互（子函数，一般不需要调用）  EN:Serial port command interaction (sub-functions, generally do not need to be called)
* @Parameter:
				  1.hCom[HANDLE,IN]			CN:设备句柄（第一步操作后返回）	 EN:	Device handle (returned after the first step)
				  2.n[int,IN]				CN:传入交互指令长度				 EN:incoming command length
				  3.cmd[const char*,IN]		CN:传入交互指令					 EN:incoming command
				  4.nhdr[int,IN]			CN:保存的数据文件名称长度		 EN:Saved data file name length
				  5.hdr_str[const char*,IN] CN:保存的数据文件名称			 EN:Saved data file name
				  6.nfetch[int,IN]			CN:数据结果长度					 EN:data result length
				  7.fetch[char*,OUT]		CN:  数据结果					 EN:data result
* @return:        HANDLE  !=NULL  说明成功，反之失败
* @others:        Null
*************************************************/

int uart_talk(HANDLE hCom, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);

DWORD  WINAPI  lidar_thread_proc_uart(void*  param);
#endif // ! UART_WIN32_H_
