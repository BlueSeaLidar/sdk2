#ifndef __STANDARD_INTERFACE_H__
#define __STANDARD_INTERFACE_H__

#include"service/LidarWebService.h"
#include"data.h"
#include"error.h"
#ifdef _WIN32
#include"win32\uart_win32.h"
#include"win32\udp_win32.h"
#elif __unix__ 
#include"./linux/udp_linux.h"
#include"./linux/uart_linux.h"
#include <sys/ipc.h>
#include <sys/msg.h>
#endif

#define SDKVERSION   "1.0"  
/************************************************
* @functionName:  read_config
* @date:          2022-03-28
* @description:   Read and parse incoming configuration files
* @Parameter:	  
				  1.cfg_file_name[const char*,IN]	config file path
				  2.cfg[RunConfig,OUT]				Structure: save configuration information

* @return:        true/false
* @others:        Null
*************************************************/
bool read_config(const char* cfg_file_name, RunConfig& cfg);

/************************************************
* @functionName:  openDev
* @date:          2022-03-28
* @description:   start radar
* @Parameter:
				  1.cfg[const RunConfig,OUT]		 config file data

* @return:        socket handle
* @others:        Null
*************************************************/
int openDev(RunConfig& cfg);
/************************************************
* @functionName:  getLidarData
* @date:          2022-05-05
* @description:   Get printed data from radar
* @Parameter:	  1.threadID [long,IN] Child thread ID/message queue ID
*				  2.dataGet	[bool,IN]  print  flag
* @return:        0  success   !=0  false
* @others:        get data from allback function 
*************************************************/
int getLidarData(long threadID,bool dataGet);

/************************************************
* @functionName:  GetDevInfo
* @date:          2022-05-05
* @description:   Get device configuration data
* @Parameter:	  1.threadID [long,IN]		   Child thread ID/message queue ID
*				  2.data	[EEpromV101,OUT]   lidar points data
* @return:        0  success   !=0  false
* @others:        Null
*************************************************/
int GetDevInfo(long threadID, EEpromV101& data);
/************************************************
* @functionName:  SetDevInfo_extre
* @date:          2022-05-05
* @description:   Set device configuration data
* @Parameter:	  1.threadID [long,IN]			 Child thread ID/message queue ID
*				  2.data	[DevData,IN/OUT]     set lidar config
* @return:        0
* @others:        Null
*************************************************/
int SetDevInfo_extre(long threadID, DevData &data);

/************************************************
* @functionName:  GetLidarTimestamp
* @date:          2022-05-05
* @description:   Get radar timestamp information
* @Parameter:	  1.threadID [long,IN]  Child thread ID/message queue ID
*				  2.dataGet	[bool,IN]   print flag
* @return:        0
* @others:        Null
*************************************************/
int GetLidarTimestamp(long threadID, bool dataGet);

/************************************************
* @functionName:  OpenLocalService
* @date:          2022-07-01
* @description:   Open the local service and can use http://local+(IP)+(port) Access browser html page, default is http://localhost:8888
* @Parameter:
				  1.run[RunConfig,OUT]			config file data

* @return:        Null
* @others:        Null
*************************************************/
void  OpenLocalService(RunConfig& cfg);
/************************************************
* @functionName:  CloseLocalService
* @date:          2022-07-01
* @description:   close the local service 
* @Parameter:
				  1.run[RunConfig,OUT]			config file data

* @return:        Null
* @others:        Null
*************************************************/
void  CloseLocalService();












int ControlDrv(long threadID, const char* data);
void StopDrv(RunConfig* run);

const char* getVersion();
void StringReplace(std::string& strBase, std::string strSrc, std::string strDes);
char* jsonValue(const char* result, const char* message, cJSON* item);
extern RunConfig* g_cfg;
#endif