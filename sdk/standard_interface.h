#ifndef __STANDARD_INTERFACE_H__
#define __STANDARD_INTERFACE_H__

#include"ZoneAlarm.h"
#include"LidarDataProcess.h"
#include"error.h"
#ifdef _WIN32

#elif __unix__ 
#include <sys/ipc.h>
#include <sys/msg.h>
#endif

#define SDKVERSION   "2.0"  //SDK版本号

/************************************************
* @functionName:  read_config
* @date:          2022-03-28
* @description:   CN:读取解析传入的配置文件  EN:Read and parse incoming configuration files
* @Parameter:	  
				  1.cfg_file_name[const char*,IN]	            CN:配置文件路径	EN:config file path
				  2.cfg[RunConfig,OUT]			CN:保存配置信息结构体	EN:Structure: save configuration information

* @return:        true/false
* @others:        Null
*************************************************/
bool read_config(const char* cfg_file_name, RunConfig& cfg);


/************************************************
* @functionName:  openDev
* @date:          2022-03-28
* @description:   CN:启动雷达  EN:start radar
* @Parameter:
				  1.cfg[const RunConfig,OUT]			CN:配置文件数据			 EN:config file data

* @return:        socket句柄或者文件句柄
* @others:        Null
*************************************************/
int openDev(RunConfig& cfg, int index);
/************************************************
* @functionName:  thread_set_run
* @date:          2022-05-05
* @description:   CN:子线程启动雷达  EN:child thread start radar
* @Parameter:
				  1.cfg[const RunConfig,OUT]			CN:配置文件数据			 EN:config file data

* @return:        linux  0 success  other failed    win32   >0 success  <=0 failed
* @others:        Null
*************************************************/
int thread_set_run(RunConfig& cfg);
/************************************************
* @functionName:  getLidarData
* @date:          2022-05-05
* @description:   CN:获取雷达的打印数据  EN:Get printed data from radar
* @Parameter:	  1.threadID [long,IN]  CN:子线程ID/消息队列ID  EN:Child thread ID/message queue ID
*				  2.dataGet	[bool,IN]   CN:打印开关			   EN:is print?
* @return:        0  success   !=0  false
* @others:        get data from allback function 
*************************************************/
int getLidarData(long threadID,bool dataGet);

/************************************************
* @functionName:  GetDevInfo
* @date:          2022-05-05
* @description:   CN:获取设备配置数据  EN:Get device configuration data
* @Parameter:	  1.threadID [long,IN]  CN:子线程ID/消息队列ID  EN:Child thread ID/message queue ID
*				  2.data	[EEpromV101,OUT]   CN:雷达点位数据  EN:lidar points data
* @return:        0  success   !=0  false
* @others:        Null
*************************************************/
int GetDevInfo(long threadID, EEpromV101& data);
/************************************************
* @functionName:  SetDevInfo_extre
* @date:          2022-05-05
* @description:   CN:设置设备配置参数  EN:Set device configuration data
* @Parameter:	  1.threadID [long,IN]  CN:子线程ID/消息队列ID  EN:Child thread ID/message queue ID
*				  2.data	[DevData,IN/OUT]   CN:设置雷达参数  EN:set lidar config
* @return:        0
* @others:        Null
*************************************************/
int SetDevInfo_extre(long threadID, DevData &data);

/************************************************
* @functionName:  GetLidarTimestamp
* @date:          2022-05-05
* @description:   CN:获取雷达时间戳信息  EN:Get radar timestamp information
* @Parameter:	  1.threadID [long,IN]  CN:子线程ID/消息队列ID  EN:Child thread ID/message queue ID
*				  2.dataGet	[bool,IN]   CN:打印开关			   EN:is print?
* @return:        0
* @others:        Null
*************************************************/
int GetLidarTimestamp(long threadID, bool dataGet);

/************************************************
* @functionName:  GetAlarmZone
* @date:          2022-07-27
* @description:   CN:获取防区数据  EN:Get zone data
* @Parameter:	  1.threadID [long,IN]  CN:子线程ID/消息队列ID  EN:Child thread ID/message queue ID
*				  2.data	[RecvZoneDatas,OUT]   CN:防区信息			   EN:zone info
* @return:        0  success   !=0  false
* @others:        Null
*************************************************/
int GetAlarmZone(long threadID, RecvZoneDatas&data);
/************************************************
* @functionName:  SetAlarmZone
* @date:          2022-07-27
* @description:   CN:设置防区数据  EN:Set zone data
* @Parameter:	  1.threadID [long,IN]  CN:子线程ID/消息队列ID  EN:Child thread ID/message queue ID
*				  2.data	[zones,IN]   CN:防区信息			   EN:zone info
* @return:        0  success   !=0  false
* @others:        Null
*************************************************/
int SetAlarmZone(long threadID, zones &data);
/************************************************
* @functionName:  StopDrv
* @date:          2022-03-28
* @description:   CN:关闭雷达  EN:stop radar
* @Parameter:
				  1.run[RunConfig,OUT]			CN:配置文件数据			 EN:config file data

* @return:        Null
* @others:        Null
*************************************************/
void StopDrv(RunConfig* run);

/************************************************
* @functionName:  ControlDrv
* @date:          2022-03-28
* @description:   CN:设置雷达开启或者暂停  EN:Set the radar on or off
* @Parameter:	  1.threadID [long,IN]  CN:子线程ID/消息队列ID  EN:Child thread ID/message queue ID
*				  2.data	[const char*,IN]   CN:雷达启停测距指令	EN:conctrl lidar cmd
* @return:        
* @others:        Null
*************************************************/
int ControlDrv(long threadID, const char*data);

/************************************************
* @functionName:  getVersion
* @date:          2022-05-09
* @description:   CN:获取版本号  EN:get version
* @Parameter:	  Null
* @return:        const char* 
* @others:        Null
*************************************************/
const char* getVersion();
/************************************************
* @functionName:  ZoneSection
* @date:          2022-10-18
* @description:   CN:读取/设置激活防区  EN:get/set alarmzone  section
* @Parameter:	  1.threadID [long,IN]  CN:子线程ID/消息队列ID  EN:Child thread ID/message queue ID
*				  2.section	[int,IN]   CN:激活的防区序号	EN:Active zone serial number
* @return:        3.mode[int,IN]     CN: 0读取 1设置 激活的防区 EN:0 Read 1 Set the active zone
* @others:        Null
*************************************************/
int ZoneSection(long threadID,int section,int mode);


//打开本地服务(web测试可视化页面)
void  OpenLocalService(RunConfig& cfg, int index);
//关闭本地服务
void  CloseLocalService();

int GetOnePoint_MSG(long threadID, PointData& data);
extern RunConfig *g_cfg[MAX_LIDARS];


#endif