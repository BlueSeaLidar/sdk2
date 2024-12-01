#ifndef __STANDARD_INTERFACE_H__
#define __STANDARD_INTERFACE_H__

#include "LidarDataProcess.h"
#include "LidarCheckService.h"
#include "error.h"
#define SDKVERSION "3.2.1" // SDK版本号

class BlueSeaLidarSDK
{
public:
	static BlueSeaLidarSDK *getInstance();
	static void deleteInstance();

	/************************************************
	 * @functionName:  getLidar
	 * @date:          2022-08-21
	 * @description:   CN:获取当前雷达的信息  EN:get current lidar info
	 * @Parameter:
	 *				  1.ID[int,IN]			CN:雷达ID			 EN:lidar ID
	 *
	 * @return:        RunConfig  struct
	 * @others:        Null
	 *************************************************/
	RunConfig *getLidar(int ID);
	/************************************************
	 * @functionName:  addLidarByPath
	 * @date:          2022-07-14
	 * @description:   CN:通过配置文件的路径添加ID  EN:add lidar by path
	 * @Parameter:
	 *				  1.cfg_file_name[const char*,IN]			CN:配置文件路径			 EN:config file path
	 *
	 * @return:        LIDAR ID
	 * @others:        Null
	 *************************************************/
	int addLidarByPath(const char *cfg_file_name);
	/************************************************
	 * @functionName:  delLidarByID
	 * @date:          2022-07-14
	 * @description:   CN:通过addLidarByPath返回的ID删除指定雷达  EN:Delete the specified lidar by the ID returned by addLidarByPath
	 * @Parameter:
	 *				  1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	 *
	 * @return:        true/false
	 * @others:        Null
	 *************************************************/
	bool delLidarByID(int ID);
	/************************************************
	 * @functionName:  setCallBackPtr
	 * @date:          2022-07-14
	 * @description:   CN:根据ID设置雷达的回调打印函数  EN:Setting up a callback print function for the radar based on an ID
	 * @Parameter:
	 *				  1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	 *				  1.ptr[printfMsg,IN]	CN:回调打印函数		EN:Callback print function
	 *
	 * @return:        NUll
	 * @others:        Null
	 *************************************************/
	void setCallBackPtr(int ID, printfMsg ptr);
	/************************************************
	 * @functionName:  openDev
	 * @date:          2022-03-28
	 * @description:   CN:启动雷达  EN:start lidar
	 * @Parameter:
	 *				  1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	 *
	 * @return:        true/false
	 * @others:        Null
	 *************************************************/
	bool openDev(int ID);
	/************************************************
	 * @functionName:  StopDrv
	 * @date:          2022-03-28
	 * @description:   CN:关闭雷达  EN:stop lidar
	 * @Parameter:
	 *				  1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	 *
	 * @return:        Null
	 * @others:        Null
	 *************************************************/
	void StopDev(int ID);
	/************************************************
	 * @functionName:  GetDevInfo
	 * @date:          2023-08-21
	 * @description:   CN:获取雷达全局参数  EN:get lidar config arg
	 * @Parameter:
	 *				  1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	 *				  2.eepromv101[EEpromV101,OUT]			CN:雷达参数信息			 EN:Lidar arg info
	 *
	 * @return:        true/false
	 * @others:        Null
	 *************************************************/
	bool GetDevInfo(int ID, EEpromV101 *eepromv101);

	/************************************************
	 * @functionName:  ControlDrv
	 * @date:          2023-08-21
	 * @description:   CN:设置雷达开启或者暂停  EN:Set the lidar on or off
	 * @Parameter:
	 *					1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	 *				    2.num	[int,IN]   CN:指令长度	EN:cmd length
	 *					3.cmd	[int,IN]   CN:指令内容	EN:cmd  data
	 * @return:			true/false
	 * @others:        Null
	 *************************************************/
	bool ControlDrv(int ID, int num, char *cmd);

	/************************************************
	 * @functionName:  getVersion
	 * @date:          2022-05-09
	 * @description:   CN:获取版本号  EN:get version
	 * @Parameter:	  Null
	 * @return:        const char*
	 * @others:        Null
	 *************************************************/
	const char *getVersion();
	/************************************************
	 * @functionName:  ZoneSection
	 * @date:          2022-10-18
	 * @description:   CN:切换当前防区  EN:change  section
	 * @Parameter:	  1.ID [long,IN]  CN: 雷达ID  EN:Lidar ID
	 *				  2.section	[char,IN]   CN:激活的防区序号	EN:Active zone serial number
	 * @return:        true/false
	 * @others:        防区取值范围 0-9 a-f
	 *************************************************/
	bool ZoneSection(int ID, char section);
	/************************************************
	 * @functionName:  SetUDP
	 * @date:          2023-07-31
	 * @description:   CN:设置雷达IP  EN:set lidar ip
	 * @Parameter:		1.ID [long,IN]  CN: 雷达ID  EN:Lidar ID
	 *					2.ip	[char*,IN]   CN:ip地址 	EN:ip
	 *					3.mask	[char*,IN]   CN:子网掩码	EN:mask
	 *					4.gateway	[char*,IN]   CN:网关 	EN:gateway
	 *					5.port	[int,IN]   CN:端口 	EN:port
	 * @return:        true/false
	 * @others:        Null
	 *************************************************/
	bool SetUDP(int ID, char *ip, char *mask, char *gateway, int port);
	/************************************************
	 * @functionName:  SetDST
	 * @date:          2023-07-31
	 * @description:   CN:设置雷达上传IP  EN:set lidar post ip  address
	 * @Parameter:		1.ID [long,IN]  CN: 雷达ID  EN:Lidar ID
	 *					2.ip[char*,IN]   CN:ip地址 	EN:ip
	 *					3.port[int,IN]   CN:端口 	EN:port
	 * @return:        true/false
	 * @others:        Null
	 *************************************************/
	bool SetDST(int ID, char *ip, int port);
	/************************************************
	 * @functionName:  SetRPM
	 * @date:          2023-08-21
	 * @description:   CN:设置雷达转速  EN:set lidar rpm
	 * @Parameter:		1.ID [long,IN]  CN: 雷达ID  EN:Lidar ID
	 *					2.RPM[int,IN]   CN:转速 	EN:rpm
	 * @return:        true/false
	 * @others:        rpm   (300-3000)
	 *************************************************/
	bool SetRPM(int ID, int RPM);
	/************************************************
	 * @functionName:  SetTFX
	 * @date:          2023-08-21
	 * @description:   CN:设置雷达固定上传  EN:set lidar fixed upload
	 * @Parameter:		1.ID [long,IN]  CN: 雷达ID  EN:Lidar ID
	 *					2.tfx[bool,IN]   true/false
	 * @return:        true/false
	 * @others:        Null
	 *************************************************/
	bool SetTFX(int ID, bool tfx);
	/************************************************
	 * @functionName:  SetDSW
	 * @date:          2023-08-21
	 * @description:   CN:设置雷达滤波，去拖点  EN:set lidar filtering
	 * @Parameter:		1.ID [long,IN]  CN: 雷达ID  EN:Lidar ID
	 *					2.tfx[bool,IN]   true/false
	 * @return:        true/false
	 * @others:        Null
	 *************************************************/
	bool SetDSW(int ID, bool dsw);
	/************************************************
	 * @functionName:  SetSMT
	 * @date:          2023-08-21
	 * @description:   CN:设置雷达数据平滑  EN:set lidar smooth
	 * @Parameter:		1.ID [long,IN]  CN: 雷达ID  EN:Lidar ID
	 *					2.smt[bool,IN]   true/false
	 * @return:        true/false
	 * @others:        Null
	 *************************************************/
	bool SetSMT(int ID, bool smt);
	/************************************************
	 * @functionName:  SetPST
	 * @date:          2023-08-21
	 * @description:   CN:设置雷达上传方式  EN:set lidar upload way
	 * @Parameter:		1.ID [long,IN]  CN: 雷达ID  EN:Lidar ID
	 *					2.mode[int,IN]  CN:0 无数据 1仅数据上传 2仅报警 3 数据+报警  EN:0 no data 1 only data 2 only alarm 3 data+alarm
	 * @return:        true/false
	 * @others:        Null
	 *************************************************/
	bool SetPST(int ID, int mode);
	/************************************************
	 * @functionName:  SetDID
	 * @date:          2023-08-21
	 * @description:   CN:设置雷达编号  EN:set lidar number
	 * @Parameter:		1.ID [long,IN]  CN: 雷达ID  EN:Lidar ID
	 *					2.number[unsigned int,IN]   CN：雷达的编号  EN:lidar number
	 * @return:        true/false
	 * @others:        Repeatable, different from ID
	 *************************************************/
	bool SetDID(int ID, unsigned int number);
	/************************************************
	 * @functionName:  getLidarsList
	 * @date:          2023-12-13
	 * @description:   CN:获取网络款雷达的心跳包数据和当前可用的串口雷达  EN:Get heartbeat packet data for networked Lidars and currently available serial Lidars
	 * @return:        vector
	 * @others:        The network heartbeat packet contains the received timestamp, which needs to be compared with the current timestamp to see if it is offline.
	 *************************************************/
	std::vector<DevConnInfo> getLidarsList();
	bool OpenHeartService();
	bool CloseHeartService();

protected:
	// 设置雷达参数(掉电存储)
	bool SetDevInfo(RunConfig *lidar, int length, char *cmd, int mode);
	// 打开本地服务(web测试可视化页面)
	bool OpenLocalService(int ID);
	// 关闭本地服务
	bool CloseLocalService(int ID);

private:
	static BlueSeaLidarSDK *m_sdk;
	BlueSeaLidarSDK();
	~BlueSeaLidarSDK();

	int m_idx;
	std::vector<RunConfig *> m_lidars;

	LidarCheckService *m_checkservice;
	bool m_checkflag; // 控制心跳线程关闭
};
void *lidar_service(void *param);
// 默认的打印函数
void CallBackMsg(int msgtype, void *param, int length);

#endif