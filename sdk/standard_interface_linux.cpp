#ifndef __STANDARD_INTERFACE_LINUX_H__
#define __STANDARD_INTERFACE_LINUX_H__


#include "standard_interface.h"
#include"service/LidarWebService.h"

RunConfig* g_cfg[MAX_LIDARS];
LidarWebService* webservice;
bool read_config(const char* cfg_file_name, RunConfig& cfg)
{
	return readConfig(cfg_file_name, cfg);
}

// 释放连接
void StopDrv(RunConfig* run)
{
	run->should_quit = true;
	sleep(2);
}


const char* getVersion()
{
	return SDKVERSION;
}
void* lidar_service(void* param)
{
	int *index = (int*)param;
	webservice->OpenLocalService(*index);
	return SUCCESS;
}
// 打开本地服务
void  OpenLocalService(RunConfig & cfg,int index)
{
	webservice = new LidarWebService(cfg.service_port);
	pthread_create(&cfg.thread, NULL, lidar_service, &index);
}
//关闭本地服务
void  CloseLocalService()
{

}
// 连接雷达，启动子线程
int openDev(RunConfig& cfg,int index)
{
	int fd = -1;//保存句柄变量
	int res = -1;//保存函数返回值
	if (strcmp(cfg.type, "uart") == 0|| strcmp(cfg.type, "vpc") == 0)
	{
		fd = (int)open_serial_port(cfg);
		if (fd <= 0)
		{
			return OPEN_UART_FD_FAILED;
		}
		//传入设备句柄
		cfg.fd = fd;
		g_cfg[index] = &cfg;
		cfg.thread_ID[1] = msgget(0x12345+index, IPC_CREAT | 0664);
		if (pthread_create(&cfg.thread, NULL, lidar_thread_proc_uart, &cfg) != 0)
			return THREAD_CREATE_FAILED;
		INFO_PR("Lidar is open success\n");
		return SUCCESS;
	}
	else if (strcmp(cfg.type, "udp") == 0)
	{
		fd = (int)open_socket_port(cfg);
		if (fd <= 0)
		{
			return OPEN_UDP_FD_FAILED;
		}
		//传入设备句柄
		cfg.fd = fd;
		g_cfg[index] = &cfg;
		cfg.thread_ID[1] = msgget(0x12345+index, IPC_CREAT | 0664);
		if (pthread_create(&cfg.thread, NULL, lidar_thread_proc_udp, &cfg) != 0)
			return THREAD_CREATE_FAILED;

		INFO_PR("Lidar is open success\n");
		return SUCCESS;

	}
	else
	{
		return ARG_ERROR_TYPE;
	}
}
int thread_set_run(RunConfig& cfg)
{
	if (strcmp(cfg.type, "uart") == 0||strcmp(cfg.type, "vpc") == 0)
		return pthread_create(&cfg.thread, NULL, lidar_thread_proc_uart, &cfg);
	else if (strcmp(cfg.type, "udp") == 0)
		return pthread_create(&cfg.thread, NULL, lidar_thread_proc_udp, &cfg);
	else
		return ARG_ERROR_TYPE;

}

int getLidarData(long threadID, bool dataGet)
{
	USER_MSG msg;
	msg.type = 1;
	msg.cmd.type2 = Print_Point_MSG;
	msg.cmd.str[0] = dataGet;
	int res = msgsnd(threadID, &msg, sizeof(msg.cmd), 0);

	USER_MSG msg2;
	sleep(0.5);
	int index = 5;
	while (index--)
	{
		if (msgrcv(threadID, &msg2, sizeof(msg2.cmd), 2, IPC_NOWAIT) >= 0)
		{
			if (msg2.cmd.type2 == Print_Point_MSG)
			{
				INFO_PR("%s recv MSG\n", __FUNCTION__);
				if (msg2.cmd.str[0] - '0')
					return SUCCESS;
				else
					return GET_DEVTIMESTAMP_FAILED;
			}
			else {
				DEBUG_PR("%s msg error!\n", __FUNCTION__);
			}
		}
		sleep(1);
	}
	return GET_DEVPONIT_FAILED;
}
int GetDevInfo(long threadID, EEpromV101& data)
{
	USER_MSG msg;
	msg.type = 1;
	msg.cmd.type2 = GetDevInfo_MSG;
	int res = msgsnd(threadID, &msg, sizeof(msg.cmd), 0);
	USER_MSG msg2;
	sleep(0.5);
	int index = 5;
	while (index--)
	{
		if (msgrcv(threadID, &msg2, sizeof(msg2.cmd), 2, IPC_NOWAIT) >= 0)
		{
			if (msg2.cmd.type2 == GetDevInfo_MSG)
			{
				memcpy(&data, msg2.cmd.str, sizeof(EEpromV101));
				INFO_PR("GetDevInfo recv MSG\n");
				return SUCCESS;
			}
			else {
				DEBUG_PR("%s msg error!\n", __FUNCTION__);
			}
		}
		sleep(1);
	}
	return GET_DEVINFO_FAILED;
}

int SetDevInfo_extre(long threadID, DevData& data)
{
	USER_MSG msg;
	msg.type = 1;
	msg.cmd.type2 = SetDevInfo_MSG;
	memcpy(&msg.cmd.str, &data, sizeof(DevData));
	int res = msgsnd(threadID, &msg, sizeof(msg.cmd), 0);
	USER_MSG msg2;
	sleep(0.5);
	int index = 5;
	while (index--)
	{
		if (msgrcv(threadID, &msg2, sizeof(msg.cmd), 2, IPC_NOWAIT) >= 0)
		{
			if (msg2.cmd.type2 == SetDevInfo_MSG)
			{
				memcpy(&data, &msg2.cmd.str, sizeof(DevData));
				INFO_PR("%s recv MSG\n", __FUNCTION__);
				for (int i = 0; i < sizeof(data.result); i++)
				{
					if (data.result[i] == 'O' || data.result[i] == 'N')
					{
						return SUCCESS;
					}

				}
				return SET_DEVINFO_FAILED;
			}
			else {
				DEBUG_PR("%s msg error!\n", __FUNCTION__);
			}
		}
		sleep(1);
	}
	return SET_DEVINFO_FAILED;
}
// 启停雷达测距
int ControlDrv(long threadID, const char* data)
{
	USER_MSG msg;
	msg.type = 1;
	msg.cmd.type2 = ctrl_MSG;
	strcpy(msg.cmd.str, data);
	int res = msgsnd(threadID, &msg, sizeof(msg.cmd), 0);

	USER_MSG msg2;
	sleep(0.5);
	int index = 5;
	while (index--)
	{
		if (msgrcv(threadID, &msg2, sizeof(msg2.cmd), 2, IPC_NOWAIT) >= 0)
		{
			if (msg2.cmd.type2 == ctrl_MSG)
			{
				INFO_PR("%s recv MSG\n", __FUNCTION__);
				return SUCCESS;
			}
			else {
				DEBUG_PR("%s msg error!\n", __FUNCTION__);
			}
		}
		sleep(1);
	}
	return SET_DEV_FAILED;
}
int GetLidarTimestamp(long threadID, bool dataGet)
{
	USER_MSG msg;
	msg.type = 1;
	msg.cmd.type2 = Print_TimeStamp_MSG;
	msg.cmd.str[0] = dataGet;
	int res = msgsnd(threadID, &msg, sizeof(msg.cmd), 0);

	USER_MSG msg2;
	sleep(0.5);
	int index = 5;
	while (index--)
	{
		if (msgrcv(threadID, &msg2, sizeof(msg2.cmd), 2, IPC_NOWAIT) >= 0)
		{
			if (msg2.cmd.type2 == Print_TimeStamp_MSG)
			{
				INFO_PR("%s recv MSG\n", __FUNCTION__);
				if (msg2.cmd.str[0] - '0')
					return SUCCESS;
				else
					return GET_DEVTIMESTAMP_FAILED;

			}
			else {
				DEBUG_PR("%s msg error!\n", __FUNCTION__);
			}
		}
		sleep(1);
	}
	return GET_DEVTIMESTAMP_FAILED;
}

int GetAlarmZone(long threadID, RecvZoneDatas& data)
{
	USER_MSG msg;
	msg.type = 1;
	msg.cmd.type2 = Get_ZONE_MSG;
	msg.cmd.str[0] = true;
	int res = msgsnd(threadID, &msg, sizeof(msg.cmd), 0);
	USER_MSG msg2;
	sleep(0.5);
	int index = 5;
	while (index)
	{
		if (msgrcv(threadID, &msg2, sizeof(msg2.cmd), 2, IPC_NOWAIT) >= 0)
		{
			if (msg2.cmd.type2 == Get_ZONE_MSG)
			{
				INFO_PR("%s recv MSG\n", __FUNCTION__);
				memcpy(&data,msg2.cmd.str,sizeof(RecvZoneDatas));
				if (data.result == true)
					return SUCCESS;
				else
					return Get_ZONE_FAILED;

			}
			else {
				DEBUG_PR("%s msg error!\n", __FUNCTION__);
			}
		}
		sleep(1);
	}
	return Get_ZONE_FAILED;
}
int SetAlarmZone(long threadID, zones& data)
{
	USER_MSG msg;
	msg.type = 1;
	msg.cmd.type2 = Set_ZONE_MSG;
	memcpy(&msg.cmd.str, &data, sizeof(DevData));
	int res = msgsnd(threadID, &msg, sizeof(msg.cmd), 0);

	USER_MSG msg2;
	sleep(0.5);
	int index = 5;
	while (index)
	{
		if (msgrcv(threadID, &msg2, sizeof(msg2.cmd), 2, IPC_NOWAIT) >= 0)
		{
			if (msg2.cmd.type2 == Set_ZONE_MSG)
			{
				int *rev= new int;
				memcpy(rev, &msg2.cmd.str, sizeof(int));
				INFO_PR("%s recv MSG\n", __FUNCTION__);
				if(*rev == true)
					return SUCCESS;
				else
					return Set_ZONE_FAILED;
			}
			else {
				DEBUG_PR("%s msg error!\n", __FUNCTION__);
			}
		}
		sleep(1);
	}
	return Set_ZONE_FAILED;
}
int ZoneSection(long threadID, int section, int mode)
{
	USER_MSG msg;
	msg.type = 1;
	msg.cmd.type2 = Set_ZoneSection_MSG;
	memcpy(&msg.cmd.str, &section, sizeof(int));
	int res = msgsnd(threadID, &msg, sizeof(msg.cmd), 0);

	USER_MSG msg2;
	sleep(0.5);
	int index = 5;
	while (index)
	{
		if (msgrcv(threadID, &msg2, sizeof(msg2.cmd), 2, IPC_NOWAIT) >= 0)
		{
			if (msg2.cmd.type2 == Set_ZoneSection_MSG)
			{
				char rev[2]={0};
				memcpy(rev, &msg2.cmd.str, 2);
				INFO_PR("%s recv MSG %s\n", __FUNCTION__,rev);
				if(strcmp(rev,"OK")==0)
					return SUCCESS;
				else
					return SET_ZONESECTION_FAILED;
			}
			else {
				DEBUG_PR("%s msg error! %s\n", __FUNCTION__);
			}
		}
		sleep(1);
	}
	return SET_ZONESECTION_FAILED;
}
#endif