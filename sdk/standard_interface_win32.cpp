#ifndef __STANDARD_INTERFACE_WIN32_H__
#define __STANDARD_INTERFACE_WIN32_H__

#include "standard_interface.h"
#include"service/LidarWebService.h"

RunConfig *g_cfg[MAX_LIDARS];
LidarWebService* webservice;

bool read_config(const char* cfg_file_name, RunConfig& cfg)
{
	return readConfig(cfg_file_name, cfg);
}

// �ͷ�����
void StopDrv(RunConfig* run)
{
	run->should_quit = true;
	sleep(2);
}


const char* getVersion()
{
	return SDKVERSION;
}
int ZoneSection(long threadID, int section, int mode)
{
	MSG msg;
	int* cmd = new int;
	*cmd = section;
	if (!PostThreadMessage(threadID, Set_ZoneSection_MSG, (WPARAM)cmd, GetCurrentThreadId()))
	{
		delete cmd;
		return MSG_POST_FAILED;
	}
	sleep(0.1);
	int index = SLEEP_SIZE;
	while (index--)
	{
		if (PeekMessage(&msg, NULL, Set_ZoneSection_MSG, Set_ZoneSection_MSG, PM_REMOVE))
		{
			switch (msg.message)
			{
			case Set_ZoneSection_MSG:
				char* revdata = (char*)msg.wParam;	
				INFO_PR("SetZoneSection recv MSG %s\n", revdata);
				if (strcmp(revdata, "OK") == 0)
				{
					delete[] revdata;
					return SUCCESS;
				}	
				return SET_ZONESECTION_FAILED;
			}
		}
		sleep(1);
	}
	return SET_ZONESECTION_FAILED;
}
DWORD  WINAPI lidar_service(void* param)
{
	int *index = (int*)param;
	webservice->OpenLocalService(*index);
	return SUCCESS;
}
//�򿪱��ط���
void  OpenLocalService(RunConfig& cfg,int index)
{
	webservice = new LidarWebService(cfg.service_port);
	DWORD threadId;
	int* arg = new int;
	*arg = index;
	CreateThread(NULL, 0, lidar_service, arg, 0, &threadId);
	cfg.thread_ID[2] = threadId;
}
//�رձ��ط���
void  CloseLocalService()
{

}

// �����״�������߳�
int openDev(RunConfig& cfg,int index)
{
	int fd = -1;//����������
	int res = -1;//���溯������ֵ
	if (strcmp(cfg.type, "uart") == 0|| strcmp(cfg.type, "vpc") == 0)
	{
		fd = (int)open_serial_port(cfg);
		if (fd <= 0)
		{
			return OPEN_UART_FD_FAILED;
		}
		//�����豸���
		cfg.fd = fd;
		MSG msg;
		PeekMessage(&msg, NULL, WM_USER, WM_USER, PM_NOREMOVE);
		HANDLE hStartEvent = ::CreateEvent(0, FALSE, FALSE, 0); //create thread start event
		if (hStartEvent == 0)
		{
			printf("create start event failed,errno:%d\n", ::GetLastError());
			return 1;
		}
		//�����ж����߳��Ƿ���������
		DWORD  threadId;
		if ((int)CreateThread(NULL, 0, lidar_thread_proc_uart, &cfg, 0, &threadId) < 0)
			return THREAD_CREATE_FAILED;

		cfg.hStartEvent = hStartEvent;
		//wait thread start event to avoid PostThreadMessage return errno:1444
		DWORD res = ::WaitForSingleObject(hStartEvent, 30000);
		if (res == WAIT_TIMEOUT)
		{
			INFO_PR("Lidar is open failed\n");
			return FAILTORUN;
		}
		CloseHandle(hStartEvent);
		cfg.hStartEvent = 0;
		INFO_PR("Lidar is open success\n");

		cfg.thread_ID[0] = GetCurrentThreadId();
		cfg.thread_ID[1] = threadId;
		//memcpy(g_cfg[index],&cfg,sizeof(RunConfig));
		g_cfg[index] = &cfg;
		return SUCCESS;
	}
	else if (strcmp(cfg.type, "udp") == 0)
	{
		fd = (int)open_socket_port(cfg);
		if (fd <= 0)
		{
			return OPEN_UDP_FD_FAILED;
		}
		//�����豸���
		cfg.fd = fd;
		MSG msg;
		PeekMessage(&msg, NULL, WM_USER, WM_USER, PM_NOREMOVE);
		HANDLE hStartEvent = ::CreateEvent(0, FALSE, FALSE, 0); //create thread start event
		if (hStartEvent == 0)
		{
			printf("create start event failed,errno:%d\n", ::GetLastError());
			return 1;
		}
		//�����ж����߳��Ƿ���������

		DWORD  threadId;
		if ((int)CreateThread(NULL, 0, lidar_thread_proc_udp, &cfg, 0, &threadId) < 0)
			return THREAD_CREATE_FAILED;

		cfg.hStartEvent = hStartEvent;
		//wait thread start event to avoid PostThreadMessage return errno:1444
		DWORD res = ::WaitForSingleObject(hStartEvent, 30000);
		if (res == WAIT_TIMEOUT)
		{
			INFO_PR("Lidar is open failed\n");
			return FAILTORUN;
		}
		CloseHandle(hStartEvent);
		cfg.hStartEvent = 0;

		INFO_PR("Lidar is open success\n");
		cfg.thread_ID[0] = GetCurrentThreadId();
		cfg.thread_ID[1] = threadId;
		g_cfg[index] = &cfg;
		//memcpy(g_cfg[index], &cfg, sizeof(RunConfig));
		return SUCCESS;

	}
	else
	{
		return ARG_ERROR_TYPE;
	}
}

int thread_set_run(RunConfig& cfg)
{
	DWORD  threadId;
	if (strcmp(cfg.type, "uart") == 0||strcmp(cfg.type, "vpc") == 0)
		return (int)CreateThread(NULL, 0, lidar_thread_proc_uart, &cfg, 0, &threadId);
	else if (strcmp(cfg.type, "udp") == 0)
		return (int)CreateThread(NULL, 0, lidar_thread_proc_udp, &cfg, 0, &threadId);
	else
		return ARG_ERROR_TYPE;

}

int getLidarData(long threadID, bool dataGet)
{

	MSG msg;
	int* cmd = new int;
	*cmd = dataGet;
	if (!PostThreadMessage(threadID, Print_Point_MSG, (WPARAM)cmd, GetCurrentThreadId()))
	{
		delete cmd;
		return MSG_POST_FAILED;
	}
	sleep(0.1);
	int index = SLEEP_SIZE;
	while (index--)
	{
		if (PeekMessage(&msg, NULL, Print_Point_MSG, Print_Point_MSG, PM_REMOVE))
		{
			switch (msg.message)
			{
			case Print_Point_MSG:
				INFO_PR("getLidarData recv MSG\n");
				return SUCCESS;
			}
		}
		sleep(1);
	}
	return GET_DEVPONIT_FAILED;
}
int GetDevInfo(long threadID, EEpromV101& data)
{
	MSG msg;
	if (!PostThreadMessage(threadID, GetDevInfo_MSG, NULL, GetCurrentThreadId()))
	{
		return MSG_POST_FAILED;
	}
	sleep(0.1);
	int index = SLEEP_SIZE;
	while (index--)
	{
		if (PeekMessage(&msg, NULL, GetDevInfo_MSG, GetDevInfo_MSG, PM_REMOVE))
		{
			switch (msg.message)
			{
			case GetDevInfo_MSG:
				EEpromV101* revdata = (EEpromV101*)msg.wParam;
				memcpy(&data, revdata, sizeof(EEpromV101));
				INFO_PR("GetDevInfo recv MSG\n");
				delete[] revdata;
				return SUCCESS;
			}
		}
		sleep(1);
	}
	return GET_DEVINFO_FAILED;
}

int SetDevInfo_extre(long threadID, DevData& data)
{
	MSG msg;
	DevData* cmd = new DevData;
	memcpy(cmd, &data, sizeof(DevData));

	if (!PostThreadMessage(threadID, SetDevInfo_MSG, (WPARAM)cmd, GetCurrentThreadId()))
	{
		delete cmd;
		return MSG_POST_FAILED;
	}
	sleep(0.1);
	int index = SLEEP_SIZE;
	while (index--)
	{
		if (PeekMessage(&msg, NULL, SetDevInfo_MSG, SetDevInfo_MSG, PM_REMOVE))
		{
			switch (msg.message)
			{
			case SetDevInfo_MSG:
				DevData* revdata = (DevData*)msg.wParam;
				INFO_PR("SetDevInfo_extre recv MSG\n");
				for (int i = 0; i < sizeof(revdata->result); i++)
				{
					if (revdata->result[i] == 'O' || revdata->result[i] == 'N')
					{
						memcpy(&data, revdata, sizeof(DevData));
						delete[] revdata;
						return SUCCESS;
					}

				}
				//˵��δ�����������߲�֧������
				delete  revdata;
				return SET_DEVINFO_FAILED;
			}
		}
		sleep(1);
	}
	return SET_DEVINFO_FAILED;
}

// ��ͣ�״���
int ControlDrv(long threadID, const char* data)
{
	MSG msg;
	char* cmd = new char[7];
	strcpy(cmd, data);
	if (!PostThreadMessage(threadID, ctrl_MSG, (WPARAM)cmd, GetCurrentThreadId()))
	{
		delete cmd;
		return MSG_POST_FAILED;
	}
	sleep(0.1);
	int index = SLEEP_SIZE;
	while (index--)
	{
		if (PeekMessage(&msg, NULL, ctrl_MSG, ctrl_MSG, PM_REMOVE))
		{
			switch (msg.message)
			{
			case ctrl_MSG:
				INFO_PR("ControlDrv recv MSG\n");
				return SUCCESS;
			}
		}
		sleep(1);
	}
	return SET_DEV_FAILED;
}
int GetLidarTimestamp(long threadID, bool dataGet)
{
	MSG msg;
	int* cmd = new int;
	*cmd = dataGet;
	if (!PostThreadMessage(threadID, Print_TimeStamp_MSG, (WPARAM)cmd, GetCurrentThreadId()))
	{
		return MSG_POST_FAILED;
	}
	::Sleep(100);
	int index = SLEEP_SIZE;
	while (index--)
	{
		if (PeekMessage(&msg, NULL, Print_TimeStamp_MSG, Print_TimeStamp_MSG, PM_REMOVE))
		{
			switch (msg.message)
			{
			case Print_TimeStamp_MSG:
				int* revdata = (int*)msg.wParam;
				int rev = *revdata;
				delete revdata;
				INFO_PR("GetLidarTimestamp recv MSG\n");
				if (rev)
					return SUCCESS;
				else
					return GET_DEVTIMESTAMP_FAILED;
			}
		}
		sleep(1);
	}

	return GET_DEVTIMESTAMP_FAILED;
}
int GetOnePoint_MSG(long threadID, PointData& data)
{
	MSG msg;
	if (!PostThreadMessage(threadID, Get_OnePoint_MSG, NULL, GetCurrentThreadId()))
	{
		return MSG_POST_FAILED;
	}
	//sleep(0.1);
	int index = SLEEP_SIZE;
	while (index--)
	{
		if (PeekMessage(&msg, NULL, Get_OnePoint_MSG, Get_OnePoint_MSG, PM_REMOVE))
		{
			switch (msg.message)
			{
			case Get_OnePoint_MSG:
				PointData* revdata = (PointData*)msg.wParam;
				memcpy(&data, revdata, sizeof(PointData));
				INFO_PR("GetOnePoint_MSG recv MSG\n");
				delete[] revdata;
				return SUCCESS;
			}
		}
		::Sleep(1000);
	}

	return GET_ONEPOINT_FAILED;
}
int GetAlarmZone(long threadID, RecvZoneDatas& data)
{
	MSG msg;
	RecvZoneDatas* cmd = new RecvZoneDatas;
	*cmd = data;
	if (!PostThreadMessage(threadID, Get_ZONE_MSG, NULL, GetCurrentThreadId()))
	{
		return MSG_POST_FAILED;
	}
	::Sleep(100);
	int index = SLEEP_SIZE;
	while (index)
	{
		if (PeekMessage(&msg, NULL, Get_ZONE_MSG, Get_ZONE_MSG, PM_REMOVE))
		{
			switch (msg.message)
			{
			case Get_ZONE_MSG:
				RecvZoneDatas* revdata = (RecvZoneDatas*)msg.wParam;
				memcpy(&data, revdata, sizeof(RecvZoneDatas));
				delete []revdata;
				INFO_PR("Get_ZONE_MSG recv MSG\n");
				if(data.result != 1 )
					return Get_ZONE_FAILED;
				else
					return SUCCESS;
			}
		}
		::Sleep(1000);
	}

	return Get_ZONE_FAILED;
}

int SetAlarmZone(long threadID, zones& data)
{
	MSG msg;
	zones* cmd = new zones;
	*cmd = data;
	if (!PostThreadMessage(threadID, Set_ZONE_MSG, (WPARAM)cmd, GetCurrentThreadId()))
	{
		return MSG_POST_FAILED;
	}
	::Sleep(100);
	int index = SLEEP_SIZE;
	while (index)
	{
		if (PeekMessage(&msg, NULL, Set_ZONE_MSG, Set_ZONE_MSG, PM_REMOVE))
		{
			switch (msg.message)
			{
			case Set_ZONE_MSG:
				int* revdata = (int*)msg.wParam;
				int rev = *revdata;
				delete revdata;
				INFO_PR("Set_ZONE_MSG recv MSG\n");
				if (rev == 1)
					return SUCCESS;
				else
					return Set_ZONE_FAILED;
			}
		}
		::Sleep(1000);
	}

	return Set_ZONE_FAILED;
}
#endif