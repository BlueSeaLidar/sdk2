#ifndef __STANDARD_INTERFACE_WIN32_H__
#define __STANDARD_INTERFACE_WIN32_H__

#include "standard_interface.h"
#include"service/LidarWebService.h"
#include <fstream>
#include <sstream>
RunConfig* g_cfg;
LidarWebService* webservice;
//Configuration file description: If you get linux in windows format, there will be an exception, please convert to unix first
bool read_config(const char* cfg_file_name, RunConfig& cfg)
{
	std::ifstream infile;
	infile.open(cfg_file_name);
	if (!infile.is_open())
		return false;

	std::string s,t;
	std::string lidar_ip_s, lidar_port_s, local_port_s;
	std::string unit_is_mm_s, with_confidence_s, resample_s;
	std::string with_deshadow_s, with_smooth_s, with_chk_s;
	std::string raw_bytes_s, rpm_s, output_scan_s, output_360_s;
	std::string from_zero_s, collect_angle;
	std::string output_file;
	std::string is_group_listener;
	std::string group_ip;
	std::string type;
	std::string baud_rate, port;
	std::string service_port, is_open_service;
	std::string alarm_msg;
	while (getline(infile, s))
	{
		std::string tmp;
		std::stringstream linestream(s);
		getline(linestream, tmp, ':');

		if (tmp == "type")
		{
			getline(linestream, type, ':');
			strcpy(cfg.type, type.c_str());
		}
		if (tmp == "baud_rate")
		{
			getline(linestream, baud_rate, ':');
			cfg.baud_rate = atoi(baud_rate.c_str());
		}
		if (tmp == "port")
		{
			getline(linestream, port, ':');
			strcpy(cfg.port, port.c_str());
		}
		if (tmp == "lidar_ip")
		{
			getline(linestream, lidar_ip_s, ':');
			strcpy(cfg.lidar_ip, lidar_ip_s.c_str());
		}
		else if (tmp == "lidar_port")
		{
			getline(linestream, lidar_port_s, ':');
			cfg.lidar_port = atoi(lidar_port_s.c_str());
		}
		else if (tmp == "local_port")
		{
			getline(linestream, local_port_s, ':');
			cfg.local_port = atoi(local_port_s.c_str());
		}
		else if (tmp == "with_confidence")
		{
			getline(linestream, with_confidence_s, ':');
			cfg.with_confidence = atoi(with_confidence_s.c_str());
		}
		else if (tmp == "raw_bytes")
		{
			getline(linestream, raw_bytes_s, ':');
			cfg.data_bytes = atoi(raw_bytes_s.c_str());
		}
		else if (tmp == "unit_is_mm")
		{
			getline(linestream, unit_is_mm_s, ':');
			cfg.unit_is_mm = atoi(unit_is_mm_s.c_str());
		}
		else if (tmp == "with_chk")
		{
			getline(linestream, with_chk_s, ':');
			cfg.with_chk = atoi(with_chk_s.c_str());
		}
		else if (tmp == "with_smooth")
		{
			getline(linestream, with_smooth_s, ':');
			cfg.with_smooth = atoi(with_smooth_s.c_str());
		}
		else if (tmp == "with_deshadow")
		{
			getline(linestream, with_deshadow_s, ':');
			cfg.with_deshadow = atoi(with_deshadow_s.c_str());
		}
		else if (tmp == "resample")
		{
			getline(linestream, resample_s, ':');
			cfg.resample = atoi(resample_s.c_str());
		}
		else if (tmp == "rpm")
		{
			getline(linestream, rpm_s, ':');
			cfg.rpm = atoi(rpm_s.c_str());
		}
		else if (tmp == "output_scan")
		{
			getline(linestream, output_scan_s, ':');
			cfg.output_scan = atoi(output_scan_s.c_str());
		}
		else if (tmp == "output_360")
		{
			getline(linestream, output_360_s, ':');
			cfg.output_360 = atoi(output_360_s.c_str());
		}
		else if (tmp == "from_zero")
		{
			getline(linestream, from_zero_s, ':');
			cfg.from_zero = atoi(from_zero_s.c_str());
		}
		else if (tmp == "collect_angle")
		{
			getline(linestream, collect_angle, ':');
			cfg.collect_angle = atoi(collect_angle.c_str());
		}
		else if (tmp == "output_file")
		{
			getline(linestream, output_file, ':');
			strcpy(cfg.output_file, output_file.c_str());
		}
		else if (tmp == "is_group_listener")
		{
			getline(linestream, is_group_listener, ':');
			cfg.is_group_listener = atoi(is_group_listener.c_str());
		}
		else if (tmp == "group_ip")
		{
			getline(linestream, group_ip, ':');
			strcpy(cfg.group_ip, group_ip.c_str());
		}
		else if (tmp == "service_port")
		{
			getline(linestream, service_port, ':');
			cfg.service_port = atoi(service_port.c_str());;
		}
		else if (tmp == "is_open_service")
		{
			getline(linestream, is_open_service, ':');
			cfg.is_open_service = atoi(is_open_service.c_str());
		}
		else if (tmp == "alarm_msg")
		{
			getline(linestream, alarm_msg, ':');
			cfg.alarm_msg = atoi(alarm_msg.c_str());
		}
		else if (tmp == "shadow_filter.enable")
		{
			getline(linestream, t, ':');
			cfg.shadows_filter.enable = atoi(t.c_str());
		}
		else if (tmp == "shadow_filter.max_range")
		{
			getline(linestream, t, ':');
			cfg.shadows_filter.max_range = atoi(t.c_str());
		}
		else if (tmp == "shadow_filter.min_angle")
		{
			getline(linestream, t, ':');
			cfg.shadows_filter.min_angle = atoi(t.c_str());
		}
		else if (tmp == "shadow_filter.max_angle")
		{
			getline(linestream, t, ':');
			cfg.shadows_filter.max_angle = atoi(t.c_str());
		}
		else if (tmp == "shadow_filter.window")
		{
			getline(linestream, t, ':');
			cfg.shadows_filter.window = atoi(t.c_str());
		}
		else if (tmp == "median_filter.enable")
		{
			getline(linestream, t, ':');
			cfg.median_filter.enable = atoi(t.c_str());
		}
		else if (tmp == "median_filter.window")
		{
			getline(linestream, t, ':');
			cfg.median_filter.window = atoi(t.c_str());
		}
	}
	return true;
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
	webservice->OpenLocalService();
	return SUCCESS;
}
//打开本地服务
void  OpenLocalService(RunConfig& cfg)
{
	webservice = new LidarWebService(cfg.service_port);
	DWORD threadId;
	CreateThread(NULL, 0, lidar_service, 0, 0, &threadId);
	cfg.thread_ID[2] = threadId;
}
//关闭本地服务
void  CloseLocalService()
{

}
// 连接雷达，启动子线程
int openDev(RunConfig& cfg)
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
		MSG msg;
		PeekMessage(&msg, NULL, WM_USER, WM_USER, PM_NOREMOVE);
		HANDLE hStartEvent = ::CreateEvent(0, FALSE, FALSE, 0); //create thread start event
		if (hStartEvent == 0)
		{
			printf("create start event failed,errno:%d\n", ::GetLastError());
			return 1;
		}
		//用来判定子线程是否正常工作
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
		g_cfg = &cfg;
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
		MSG msg;
		PeekMessage(&msg, NULL, WM_USER, WM_USER, PM_NOREMOVE);
		HANDLE hStartEvent = ::CreateEvent(0, FALSE, FALSE, 0); //create thread start event
		if (hStartEvent == 0)
		{
			printf("create start event failed,errno:%d\n", ::GetLastError());
			return 1;
		}
		//用来判定子线程是否正常工作

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
		g_cfg = &cfg;
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
				//说明未添加设置项或者不支持设置
				delete  revdata;
				return SET_DEVINFO_FAILED;
			}
		}
		sleep(1);
	}
	return SET_DEVINFO_FAILED;
}

// 启停雷达测距
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