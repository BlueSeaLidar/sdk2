#pragma once

#include"Global.h"

class LidarCheckService
{
public:
	LidarCheckService();
	~LidarCheckService();

	void openService();
	void closeService();
	void clearLidarsCache();
	std::vector<DevConnInfo> getLidarsList();
	static void getTime_HMS(char*data);
	void uartDevInfo();
private:
	unsigned long m_thread_heart{ 0 };
	bool m_close_service;

};
void* lidar_heart(void* p);

//static void getTime_HMS(char*data);
void uptodate(DevConnInfo data);

