#pragma once

#include"Global.h"

class LidarCheckService
{
public:
	LidarCheckService();
	~LidarCheckService();

	void run();
	void stop();
	void clear();
	std::vector<DevConnInfo> getLidarsList();
	static void getTime_HMS(char*data);
	void uartDevInfo();
private:
	unsigned long m_thread_heart;
	bool m_close_service;

};
void* thread_heart(void* p);

//static void getTime_HMS(char*data);
void uptodate(DevConnInfo data);

