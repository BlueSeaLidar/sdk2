#pragma once
#include<iostream>
#include"data.h"
extern "C"
{
	#include"third_party/cJson/cJSON.h"
}

class LidarWebService
{
public:
	LidarWebService(int port);
	~LidarWebService();
	void  run(int lidarID);
	void  stop();

private:
	
	int m_port;

};

