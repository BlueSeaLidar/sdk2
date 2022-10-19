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
	void  OpenLocalService();
	void  CloseLocalService();

private:
	
	int m_port;

};
static void fn(struct mg_connection* c, int ev, void* ev_data, void* fn_data);
static char* jsonValue(const char* result, const char* message, cJSON* array);
static void EEpromV101ToStr(EEpromV101* eepromv101, char* version, char* result);
static void StringReplace(std::string& strBase, std::string strSrc, std::string strDes);
static void DevDataToStr(DevData* devdata, int index, char* value);