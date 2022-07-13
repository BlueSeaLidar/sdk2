#pragma once
#ifdef _WIN32
#include<afx.h>
#include<Ws2tcpip.h>
#include <atlbase.h>
#include <afxcoll.h>
#pragma comment(lib,"wsock32.lib")
#pragma comment(lib,"setupapi.lib")
#include <winioctl.h>
#include <setupapi.h>
#endif

#ifdef __unix__
#include <sys/time.h>
#include <pthread.h>
#endif


#include<stdint.h>
#include<vector>

#define CHECKSERVICE  6789  //检测本地雷达列表端口号
struct DevInfo
{
	//转速	2个字节
	unsigned short rpm;
	//启动脉冲个数	2个字节
	unsigned short pulse;
	//保留	4个字节
	char sign[4];
	//版本号	2个字节
	unsigned short version;
	//ip地址	4个 字节
	unsigned char ip[4];
	//子网掩码	4个字节
	unsigned char mask[4];
	//网关地址	4个字节
	unsigned char gateway[4];
	//默认目标IP	4个字节
	unsigned char remote_ip[4];
	//默认目标udp端口号	2个字节
	unsigned short remote_udp;
	//默认UDP对外服务端口号	2个字节
	unsigned short port;
	//物体分辨率	1个字节
	unsigned char fir;
	//偏置	6个字节
	char zero_offset[6];
	//机器序号	20个字节
	char dev_sn[20];
	//机器类型	11个字节
	char dev_type[11];
	//IO类型选择	1个字节
	char io_type;
	//响应圈数	1个字节 
	unsigned char cir;
	//IO功能引脚配置	10个字节
	unsigned char io_mux[10];
};
struct DevInfo2
{
	// 标签	4个字节
	char sign[4];
	// 机器序号	20个字节
	char dev_sn[20];
	// 机器类型	11个字节
	char dev_type[12];
	//版本号	2个字节
	unsigned short version;
	// ip地址	4个 字节
	unsigned char ip[4];
	// 子网掩码	4个字节
	unsigned char mask[4];
	// 网关地址	4个字节
	unsigned char gateway[4];
	// 默认目标IP	4个字节
	unsigned char remote_ip[4];
	//默认目标udp端口号	2个字节
	unsigned short remote_udp;
	// 默认UDP对外服务端口号	2个字节
	unsigned short port;
	//保留	2个字节
	char reserver[2];
};


struct DevInfoV101
{
	char sign[4];  // must be "LiDA"
	uint32_t proto_version; // Ð­Òé°æ±¾ V101
	uint32_t timestamp[2];// Ê±¼ä´Á
	char dev_sn[20];
	char dev_type[16];
	uint32_t version;
	uint32_t dev_id; //Éè±¸±àºÅ
	uint8_t ip[4]; //Éè±¸µØÖ·
	uint8_t mask[4]; //
	uint8_t gateway[4];
	uint8_t remote_ip[4]; // ·þÎñÆ÷µØÖ·

	uint16_t remote_udp; //·þÎñÆ÷¶Ë¿Ú
	uint16_t port; // lidar service udp port
	uint16_t status;// Éè±¸×´Ì¬
	uint16_t rpm; // µ±Ç°×ªËÙ

	uint16_t freq; // µ±Ç°×ªËÙ
	uint8_t ranger_version[2];
	uint16_t CpuTemp;
	uint16_t InputVolt;
	uint8_t alarm[16]; // ±¨¾¯ÐÅÏ¢
	uint32_t crc;
};

//#pragma pack(pop)

enum ConnType
{
	TYPE_COM,
	TYPE_UDP_V1,
	TYPE_UDP_V2,
	TYPE_UDP_V101
};

struct DevConnInfo
{
	ConnType type;
	char com_port[128];
	int com_speed;
	char conn_ip[32];
	int conn_port;
	char timeStr[16];  //HH-MM-SS
	union {
		DevInfo v1;// info;
		DevInfoV101 v101;// info101;
		DevInfo2 v2;
	} info;
};

class LidarCheckService
{
public:
	LidarCheckService(int port);
	~LidarCheckService();

	void openService();
	void closeService();
	void clearLidarsCache();
	std::vector<DevConnInfo> getLidarsList();
	static void getTime_HMS(char*data);
	void uartDevInfo();
private:
#ifdef _WIN32
	static bool RegQueryValueString(_In_ ATL::CRegKey& key, _In_ LPCTSTR lpValueName, _In_ ULONG nBytes, _Inout_ char* sValue);
	static bool QueryDeviceDescription(_In_ HDEVINFO hDevInfoSet, _In_ SP_DEVINFO_DATA& devInfo, _In_ DWORD dwSize, _Inout_ char* sFriendlyName);
	static bool enum_ports(const GUID& guid, CMapStringToString& ports);
	static HANDLE OpenPort(const char* name, int speed);
	static BOOL GetDevInfoByCom(const char* port_str, int speed);	
	DWORD m_dwThreadUdp  {0};
#endif
#ifdef __unix__

	pthread_t  m_dwThreadUdp {0};
#endif
};
#ifdef _WIN32
DWORD WINAPI UDPThreadProc(void* p);
#endif

#ifdef __unix__
void* UDPThreadProc(void* p);
#endif
//static void getTime_HMS(char*data);
void uptodate(DevConnInfo* data);

