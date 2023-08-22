#ifndef __LIDARDATAPROCESS_H__
#define  __LIDARDATAPROCESS_H__
#include"Global.h"
#include"service/LidarWebService.h"

#ifdef _WIN32

#elif __unix__ 
#include <sys/time.h>
#endif
typedef void (*printfMsg)(int, void*,int);

enum ACTION
{
	OFFLINE=0,
	ONLINE,
	RUN,
	CONTROL,
	GETALLPARAMS,
	SETPARAM,
	READZONE,
	WRITEZONE,
	FINISH
};
enum STATE
{
	INIT=0,
	WORK,
	WORK_AND_WEB,
	STOP_ALL
};

struct Responsive
{
	int mode;//0x0043  0x0053  0x4753
	int send_len;
	char send_cmd[1024];
	short rand;
	long timestamp;
};

struct RunScript
{
	//connect
	char type[16];//"uart"   or   "udp"  "vpc"
	char connectArg[16];//ip/com
	int connectArg2;//port/baud
	int local_port;
	//data
	char logPath[256];
    int data_bytes;
	int from_zero;
	int with_chk;			//CN:数据校验(0:关闭 1:打开)			EN:Data check (0: close 1: open)
	int error_circle;		//CN:检测长度为0的圈数					EN:Detect the number of turns with a length of 0
	double error_scale;		//CN:检测长度为0的比例					EN:Detect the Scale of turns with a length of 0
	int output_360;			//CN:扇形打印(0:部分扇形打印  1：完成的)EN:Fan printing (0: Partial fan printing 1: Completed)
	int service_port;		//CN:本地服务启用端口					EN:Local service enable port
	int is_open_service;	//CN:是否启用本地服务					EN:Enable local service
	//udp
	int is_group_listener;	//CN:0正常模式   1监听模式   2发送模式	EN:0 Normal mode 1 Listening mode 2 Sending mode
	char group_ip[16];		//CN:组播IP	

	// E120 scan filter
	ShadowsFilterParam shadows_filter;
	MedianFilterParam median_filter;
	//get
	int uuid;
	int model;
	int version;

	//set common
	int rpm;
	int resample_res;

	int with_smooth;
	int with_deshadow;

	int with_start;
	//set uart
	int with_confidence;
	int unit_is_mm;
	//set udp  vpc
	int alarm_msg;
	int direction;
	int ats;  //1udp  2vpc
};

//运行配置 
struct RunConfig
{	
	int ID;
	STATE state;	//-1 stop all 0 init   1 run work thread  2 run work and web thread 

	unsigned long thread_data;//DWORD == pthread_t
	unsigned long thread_web;
#endif 
	LidarWebService *webservice;
	int fd;//句柄
	printfMsg  callback;	//CN:信息打印回调函数					EN:Information printing callback function
	UserData userdata;//当前帧
	LidarMsgHdr zonemsg;//当前的防区信息
	EEpromV101 eepromv101;//全局变量参数
	char hardwareVersion[32];//硬件版本号
	
	ACTION action;//0 无操作   1.启停控制指令  2.获取设备全局参数  3.设置设备参数  4 读取防区   5 设置防区
	//当前传入的指令
	int mode;
	int send_len;
	char send_cmd[1024];
	int recv_len;
	char recv_cmd[1024];

	RunScript  runscript;
	
};

 int setup_lidar_uart(int fd_uart, RunScript* arg, EEpromV101* eepromv101, char* version);
 int setup_lidar_vpc(int hCom, RunScript* arg);

 bool readConfig(const char* cfg_file_name, RunScript& cfg);
 bool checkPointsLengthZero(UserData *tmp, float scale);
 void* lidar_thread_proc_udp(void* param);
 void* lidar_thread_proc_uart(void* param);


