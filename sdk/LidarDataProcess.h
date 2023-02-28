#ifndef __LIDARDATAPROCESS_H__
#define  __LIDARDATAPROCESS_H__
#include"Global.h"
#include"service/ZoneAlarm.h"

#ifdef _WIN32

#elif __unix__ 
#include <sys/time.h>
#endif
typedef void (*printfMsg)(int, void*);
typedef void (*send_cmd_uart_ptr)(int hCom, int mode, int sn, int len, const char* cmd);
typedef void (*send_cmd_udp_ptr)(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf, bool isSaveLog);


//运行配置  全局变量
struct RunConfig
{
	char type[16];			//CN:传输类型							EN:transmission type	for example:"uart"   or   "udp"  "vpc"
	char port[16];			//CN:串口名称							EN:uart name
	int baud_rate;			//CN:波特率								EN:baud rate
	int lidar_port;			//CN:目标(雷达)端口号					EN:target(Lidar) port
	char lidar_ip[16];		//CN:雷达的IP地址						EN:Lidar IP address
	int local_port;			//CN:本地端口号						    EN:local port
	int unit_is_mm;			//CN:数据单位(1.mm 0 cm)                EN:unit（1.mm  0.cm）
	int with_confidence;	//CN:是否带强度， 0否  1是   			EN:With or without strength,0 false  1 true
	int resample;			//CN:分辨率								EN:Resolution
	int with_deshadow;		//CN:去拖点(0：关闭，1：开启)			EN:go to drag point (0: off, 1: on)
	int with_smooth;		//CN:数据平滑(0：关闭， 1：开启)		EN:Data smoothing (0: off, 1: on)
	int with_chk;			//CN:数据校验(0:关闭 1:打开)			EN:Data check (0: close 1: open)
	int data_bytes;			//CN:数据打包模式，2：2字节， 3：3字节  EN:Data packing mode, 2: 2 bytes, 3: 3 bytes Special instructions: 2 for the old model, 3 for the new model
	int rpm;				//CN:转速								EN:Rotating speed
	int output_scan;		//CN:是否打印 (0：不打印   1：打印 )	EN:Whether to print (0: not print 1: print)
	int output_360;			//CN:扇形打印(0:部分扇形打印  1：完成的)EN:Fan printing (0: Partial fan printing 1: Completed)
	int from_zero;			//CN:是否从0度角开始统计				EN:Whether to start counting from 0 degree angle
	int collect_angle;		//CN:统计起始角度的修正值				EN:Correction value of statistical starting angle
	char output_file[256];	//CN:数据保存文件绝对路径				EN:Data save file absolute path
	int is_group_listener;	//CN:0正常模式   1监听模式   2发送模式	EN:0 Normal mode 1 Listening mode 2 Sending mode
	char group_ip[16];		//CN:组播IP								EN:Multicast IP
	// control
	bool should_quit;		//CN:退出标志位							EN:quit flag	
	int alarm_msg;			//CN:是否打开防区报警					EN:Whether to turn on the zone alarm
	int error_circle;		//CN:检测长度为0的圈数					EN:Detect the number of turns with a length of 0
	float error_scale;		//CN:检测长度为0的比例					EN:Detect the Scale of turns with a length of 0
	char version[64];		//CN:硬件版本号							EN:Hardware version
#ifdef __linux
	int msgid;//消息队列ID
	pthread_t thread;		//CN:和雷达通信子线程					EN:Sub-thread: responsible for communicating with lidar
#elif  _WIN32
	HANDLE thread;
	HANDLE hStartEvent;
#endif 
	int fd;//句柄
	printfMsg  callback;	//CN:信息打印回调函数					EN:Information printing callback function
	PointData  pointdata;	//CN:单帧点云数据						EN:Single frame point cloud data
	LidarMsgHdr zone;		//CN:报警信息数据						EN:Alarm information data
	long thread_ID[3];		//CN:主线程，数据子线程，服务子线程ID	EN:main thread, data sub-thread, service sub-thread ID
	int service_port;		//CN:本地服务启用端口					EN:Local service enable port
	int is_open_service;	//CN:是否启用本地服务					EN:Enable local service
	// E100 scan filter
	ShadowsFilterParam shadows_filter;
	MedianFilterParam median_filter;
};

#ifdef _WIN32
int open_socket_port(RunConfig& cfg);
HANDLE open_serial_port(RunConfig& cfg);

 int setup_lidar_vpc(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char* version);
 int setup_lidar_uart(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char* version);
 bool uart_talk(HANDLE hCom, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);
 bool uart_talk2(HANDLE hCom, int mode, int sn, int len, const char* cmd, int nfetch, char* fetch);
 bool uart_talk3(HANDLE hCom, int mode, int sn, int len, const char* cmd, int result_len, void* result);
DWORD  WINAPI lidar_thread_proc_udp(void* param);
DWORD  WINAPI  lidar_thread_proc_uart(void* param);
#endif

#ifdef __unix__
extern "C"  int change_baud(int fd, int baud);
int open_serial_port(RunConfig &cfg);
int open_socket_port(RunConfig &cfg);
bool uart_talk(int fd, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);
bool uart_talk2(int hCom, int mode, int sn, int len, const char* cmd, int nfetch, char* fetch);
bool uart_talk3(int  hCom, int mode, int sn, int len, const char* cmd, int result_len,void *result);
void send_cmd_uart(int fd, int mode, int sn, int len, const char* cmd);
int strip(const char *s, char *buf);
void *lidar_thread_proc_udp(void* param);
void *lidar_thread_proc_uart(void *param);

int setup_lidar_vpc(int hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char *version);
int setup_lidar_uart(int fd_uart, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char *version);
#endif



 bool setup_lidar_udp(int fd_udp, const char* ip, int port, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, int should_post, char* version);
 int setup_lidar_extre(std::string type, int fd_udp, const char* ip, int port, DevData& data);
 bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);
 bool udp_talk_S_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result);
 bool udp_talk_GS_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result);
 void send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf, bool savelog);

 bool readConfig(const char* cfg_file_name, RunConfig& cfg);
 bool checkPointsLengthZero(PointData tmp, float scale);
#endif


