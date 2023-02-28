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


//��������  ȫ�ֱ���
struct RunConfig
{
	char type[16];			//CN:��������							EN:transmission type	for example:"uart"   or   "udp"  "vpc"
	char port[16];			//CN:��������							EN:uart name
	int baud_rate;			//CN:������								EN:baud rate
	int lidar_port;			//CN:Ŀ��(�״�)�˿ں�					EN:target(Lidar) port
	char lidar_ip[16];		//CN:�״��IP��ַ						EN:Lidar IP address
	int local_port;			//CN:���ض˿ں�						    EN:local port
	int unit_is_mm;			//CN:���ݵ�λ(1.mm 0 cm)                EN:unit��1.mm  0.cm��
	int with_confidence;	//CN:�Ƿ��ǿ�ȣ� 0��  1��   			EN:With or without strength,0 false  1 true
	int resample;			//CN:�ֱ���								EN:Resolution
	int with_deshadow;		//CN:ȥ�ϵ�(0���رգ�1������)			EN:go to drag point (0: off, 1: on)
	int with_smooth;		//CN:����ƽ��(0���رգ� 1������)		EN:Data smoothing (0: off, 1: on)
	int with_chk;			//CN:����У��(0:�ر� 1:��)			EN:Data check (0: close 1: open)
	int data_bytes;			//CN:���ݴ��ģʽ��2��2�ֽڣ� 3��3�ֽ�  EN:Data packing mode, 2: 2 bytes, 3: 3 bytes Special instructions: 2 for the old model, 3 for the new model
	int rpm;				//CN:ת��								EN:Rotating speed
	int output_scan;		//CN:�Ƿ��ӡ (0������ӡ   1����ӡ )	EN:Whether to print (0: not print 1: print)
	int output_360;			//CN:���δ�ӡ(0:�������δ�ӡ  1����ɵ�)EN:Fan printing (0: Partial fan printing 1: Completed)
	int from_zero;			//CN:�Ƿ��0�Ƚǿ�ʼͳ��				EN:Whether to start counting from 0 degree angle
	int collect_angle;		//CN:ͳ����ʼ�Ƕȵ�����ֵ				EN:Correction value of statistical starting angle
	char output_file[256];	//CN:���ݱ����ļ�����·��				EN:Data save file absolute path
	int is_group_listener;	//CN:0����ģʽ   1����ģʽ   2����ģʽ	EN:0 Normal mode 1 Listening mode 2 Sending mode
	char group_ip[16];		//CN:�鲥IP								EN:Multicast IP
	// control
	bool should_quit;		//CN:�˳���־λ							EN:quit flag	
	int alarm_msg;			//CN:�Ƿ�򿪷�������					EN:Whether to turn on the zone alarm
	int error_circle;		//CN:��ⳤ��Ϊ0��Ȧ��					EN:Detect the number of turns with a length of 0
	float error_scale;		//CN:��ⳤ��Ϊ0�ı���					EN:Detect the Scale of turns with a length of 0
	char version[64];		//CN:Ӳ���汾��							EN:Hardware version
#ifdef __linux
	int msgid;//��Ϣ����ID
	pthread_t thread;		//CN:���״�ͨ�����߳�					EN:Sub-thread: responsible for communicating with lidar
#elif  _WIN32
	HANDLE thread;
	HANDLE hStartEvent;
#endif 
	int fd;//���
	printfMsg  callback;	//CN:��Ϣ��ӡ�ص�����					EN:Information printing callback function
	PointData  pointdata;	//CN:��֡��������						EN:Single frame point cloud data
	LidarMsgHdr zone;		//CN:������Ϣ����						EN:Alarm information data
	long thread_ID[3];		//CN:���̣߳��������̣߳��������߳�ID	EN:main thread, data sub-thread, service sub-thread ID
	int service_port;		//CN:���ط������ö˿�					EN:Local service enable port
	int is_open_service;	//CN:�Ƿ����ñ��ط���					EN:Enable local service
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


