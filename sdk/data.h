#ifndef _LIDAR_DATA
#define  _LIDAR_DATA

/**

 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved

 * File name:      data.h

 * Author:  	   *
      
 * Version:        1.0  
    
 * Date:		   2022.3.25

 * Description:    Data storage structure and data parsing capabilities for all devices

 */

#ifdef _WIN32
#include<Windows.h>
#define GetDevInfo_MSG WM_USER+100
#define SetDevInfo_MSG WM_USER+101
#define ctrl_MSG WM_USER+102
#define Print_Point_MSG WM_USER+103
#define Print_TimeStamp_MSG WM_USER+104
#define Get_OnePoint_MSG WM_USER+105
#elif __linux
#include <unistd.h>
enum msg_queue
{
	GetDevInfo_MSG=0,
	SetDevInfo_MSG,
	ctrl_MSG,
	Print_Point_MSG,
	Print_TimeStamp_MSG
};
struct CMD
{
	msg_queue type2;
	char str[1024];
};

typedef struct {
    long type;
	CMD  cmd;
}USER_MSG;

#endif

#include <stdint.h>
#include <string>
#include <stdlib.h>
#include"third_party/mongoose/mongoose.h"


#define PI 3.1415926535898

#define HDR_SIZE 6
#define HDR2_SIZE 8
#define HDR3_SIZE 16 
#define HDR7_SIZE 28 
#define HDR99_SIZE 32 
#define MAX_POINTS 500

#define BUF_SIZE 8*1024

#define SLEEP_SIZE 5
#define USER_SIZE  10
//日志打印开关   debug  info
#define _DEBUG_  1
#define _INFO_   0
#if  _DEBUG_ 
#define DEBUG_PR(...) printf(__VA_ARGS__)
#else
#define DEBUG_PR(...)
#endif

#if  _INFO_ 
#define INFO_PR(...) printf(__VA_ARGS__)
#else
#define INFO_PR(...)
#endif


//CN：心跳检测包 EN：Heartbeat detection package
struct KeepAlive {
	uint32_t world_clock;//CN:时间戳  EN:timestamp
	uint32_t mcu_hz;
	uint32_t arrive;
	uint32_t delay;//延时
	uint32_t reserved[4];
};
struct DataPoint
{
	float angle;				//CN:弧度		EN:radian

	float distance;				//CN:距离(米)	EN:distance(Meter)

	unsigned char confidence;	//CN:强度		EN:strength
};
//客户使用的雷达实时数据
struct PointData
{
	unsigned short N;			//CN:扇区内测距点数								EN:The number of ranging points in the sector
	DataPoint points[3000];//CN:扫描点的具体信息(具体初始化个数由N决定)	EN:The specific information of the scanning point (the specific initialization number is determined by N)
	uint32_t ts[2];				//CN:时间戳(秒和微秒)							EN:timestamps(Second and microseconds )
};
typedef void (*printfMsg)(int,void *);
//运行配置
struct RunConfig 
{
	// paramters
	char type[16];//"uart"   or   "udp"
    char port[16];//端口名称
    int baud_rate;
    int local_port;       //CN:端口名称										 EN:port name
	int unit_is_mm;		  //CN:波特率										 EN:baud rate
	int with_confidence;  //CN:数据打包模式，2：2字节， 3：3字节   			  EN:Data packing mode, 2: 2 bytes, 3: 3 bytes Special instructions: 2 for the old model, 3 for the new model
	int resample;		  //CN:分辨率										EN:Resolution
	int with_deshadow;	  //CN:去拖点(0：关闭，1：开启)						 EN:go to drag point (0: off, 1: on)
	int with_smooth;	  //CN:数据平滑(0：关闭， 1：开启)					 EN:Data smoothing (0: off, 1: on)
	int with_chk;		  //CN:数据校验(0:关闭 1:打开)						 EN:Data check (0: close 1: open)
	int data_bytes;		  //CN:数据打包模式，2：2字节， 3：3字节   			  EN:Data packing mode, 2: 2 bytes, 3: 3 bytes Special instructions: 2 for the old model, 3 for the new model
	int rpm;			  //CN:转速											EN:Rotating speed
	int output_scan;	  //CN:是否打印 (0：不打印   1：打印 )				 EN:Whether to print (0: not print 1: print)
	int output_360;		  //CN:扇形打印(0:完成的  1：部分扇形打印)			 EN:Fan printing (0: Completed 1: Partial fan printing)
	int from_zero;		  //CN:角度显示坐标   0  -180°-180°   1  0°到360度  EN:Angle display coordinates 0 -180°-180° 1 0° to 360°
	char output_file[256];//CN:数据保存文件绝对路径							EN:Data save file absolute path
	char lidar_ip[256];	  //CN:雷达的IP地址									EN:Radar IP address
	int lidar_port;		  //CN:雷达的端口号									EN:Radar  port
	int is_group_listener;//0正常模式   1监听模式   2发送模式
	char group_ip[16];//组播IP
	// control
	bool should_quit;	  //CN:退出标志位									EN:quit flag	
	char version[16];//硬件版本号
#ifdef __linux
	int msgid;//消息队列ID
	pthread_t thread;	  //CN:和雷达通信子线程								EN:Sub-thread: responsible for communicating with radar
#elif  _WIN32
	HANDLE thread;
	HANDLE hStartEvent;
#endif 
	int fd;//句柄
	printfMsg  callback;//信息打印函数指针
	PointData  pointdata;//单帧点数数据
	long thread_ID[3];//主线程，数据子线程，服务子线程
	int service_port;//本地服务启用端口
	int is_open_service;//是否启用本地服务
	//LidarCheckService  *checkservice;//查询本地的雷达列表
	//LidarWebService* webservice;//web服务
};


/*
* RawDataHdr
* CN:代表不同的数据存储格式，每个设备仅使用一种格式，但是不同的设备可能使用同一种格式
* EN:Represents different data storage formats, each device uses only one format, but different devices may use the same format
*/

struct RawDataHdr
{
	unsigned short code; //CN:帧头				EN:data frame header
	unsigned short N;    //CN:扇区内测距点数	EN:The number of ranging points in the sector
	unsigned short angle;//CN:对应测距角度		EN:Corresponding ranging angle
};

struct RawDataHdr2
{
	unsigned short code;	//CN:帧头				EN:data frame header
	unsigned short N;		//CN:扇区内测距点数		EN:The number of ranging points in the sector
	unsigned short angle;	//CN:对应测距角度		EN:Corresponding ranging angle
	unsigned short span;	//CN:扇区个数			EN:Number of sectors
};

struct RawDataHdr3
{
	unsigned short code;	//CN:帧头				EN:data frame header
	unsigned short N;		//CN:扇区内测距点数		EN:The number of ranging points in the sector
	unsigned short angle;	//CN:对应测距角度		EN:Corresponding ranging angle
	unsigned short span;	//CN:扇区个数			EN:Number of sectors
	unsigned short fbase;	//CN:扇区起始偏差		EN:Sector start offset
	unsigned short first;	//CN:第一个点角度		EN:first point angle
	unsigned short last;	//CN:最后一个点角度		EN:last point angle
	unsigned short fend;	//CN:扇区终止偏差		EN:Sector end offset
};


struct RawDataHdr7 {
	uint16_t code;		//CN:帧头						EN:data frame header
	uint16_t N;			//CN:扇区内这个分包的测距点数	EN:The number of ranging points for this subpacket in the sector
	uint16_t whole_fan;	//CN:扇区内总测距点数			EN:The total number of ranging points in the sector
	uint16_t ofset;		//CN:扇区偏移量					EN:sector offset
	uint32_t beg_ang;	//CN:扇区起始角度				EN:Sector start angle
	uint32_t end_ang;	//CN:扇区终止角度				EN:Sector end angle
	uint32_t flags;		//CN:状态开关标识				EN:status switch flag
	uint32_t timestamp;	//CN:时间戳						EN:timestamp		(other:The data timestamps of different sectors in a frame are consistent)
	uint32_t dev_id;	//CN:设备号						EN:device ID
};


struct FanSegment
{
	RawDataHdr7 hdr;			//CN：Hdr7结构体		EN：Hdr7structure 

	uint16_t dist[MAX_POINTS];	//CN:距离				EN:distance
	uint16_t angle[MAX_POINTS]; //CN:角度				EN:angle
	uint8_t energy[MAX_POINTS]; //CN:能量强度			EN:energy intensity

	struct FanSegment* next;	// CN:下个扇区指针		EN:next sector pointer 
};

struct RawDataHdr99 {
	uint16_t code;			//CN:帧头						EN:data frame header
	uint16_t N;				//CN:扇区内这个分包的测距点数	EN:The number of ranging points for this subpacket in the sector
	uint16_t from;			//CN:扇区起始角度				EN:Sector start angle
	uint16_t total;			//CN:扇区角度					EN:Total Sector Angle
	uint32_t flags;			//CN:状态开关标识				EN:status switch flag
	uint32_t timestamp;		//CN:时间戳						EN:timestamp 
	uint32_t dev_no;		//CN:设备号						EN:device ID
	uint32_t reserved[3];	//CN:保留位						EN:reserved
};



struct RawData
{
	unsigned short code;		//CN:帧头										EN:data frame header
	unsigned short N;			//CN:扇区内测距点数								EN:The number of ranging points in the sector
	unsigned short angle;		//CN:当前扇区累加的角度							EN:The accumulated angle of the current sector 
	unsigned short span;		//CN:当前扇区的角度								EN:The angle of the current sector
	unsigned short fbase;		//CN:扇区起始偏差								EN:Sector start offset
	unsigned short first;		//CN:第一个点角度								EN:first point angle
	unsigned short last;		//CN:最后一个点角度								EN:last point angle
	unsigned short fend;		//CN:扇区终止偏差								EN:Sector end offset
	// short ros_angle;	// 0.1 degree
	DataPoint points[MAX_POINTS];//CN:扫描点的具体信息(具体初始化个数由N决定)	EN:The specific information of the scanning point (the specific initialization number is determined by N)
	uint32_t ts[2];				//CN:时间戳(秒和毫秒)							EN:timestamps(Second and millisecond )

};


//客户设置的雷达设备配置信息
struct DevData
{
	int RPM;//转速    范围0450-1200   例如：LSRPM:0450H
	int ERR;//偏差	  范围-99-+99	例如:LSERR:-23H  LSERR:+23H
	char UDP[64];//udp组合信息  设置雷达IP地址 子网掩码 网关 服务端口号，例如LSUDP:192.168.158.091 255.255.255.000 192.168.158.001 05000H
	char DST[64];//接收雷达信息的ip地址和端口号  LSDST:192.168.158.043 12300H
	char NSP[32];//机器类型    LSNSP:LDS-50C-S-UH
	char UID[32];//机器序号		例如LSUID:201812030001H
	int FIR;//滤波圈数    (范围01~99) 例如：LSFIR:03H
	int PUL;//设置电机启动脉冲数   (范围0500~4000) 例如：LSPUL:2000H
	int VER;//设置版本号 例如：LSPUL:2000H
	int PNP;//设置IO类型  比如设置LSNPN:1H 输出IO类型为PNP
	int SMT;//数据平滑  LSSMT:1H  打开 LSSMT:0H  关闭
	int DSW;//去拖点  LSDSW:1H 打开 LSDSW:0H  关闭
	int DID;//设备ID   LSDID:xxxH
	int ATS;//开机自动上传  LSATS:xH    1/0   打开/关闭   
	int TFX;//固定上传地址  LSTFX:xH    1/0   打开/关闭   
	//另外
	int PST;//数据/报警上传类型  LSPST:xH    0:无 1:数据 2报警  3 数据+报警  
	int AF;//去拖点系数
	char set[18];//需要设置的参数，1需要设置  0不设置	
	char result[35];//设置是否成功  1成功   0失败
};
struct  DevTimestamp
{
	char ip[256];
	int port;
	uint32_t timestamp;
	uint32_t delay;
};
struct EEpromV101
{
	char label[4];			// "EPRM"
	uint16_t pp_ver; // paramter protocol version
	uint16_t size;			// total size of this structure

	//uint32_t version;		// firmware version

	// device
	uint8_t dev_sn[20];
	uint8_t dev_type[16];
	uint32_t dev_id;		// identiy

	// network
	uint8_t IPv4[4];
	uint8_t mask[4];
	uint8_t gateway[4];
	uint8_t srv_ip[4];
	uint16_t srv_port;
	uint16_t local_port;

	//
	uint16_t RPM;
	uint16_t RPM_pulse;
	uint8_t fir_filter;
	uint8_t cir;
	uint16_t with_resample;

	uint8_t auto_start;
	uint8_t target_fixed;
	uint8_t with_smooth;
	uint8_t with_filter;

	//
	uint8_t ranger_bias[8];
	uint32_t net_watchdog;

	uint32_t pnp_flags;
	uint16_t deshadow;
	uint8_t zone_acted;
	uint8_t should_post;

	uint8_t functions_map[16];
	uint8_t reserved[36];
};


/************************************************
* @functionName:  parse_data
* @date:          2022-03-25
* @description:   CN: 固定格式解析数据	EN:Fixed format parsing data
* @Parameter:
				  1.len[int,IN]					CN:数据的长度						EN:length of data		
				  2.buf[unsigned char*,IN]		CN:数据存储指针						EN:data storage pointer
				  3.span[int,IN]				CN:扇形角度							EN:Sector angle: 
				  4.is_mm[int,IN]				CN:毫米/厘米(1/0)					EN:mm/cm(1/0)
				  5.with_conf[int,IN]			CN:数据中带强度/数据中不带强度(1/0) EN:Data has strength/Data has no strength (1/0)
				  6.dat[RawData,OUT]			CN:解析后保留的数据					EN:Data retained after parsing
				  7.consume[int,OUT]			CN:解析处理的数据长度				EN:Parsing data length
				  8.with_chk[int,IN]			CN:数据校验标志位					EN:data check flag

* @return:        bool(true/false)
* @others:        Null
*************************************************/
bool parse_data(int len, unsigned char* buf, 
	int& span, int& is_mm, int& with_conf, 
	RawData& dat, int& consume, int with_chk);


/************************************************
* @functionName:  parse_data_x
* @date:          2022-03-25
* @description:   CN:多种格式解析数据	EN:Parse data in multiple formats
* @Parameter:
				  1.len[int,IN]					CN:数据的长度						EN:length of data
				  2.buf[unsigned char*,IN]		CN:数据存储指针						EN:data storage pointer
				  3.span[int,IN]				CN:扇形角度							EN:Sector angle:
				  4.is_mm[int,IN]				CN:毫米/厘米(1/0)					EN:mm/cm(1/0)
				  5.with_conf[int,IN]			CN:数据中带强度/数据中不带强度(1/0) EN:Data has strength/Data has no strength (1/0)
				  6.dat[RawData,OUT]			CN:解析后保留的数据					EN:Data retained after parsing
				  7.consume[int,OUT]			CN:解析处理的数据长度				EN:Parsing data length
				  8.with_chk[int,IN]			CN:数据校验标志位					EN:data check flag

* @return:        bool(true/false)
* @others:        Null
*************************************************/
bool parse_data_x(int len, unsigned char* buf, 
	int& span, int& is_mm, int& with_conf, 
	RawData& dat, int& consume, int with_chk);


/************************************************
* @functionName:  data_process
* @date:          2022-03-25
* @description:   CN:获取扫描点数组的具体信息	EN:Get the specific information of the scan point array
* @Parameter:
				  1.n[int,IN]				CN:点个数	EN: number of points
				  2.points[DataPoint,IN]	CN:点信息	EN: points information
				  3.timestamp[uint32_t,int]	CN:时间戳	EN:	timestamp

* @return:        Null
* @others:        Null
*************************************************/
void data_process2(const RawData& raw, const char* output_file, PointData& tmp);

/************************************************
* @functionName:  data_process
* @date:          2022-03-24
* @description:   
					CN:获取全部扇区具体数据（写在循环里，如果没到180°，则累加到数组，继续循环。如果到了180°，就解析全部的扇形数据）
					EN:Get the specific data of all sectors (write in a loop, if it is less than 180°, 
						it will be added to the array, the loop will continue, and if it reaches 180°, all sector data will be parsed)
* @Parameter:
				  1.raw[RawData,IN]  CN:存储数据的结构体	EN:structure to store data

* @return:        Null
* @others:        Null
*************************************************/
void data_process(const RawData& raw, const char* output_file, PointData& data);
/************************************************
* @functionName:  data_process
* @date:          2022-03-25
* @description:   CN:解析某一个扇区的点数据	EN:Parse point data for a sector
* @Parameter:
				  1.raw[RawData,IN]				CN:存储数据的结构体			EN:structure to store data
				  2.output_file[const char*,IN]	CN:输出文件路径(绝对路径)	EN:output file path (absolute path)

* @return:        Null
* @others:        Null
*************************************************/
void fan_data_process(const RawData& raw, const char* output_file, PointData&data);
/************************************************
* @functionName:  data_process
* @date:          2022-03-24
* @description:   CN:解析全部扇区的点数据	EN:Parse point data of all sectors
* @Parameter:
				  1.raw[RawData,IN]				CN:存储数据的结构体												EN:structure to store data
				  2.from_zero [bool IN]			CN:转动范围区间(这里是用来设置扇区的范围是-180-180还是0-360)    EN:Rotation range interval (here is used to set the range of the sector is -180-180 or 0-360)
				  3.output_file[const char*,IN]	CN:输出文件路径(绝对路径)										EN:output file path (absolute path)

* @return:        Null
* @others:        Null
*************************************************/
void whole_data_process(const RawData& raw, bool from_zero, const char* output_file, PointData& data);


extern int pack_format;//CN:用来保存帧头的第二个字节，用来判断帧头数据类型	EN:Used to save the second byte of the frame header, used to determine the data type of the frame header
extern char g_uuid[32];//CN:设备号  EN:device ID

#ifdef _WIN32
void gettimeofday(timeval* tv, void*);
//#define  sleep(x)   Sleep(x * 1000)
#endif



#endif
