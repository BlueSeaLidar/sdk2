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

#include <stdint.h>
#include <string>
#include<stdio.h>
#include <stdlib.h>
extern "C"
{
	#include"third_party/mongoose/mongoose.h"
}


#ifdef _WIN32
#include<Windows.h>
#define GetDevInfo_MSG WM_USER+100
#define SetDevInfo_MSG WM_USER+101
#define ctrl_MSG WM_USER+102
#define Print_Point_MSG WM_USER+103
#define Print_TimeStamp_MSG WM_USER+104
#define Get_OnePoint_MSG WM_USER+105
#define Get_ZONE_MSG WM_USER+106
#define Get_ZoneSection_MSG WM_USER+107
#define Set_ZONE_MSG WM_USER+110
#define Set_ZoneSection_MSG WM_USER+111
#elif __linux
#include <unistd.h>
enum msg_queue
{
	GetDevInfo_MSG=0,
	SetDevInfo_MSG,
	ctrl_MSG,
	Print_Point_MSG,
	Print_TimeStamp_MSG,
	Get_ZONE_MSG,
	Set_ZONE_MSG,
	Set_ZoneSection_MSG
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



#define PI 3.1415926535898

#define MAX_LIDARS 8


#define HDR_SIZE 6
#define HDR2_SIZE 8
#define HDR3_SIZE 16 
#define HDR7_SIZE 28 
#define HDR99_SIZE 32 
#define MAX_POINTS 500

#define BUF_SIZE 8*1024
#define ZONE_SIZE 1024
#define SLEEP_SIZE 5
#define USER_SIZE  10

#define ZS_PACK 0x5A53
#define ZG_PACK 0x5A47

#define GO_PACK 0x474F
#define GT_PACK 0x4754
#define G0_PACK 0x4730
#define G1_PACK 0x4731
#define GF_PACK 0x4746

#define FH_PACK 0x4648
#define F_PACK 0x0046
#define PO_PACK 0x504F
#define P0_PACK 0x5030
#define PT_PACK 0x5054
#define P1_PACK 0x5031
#define GS_PACK 0x4753
#define S_PACK 0x0053
#define T_PACK 0x0054
#define C_PACK 0x0043

#define _TEST_    0     //test use
//日志打印开关   debug  info
#define _DEBUG_  1
#define _INFO_   1
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

#define getbit(x,y)   ((x) >> (y)&1)
#define setbit(x,y) x|=(1<<y)         //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y)            //将X的第Y位清0


//CN：心跳检测包 EN：Heartbeat detection package
struct KeepAlive {
	uint32_t world_clock;//时间戳
	uint32_t mcu_hz;//mcu频率(内部使用)
	uint32_t arrive;//包数据主机到雷达的时间(内部使用)
	uint32_t delay;//延迟(内部使用)
	uint32_t reserved[4];//预留位
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
	char ip[16];
	int port;
	unsigned short N;			//CN:扇区内测距点数								EN:The number of ranging points in the sector
	DataPoint points[10000];//CN:扫描点的具体信息(具体初始化个数由N决定)	EN:The specific information of the scanning point (the specific initialization number is determined by N)
	uint32_t ts[2];				//CN:时间戳(秒和微秒)							EN:timestamps(Second and microseconds )
};
//心跳包
struct LidarMsgHdr
{
	char sign[4];  	// must be "LMSG"
	uint32_t proto_version;	//协议版本，当前为0x101
	char dev_sn[20];	//设备编号
	uint32_t dev_id;  	//设备序号
	uint32_t timestamp; 	//时间戳
	uint32_t flags;	//消息类型
	uint32_t events;	//消息内容的位组合
	uint16_t id;	//消息序号
	uint16_t extra;	//（当前激活防区 + 设备各功能状态 + 保留）长度
	uint32_t zone_actived;	//当前激活防区（范围0~F）
	uint8_t all_states[32];//	设备各功能状态
	uint32_t reserved[11];	//保留
};
//E100系列过滤点云数据
struct ShadowsFilterParam
{
	int enable;
	double max_range;
	double min_angle, max_angle;
	int window;
};
//E100系列过滤点云数据
struct MedianFilterParam
{
	int enable;
	int window;
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
	unsigned short angle;//CN:扇区起始角度		EN:Corresponding ranging angle
};

struct RawDataHdr2
{
	unsigned short code;	//CN:帧头				EN:data frame header
	unsigned short N;		//CN:扇区内测距点数		EN:The number of ranging points in the sector
	unsigned short angle;	//CN:扇区起始角度 		EN:Sector starting angle
	unsigned short span;	//CN:扇区大小 			EN:sector size
};

struct RawDataHdr3
{
	unsigned short code;	//CN:帧头				EN:data frame header
	unsigned short N;		//CN:扇区内测距点数		EN:The number of ranging points in the sector
	unsigned short angle;	//CN:扇区起始角度		EN:Corresponding ranging angle
	unsigned short span;	//CN:扇区大小			EN:sectors size
	unsigned short fbase;	//CN:扇区起始偏差		EN:Sector start offset
	unsigned short first;	//CN:第一个点角度		EN:first point angle
	unsigned short last;	//CN:最后一个点角度		EN:last point angle
	unsigned short fend;	//CN:扇区终止偏差		EN:Sector end offset
};


struct RawDataHdr7 {
	uint16_t code;		//CN:帧头						EN:data frame header
	uint16_t N;			//CN:扇区内这个分包的测距点数	EN:The number of ranging points for this subpacket in the buffer
	uint16_t whole_fan;	//CN:缓冲区内总测距点数			EN:The total number of ranging points in the sector
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
	uint16_t code;			//CN:帧头，固定为0x99FA			EN:Frame header, fixed at 0x99FA
	uint16_t N;				//CN:扇区内这个分包的测距点数	EN:The number of ranging points for this subpacket in the sector
	uint16_t from;			//CN:扇区起始角度			    EN:Sector start angle
	uint16_t total;			//CN:一整圈数据点个数			EN:The number of data points in a full circle
	uint32_t flags;			//CN:状态开关标识				EN:status switch flag
	uint32_t timestamp;		//CN:时间戳						EN:timestamp 
	uint32_t dev_no;		//CN:设备编号					EN:device ID
	uint32_t reserved[3];	//CN:保留位						EN:reserved
};


//Special instructions : fbase, first, last, fend are empty by default, only special models have data
struct RawData
{
	unsigned short code;		//CN:帧头										EN:data frame header
	unsigned short N;			//CN:扇区内测距点数								EN:The number of ranging points in the sector
	unsigned short angle;		//CN:当前扇区的起始角度							EN:The starting angle of the current sector *10
	unsigned short span;		//CN:扇区的总角度(扇区终止角度-扇区起始角度)	EN:The total angle of the sector (sector end angle - sector start angle)
	unsigned short fbase;		//CN:扇区起始偏差 （NULL）						EN:Sector start offset（NULL）
	unsigned short first;		//CN:第一个点角度  (NULL)						EN:first point angle（NULL）
	unsigned short last;		//CN:最后一个点角度(NULL)						EN:last point angle（NULL）
	unsigned short fend;		//CN:扇区终止偏差(NULL)							EN:Sector end offset（NULL）
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
//CN:UDP设置雷达参数时接收使用报文头	EN:UDP uses the header when setting lidar parameters
struct CmdHeader
{
	unsigned short sign; //CN:与硬件约定的标志位						EN:Flags consistent with hardware
	unsigned short cmd;	 //CN:命令										EN：command
	unsigned short sn;	//CN:随机数，发送报文和接收报文时验证是否一致	EN:Random numbers, verify consistency when sending and receiving messages
	unsigned short len;	//CN:命令长度									EN:command length
};



#endif
