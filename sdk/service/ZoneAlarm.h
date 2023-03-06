#ifndef __ZONEALARM_H_
#define  __ZONEALARM_H_
//#include<iostream>
#include"data.h"
#include<math.h>
#define MAX_ZONE_LINES 1000
#define PACK_PREAMLE 0X484C

#define OP_ZONE_ERASE		0xFD00EEEE
#define OP_ZONE_RELOAD		0xFD0010AD

#define OP_ZONE_QUERY		0xFD009E21
#define OP_ZONE_INFO		0xFD0010F0

#define GRAPH_BUF_SIZE 960
#define ZONE_MAX_SIZE  20    //最多同时设置的防区数量

#define ZONE_RETRY_NUMBER  10//防区读写失败最多尝试的次数

struct PacketBuf
{
	int len;
	struct sockaddr_in addr;
	unsigned char* buf;
};
struct LIDAR_UDP
{
	unsigned short face;
	unsigned short N;
	unsigned short start;
	unsigned short dist[1024];
};

struct UnknownMsg
{
	int len;
	unsigned char buff[4];
};
struct KnownMsg
{
	CmdHeader hdr;
	unsigned char buf[2048];
};
struct Range {
	uint16_t Near, Far;
};
struct Ray {
	struct Range zone[3];
};

struct ZoneHeader {
	uint32_t code;// must be "PZON"
	uint32_t proto_ver;// Ð­Òé°æ±¾ V101
	uint32_t size;
	uint32_t crc;
	uint32_t ray_cnt;
	uint32_t last_modified;
	uint32_t reserved[250];
	uint8_t graphics[15 * 1024];
};

struct ZoneDef {
	struct ZoneHeader hdr;
	struct Ray rays[16 * 1024];
};
struct TransBuf {
	uint32_t total;
	uint32_t crc;
	uint8_t filled[512];
	uint8_t buf[1024 * 1024];
};
//标记
struct ZonePacket
{
	uint32_t proto_version;
	//uint32_t total;
	uint32_t op_offset;
	uint16_t len;
	uint16_t flags;
	union {
		uint32_t total_crc[2];
		uint32_t buf[128];
	};
};


struct ZoneWriteResp {
	uint32_t proto_version;
	uint32_t op_offset;
	int32_t result;
	char msg[128];
};
enum DRAW_STATE
{
	AT_NONE,
	AT_MOVING,
	AT_SELECTED,
	AT_SCROLL,
	AT_RECT_0,
	//AT_RECT_1,
	AT_ELLIPSE_0,
	//AT_ELLIPSE_1,
	AT_FAN_0,
	AT_POLYGON_0,
	AT_POLYGON_1,
	AT_POLYGON_2,
};

enum POLYGON_TYPE
{
	POLY_RAW = 1,
	POLY_RECT,
	POLY_ELLIPSE,
	POLY_FAN
};
struct Point
{
	long x;
	long y;
};
struct PolyPoint
{
	Point pt;
	int selected;
	PolyPoint* next;
};

struct PolygonX
{
	POLYGON_TYPE type;
	PolyPoint* first;
	int zone_id;
	int output_id;
	//RECT box;
//	int selected;
	PolygonX* next;
	PolyPoint* helper;
};

struct Sketch
{
	int n;
	int output_id;
	int xs[3600];
	int ys[3600];
};

struct Vertex
{
	int n;
	int xs[1024];
	int ys[1024];
};

struct Fan
{
	double from, to;
	double r1, r2;
};


struct ResendPack
{
	time_t timeout;
	uint32_t tried;
	uint16_t cmd;
	uint16_t sn;
	uint16_t len;
	char buf[2048];
};
struct ProtectZoneInfo
{
	short version; // 版本号	2个字节
	int length; // 文件大小	4个字节		最大超过100kbyte
	short year; // 年	2个字节

	char month;// 月	1个字节
	char day; // 日	1个 字节

	unsigned char ang; // 角度精度	1个字节		实际精度放大十倍
	//unsigned char col; // 实际列项	1个字节		小于6，2的整数倍
	unsigned char graph_sig;

	unsigned short ngl;
	unsigned char graph[GRAPH_BUF_SIZE];
};

struct ProtectSeg
{
	unsigned short from;
	unsigned short to;
};
struct ProtectLine
{
	//unsigned short angle;
	ProtectSeg seg[3];
};


struct zone//结构体
{
	POLYGON_TYPE type;//图形类型
	int zoneID;
	int outputID;
	int arg1;
	int arg2;
	int arg3;
	int arg4;
};
struct zones
{
	int num;
	zone zoneArr[ZONE_MAX_SIZE];
};
struct ZoneReq
{
	uint32_t proto_version;
	//uint32_t zone_id;
	uint32_t op_offset;
	uint32_t size;
	uint32_t reserved[5];
};
struct RecvBuf {
	uint32_t total;
	uint32_t crc;
	uint8_t filled[512];
	uint8_t buf[1024 * 1024];
};

struct  RecvZoneData
{
	int zoneID;
	int outputID;
	int cnt;
	int type;
	int value[16];
};

struct  RecvZoneDatas {
	int N;
	int result;
	RecvZoneData zoneData[8];
};



class ZoneAlarm
{
public:
	
	ZoneAlarm(int fd,bool isUDP,void *ptr);
	ZoneAlarm(int fd, bool isUDP,char* dev_ip,int dev_port, void* ptr);
	~ZoneAlarm();

	int getZone(int sn);
	//虚拟串口
	int getZoneRev(unsigned char* buf, int cmd_len, int sn, int& consume);
	//网口
	int getZoneRev(unsigned char* buf,int sn);
	RecvZoneDatas* getZoneResult() { return m_zoneData;}
	

	int setZone(zones& data,int sn);
	//虚拟串口
	int setZoneRev(unsigned char* buf, int cmd_len, int sn,int &consume);
	//网口
	int setZoneRev(unsigned char* buf, int sn);
protected:

	void SendZoneReq(uint32_t offset, uint32_t size, uint16_t sn);
	void SendZoneWrite(uint32_t offset, uint32_t size, uint16_t sn);
	void SavePacket(uint16_t cmd, uint16_t sn, uint16_t len, void* buf);
	PolygonX* NewPolygon(int zone_id, POLYGON_TYPE type, int output_id);
	int GetAllSketch(int zone_id, Sketch** pps);
	int GetGraph(int zone_id, unsigned char* buf);
	bool PackAll();
	int SetGraph(int zone_id, int ngl, unsigned char* graph);
	void SetProtectLines(int zone_id, const ProtectLine* lines);
	void UpdatePolygon(PolygonX* po);
	void UpdatePolygon2(PolygonX* po);
private:
	bool isRun  {true};
	int m_fd{ 0 };
	PolygonX* m_selected_poly {NULL};
	PolyPoint* m_selected_point{ NULL };
	char m_devIP[16]  {0};
	int m_devPort{ 0 };
	ResendPack* m_resndBuf { NULL };
	RecvBuf* m_recvBuf{ NULL };
	TransBuf* m_transBuf { NULL };
	PolygonX* m_sketch{ NULL };
	ZoneDef* m_zoneDef {NULL};
	ProtectLine m_proLines[16][MAX_ZONE_LINES];
	int m_fail_num  {0};//当次设置防区失败次数
	RecvZoneDatas* m_zoneData{NULL};//最终整合后的防区范围信息
	bool m_fdType{ false };//防区消息交互的方式，false 串口 true 网口
	void* m_ptr{NULL};
};

void FreePolygon(PolygonX* po);
void CalcFan(Point pt1, Point pt2, Fan& fan);
int Fan2Sketch(const PolygonX* po, int* xs, int* ys);
int Raw2Vertex(const PolygonX* po, int* xs, int* ys);
bool Poly2Sketch(const PolygonX* po, Sketch& sketch);
int Rect2Vertex(const PolygonX* po, int* xs, int* ys);
int Ellipse2Vertex(const PolygonX* po, int n, int* xs, int* ys);
PolyPoint* PolygonAppend(PolygonX* po, int x, int y);
int hit_test(int ang, int npo, Sketch* pos, ProtectSeg* seg);
int hit_test(int ang, int outpid, int npo, Sketch* pos, ProtectSeg* seg);
#endif