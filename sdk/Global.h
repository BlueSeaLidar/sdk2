#pragma once
#include"data.h"
#include <set>
#include <vector>
#include<string>
#include<math.h>
#pragma warning(disable:4996)
#ifdef _WIN32
#include<Windows.h>
#ifndef sleep(sec)   Sleep(sec * 1000)
#define sleep(sec)   Sleep(sec * 1000)
#endif
#define msleep(msec) Sleep(msec)
#elif __linux
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <termios.h>
#define msleep(msec) usleep(msec * 1000)
#endif
#define UNUSED(x) (void)x
struct UARTARG
{
    char portName[16];
    int port;
};

namespace BaseAPI {
    bool checkAndMerge(int type, char* ip, char* mask, char* gateway, int port, char* result);
    std::string stringfilter(char *str,int num);
}

namespace ParseAPI {
    int parse_data_x(int len, unsigned char* buf,UartState *uartstate,
    RawData& dat, int& consume, int with_chk, char *result,CmdHeader *cmdheader,void** fan_segs);
    int parse_data(int len, unsigned char* buf,UartState *uartstate,RawData& dat, int& consume, int with_chk);
}

namespace UserAPI {
    void fan_data_process(const RawData& raw, std::vector<RawData>& whole_datas);
    int whole_data_process(const RawData& raw,int collect_angle, std::vector<RawData> &whole_datas,std::string &error);

    int autoGetFirstAngle(const RawData &raw, bool from_zero, std::vector<RawData> &raws,std::string &error);
}

namespace AlgorithmAPI_E100 {

    int ShadowsFilter(UserData*, const ShadowsFilterParam&);
    int MedianFilter(UserData*, const MedianFilterParam&);
}

namespace SystemAPI {
    int GetComList(std::vector<UARTARG>& list);
    int closefd(int __fd, bool isSocket);
    std::vector<std::string> GetComPort();
    int open_serial_port(const char* name, int speed);
    int open_socket_port(int localhost);
}
int GetDevInfoByVPC(const char* port_str, int speed);
int GetDevInfoByUART(const char* port_str, int speed);




namespace CommunicationAPI {
    void send_cmd_vpc(int hCom, int mode, int sn, int len, const char* cmd);
    bool uart_talk(int hCom, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);
    bool vpc_talk(int hCom, int mode, short sn, int len, const char* cmd, int nfetch, void* fetch);
    void send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf);
    bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch);
    bool udp_talk_S_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result);
    bool udp_talk_GS_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result);
}



unsigned int stm32crc(unsigned int* ptr, unsigned int len);

#ifdef _WIN32
    void gettimeofday(timeval* tv, void*);
#endif


#ifdef _WIN32
    int Open_serial_port(const char* name, int port);
    int read(int __fd, void* __buf, int __nbytes);
    int write(int __fd, const void* __buf, int __n);
#elif __linux__
    extern "C"  int change_baud(int fd, int baud);
    int Open_serial_port(const char* name, int port);
#endif