#ifndef __ERROR_H_
#define __ERROR_H_

#define  SUCCESS   0   //成功
#define  FAILTORUN   -1   //雷达没有正常运行
//入参相关
#define  ARG_ERROR_NUM  -10    //入参个数错误
#define  ARG_ERROR_FILE_EXIST  -11 //入参传入的配置文件路径错误
#define  ARG_ERROR_TYPE     -12  //入参类型type不在可选的范围内
#define  SIGNAL_BIND_FAILED  -13//信号绑定函数失败
#define  FUNCTION_NOT_SUPPORT  -14//该设备不支持此功能
#define  UNDEFINED_ERROR     -15//未知报错




#define UPDATE_ATTRS_FAILED  -23//更新终端命令失败
#define UPDATE_BANDRATE_FIILED -24//更改端口号失败
#define BIND_SOCKET_PORT_FAILED -26//绑定端口失败
#define THREAD_CREATE_FAILED -27 //线程创建失败
#define READ_UART_FAILED  -28//循环读取串口文件失败
#define  MUTICAST_GROUP_FIALED  -29//组播模式创建失败
#define MSG_POST_FAILED    -30//信号发送失败

//open
#define OPEN_UART_FD_FAILED -31//打开uart句柄失败
#define OPEN_UDP_FD_FAILED -32//打开udp句柄失败
#define  OPEN_SERIAL_PORT_FAILED  -33//打开串口端口失败
#define OPEN_ATTRS_FAILED  -34//获取终端句柄
#define OPEN_SOCKET_PORT_FAILED -35//打开socket端口失败


//get
#define GET_DEVINFO_FAILED -41//获取设备信息失败
#define GET_DEVPONIT_FAILED -42 //获取设备点位数据失败
#define GET_DEVTIMESTAMP_FAILED -43 //获取设备时间戳失败
#define GET_ONEPOINT_FAILED	    -44//获取一组雷达点位信息失败
#define Get_ZONE_FAILED  -45 //读取防区失败

//set
#define SET_DEVINFO_FAILED -51//设置设备信息失败
#define SET_DEV_FAILED  -52//设置设备启停重启测试失败
#define SET_EVENT_FAILED  -53//设置事件同步失败
#define Set_ZONE_FAILED  -54 //设置防区失败
#define SET_ZONESECTION_FAILED -55 //设置激活的防区失败
//net
#define URL_ADDRESS_FAILED  -60//url地址不存在

#endif