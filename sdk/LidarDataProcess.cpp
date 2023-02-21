

#include "LidarDataProcess.h"
#include"error.h"
#include"service/ZoneAlarm.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#ifdef __unix__
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

#endif

send_cmd_udp_ptr CallBack_Udp;
send_cmd_uart_ptr CallBack_Uart;


bool setup_lidar_udp(int fd_udp, const char* ip, int port, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, int should_post, char* version)
{
	char buf[32];
	int nr = 0;
	//初始化默认开始旋转
	if (udp_talk_C_PACK(fd_udp, ip, port, 6, "LSTARH", 2, "OK", 0, NULL))
	{
		printf("set LiDAR LSTARH  OK \n");
	}
	//硬件版本号
	if (udp_talk_C_PACK(fd_udp, ip, port, 6, "LXVERH", 14, "MOTOR VERSION:", 15, buf))
	{
		memcpy(version, buf, 12);
		printf("set LiDAR LXVERH  OK %s\n", version);
	}

	//查询当前雷达配置信息，如果不同则修改
	EEpromV101* eepromv101 = new EEpromV101;
	memset(eepromv101, 0, sizeof(EEpromV101));
	char result[3] = { 0 };
	if (!udp_talk_GS_PACK(fd_udp, ip, port, 6, "xxxxxx", eepromv101))
	{
		DEBUG_PR("GetDevInfo_MSG failed\n");
		return false;
	}
	if (eepromv101->with_filter != with_deshadow)
	{
		char cmd[12] = { 0 };
		sprintf(cmd, "LSDSW:%dH", with_deshadow);
		if (udp_talk_S_PACK(fd_udp, ip, port, sizeof(cmd), cmd, result))
		{
			printf("set LiDAR deshadow %s\n", result);
		}
		else
		{
			printf("set LiDAR deshadow NG\n");
		}
	}
	if (eepromv101->with_smooth != with_smooth)
	{
		char cmd[12] = { 0 };
		sprintf(cmd, "LSSMT:%dH", with_smooth);
		if (udp_talk_S_PACK(fd_udp, ip, port, 6, cmd, result))
		{
			printf("set LiDAR with_smooth %s\n", result);
		}
		else
		{
			printf("set LiDAR with_smooth NG\n");
		}
	}
	if (eepromv101->with_resample != resample)
	{
		//resample == 0  非固定角分辨率不适用于网络包计算 
		if (resample == 1 || (resample > 100 && resample <= 1500))
			sprintf(buf, "LSRES:%04dH", resample);
		else
			buf[0] = 0;

		if (buf[0]) {
			if (udp_talk_C_PACK(fd_udp, ip, port, 10, buf, 2, "OK", 0, NULL))
			{
				printf("set LiDAR resample %d OK\n", resample);
			}
			else
			{
				printf("set LiDAR resample %d NG\n", resample);
			}
		}

	}
	if (eepromv101->RPM != init_rpm)
	{
		for (int i = 0; i < 5; i++)
		{
			char cmd[16];
			sprintf(cmd, "LSRPM:%dH", init_rpm);
			if (udp_talk_S_PACK(fd_udp, ip, port, strlen(cmd), cmd, result))
			{
				printf("set RPM to %d  %s\n", init_rpm, result);
				break;
			}
			else
			{
				printf("set RPM to %d  NG   index=%d\n", init_rpm, i + 1);
			}
		}
	}
	if (eepromv101->should_post != (should_post == 1 ? 3 : 1))
	{
		char cmd[12] = { 0 };
		sprintf(cmd, "LSPST:%dH", should_post == 1 ? 3 : 1);
		if (udp_talk_S_PACK(fd_udp, ip, port, sizeof(cmd), cmd, result))
		{
			printf("set LiDAR %s %s\n", cmd, result);
		}
		else
		{
			printf("set LiDAR should_post NG\n");
		}
	}
	delete eepromv101;
	//网络款都是毫米级,带强度
	/*if (udp_talk_C_PACK(fd_udp, ip, port, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH",2, "OK", 0, NULL))
	{
		printf("set LiDAR unit_is_mm  OK\n");
	}

	if (udp_talk_C_PACK(fd_udp, ip, port, 6, with_confidence == 0 ? "LNCONH" : "LOCONH",2, "OK ", 0, NULL))
	{
		printf("set LiDAR with_confidence OK\n");
	}*/

	return true;
}


int setup_lidar_extre(std::string type,int fd, const char* ip, int port, DevData& data)
{
	//检测需要设置的参数项
	char result[3] = { 0 };
	char cmd[128];
	for (int i = 0; i < sizeof(data.set) - 1; i++)
	{
		int flag = 0;
		memset(cmd, 0, sizeof(cmd));
		//转速
		if (i == 0 && data.set[i] == '1')
		{
			flag = 1;
			int RPM = data.RPM;
			sprintf(cmd, "LSRPM:%04dH", RPM);
		}
		//偏差距离
		else if (i == 1 && data.set[i] == '1')
		{
			flag = 1;
			int ERR = data.ERR;
			if (ERR > 0)
				sprintf(cmd, "LSERR:+%dH", ERR);
			else
				sprintf(cmd, "LSERR:%dH", ERR);
		}
		else if (i == 2 && data.set[i] == '1')
		{
			flag = 1;
			sprintf(cmd, "LSUDP:%sH", data.UDP);
		}
		else if (i == 3 && data.set[i] == '1')
		{
			flag = 1;
			sprintf(cmd, "LSDST:%sH", data.DST);
		}
		else if ((i == 4) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSNSP:%sH", data.NSP);
		}
		else if ((i == 5) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSUID:%sH", data.UID);
		}
		else if ((i == 6) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSFIR:%02dH", data.FIR);
		}
		else if ((i == 7) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSPUL:%04dH", data.PUL);
		}
		else if ((i == 8) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSVER:%04dH", data.VER);
		}
		else if ((i == 9) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSPNP:%dH", data.PNP);
		}
		else if ((i == 10) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSSMT:%dH", data.SMT);
		}
		else if ((i == 11) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSDSW:%dH", data.DSW);
		}
		else if ((i == 12) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSDID:%dH", data.DID);
		}
		else if ((i == 13) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSATS:%dH", data.ATS);
		}
		else if ((i == 14) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSTFX:%dH", data.TFX);
		}
		else if ((i == 15) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSPST:%dH", data.PST);
		}
		else if ((i == 16) && (data.set[i] == '1'))
		{
			flag = 1;
			sprintf(cmd, "LSAF:%dH", data.AF);
		}
		if (flag == 1)
		{
			int index = 3;
			bool ret = false;
			//发送命令合成
			while (index--)
			{
				if(type =="udp")
					ret = udp_talk_S_PACK(fd, ip, port, sizeof(cmd), cmd, result);
#ifdef _WIN32
				if (type == "vpc")
					ret = uart_talk3((void*)fd, 0x0053, rand(), sizeof(cmd), cmd, 3, result);
#elif __unix__
				if (type == "vpc")
					ret = uart_talk3(fd, 0x0053, rand(), sizeof(cmd), cmd, 3, result);
#endif

				INFO_PR("%s %d\n", cmd, ret);
				if (ret)
				{
					memcpy(data.result + 2 * i, result, 2);
					break;
				}
				//特殊情况:调整了雷达的ip和端口号，雷达会重新启动，这里需要循环发送命令
				else
				{
					//如果最后一次也没有收到应答，则默认为失败
					if (index == 1)
						memcpy(data.result + 2 * i, "NG", 2);
					DEBUG_PR("timeout waiting:%d\n", index);
					sleep(1);
				}
			}
			flag = 0;
		}
	}
	return 0;
}
bool udp_talk_GS_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result)
{
	unsigned short sn = rand();
	send_cmd_udp(fd_udp, ip, port, 0x4753, sn, n, cmd, false);

	int nr = 0;
	for (int i = 0; i < 1000; i++)
	{
		fd_set fds;
		FD_ZERO(&fds);

		FD_SET(fd_udp, &fds);

		struct timeval to = { 1,0 };
		int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);

		if (ret <= 0)
		{
			return false;
		}

		// read UDP data
		if (FD_ISSET(fd_udp, &fds))
		{
			nr++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			char buf[1024] = { 0 };
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader* hdr = (CmdHeader*)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;

				memcpy(result, buf + 8, sizeof(EEpromV101));
				EEpromV101 t;
				memcpy(&t, result, sizeof(EEpromV101));
				return true;
			}
		}
	}

	printf("read %d packets, not response\n", nr);
	return false;
}
//配置信息设置
bool udp_talk_S_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result)
{
	unsigned short sn = rand();
	send_cmd_udp(fd_udp, ip, port, 0x0053, sn, n, cmd, false);

	int nr = 0;
	for (int i = 0; i < 1000; i++)
	{
		fd_set fds;
		FD_ZERO(&fds);

		FD_SET(fd_udp, &fds);

		struct timeval to = { 3, 0 };
		int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);
		if (ret <= 0)
		{
			return false;
		}
		// read UDP data
		if (FD_ISSET(fd_udp, &fds))
		{
			nr++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);
			char buf[1024] = { 0 };
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader* hdr = (CmdHeader*)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;
				memcpy(result, buf + 8, 2);
				return true;
			}
		}
	}

	INFO_PR("read %d packets, not response\n", nr);
	return false;
}

bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port,
	int n, const char* cmd,
	int nhdr, const char* hdr_str,
	int nfetch, char* fetch)
{
	printf("send command : \'%s\' \n", cmd);

	unsigned short sn = rand();
	send_cmd_udp(fd_udp, lidar_ip, lidar_port, 0x0043, sn, n, cmd, false);

	time_t t0 = time(NULL);
	int ntry = 0;
	while (time(NULL) < t0 + 3 && ntry < 1000)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd_udp, &fds);

		struct timeval to = { 1, 0 };
		int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);

		if (ret < 0)
		{
			printf("select error\n");
			return false;
		}
		if (ret == 0)
		{
			continue;
		}

		// read UDP data
		if (FD_ISSET(fd_udp, &fds))
		{
			ntry++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			char buf[1024] = { 0 };
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader* hdr = (CmdHeader*)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;

				char* payload = buf + sizeof(CmdHeader);
				for (int i = 0; i < nr - nhdr - 1; i++)
				{
					if (memcmp(payload + i, hdr_str, nhdr) == 0)
					{
						if (nfetch > 0)
						{
							memset(fetch, 0, nfetch);
							for (int j = 0; j < nfetch && i + nhdr + j < nr; j++)
								fetch[j] = payload[i + nhdr + j];
						}
						return true;
					}
				}
			}
		}
	}
	printf("read %d packets, not response\n", ntry);
	return false;
}
void  send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf, bool savelog)
{
	char buffer[2048];
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd;
	hdr->sn = sn;

	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), snd_buf, len);

	int n = sizeof(CmdHeader);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = BaseAPI::stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	sockaddr_in to;
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = inet_addr(dev_ip);
	to.sin_port = htons(dev_port);

	int len2 = len + sizeof(CmdHeader) + 4;

	sendto(fd_udp, buffer, len2, 0, (struct sockaddr*)&to, sizeof(struct sockaddr));

	if (savelog) {
		char s[3096];
		for (int i = 0; i < len2; i++)
			sprintf(s + 3 * i, "%02x ", (unsigned char)buffer[i]);

		printf("send to %s:%d 0x%04x sn[%d] L=%d : %s\n",
			dev_ip, dev_port, cmd, sn, len, s);
	}
}
#ifdef _WIN32
DWORD  WINAPI lidar_thread_proc_udp(void* param)
{
	//启动消息队列
	MSG msg;
	PeekMessage(&msg, NULL, GetDevInfo_MSG, Print_TimeStamp_MSG, PM_NOREMOVE);
	bool isrun = false;//初始化运行雷达是否正常运行标志位
	bool timeprint = false;
	RunConfig* cfg = (RunConfig*)param;
	int zoneFlag = 0;//读写防区标志位    0为正常运行  1为读  2为写
	int zoneSN = rand();
	CallBack_Udp = send_cmd_udp;
	ZoneAlarm* zonealarm = new ZoneAlarm(cfg->fd, true, cfg->lidar_ip, cfg->lidar_port, CallBack_Udp);
	PointData tmp;
	strcpy(tmp.ip, cfg->lidar_ip);
	tmp.port = cfg->lidar_port;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	time_t tto = tv.tv_sec + 1;
	uint32_t delay = 0;
	FanSegment** fan_segs = new FanSegment*;
	*fan_segs = NULL;
	std::vector<RawData*> whole_datas;
	//ws2_32库会导致setsockopt失败，这里使用wsock32库
	//设置组播模式
	if (cfg->is_group_listener == 1)
	{
		ip_mreq group;
		memset(&group, 0, sizeof(group));
		group.imr_multiaddr.s_addr = inet_addr(cfg->group_ip);
		group.imr_interface.s_addr = INADDR_ANY;
		int rt = setsockopt(cfg->fd, IPPROTO_IP,
			IP_ADD_MEMBERSHIP, (char*)&group,
			sizeof(group));

		if (rt < 0)
		{
			DEBUG_PR("Adding to multicast group %s %s\n", cfg->group_ip, rt < 0 ? "fail!" : "ok");
			return MUTICAST_GROUP_FIALED;
		}
		INFO_PR("Adding to multicast group success\n");
	}
	else
	{
		setup_lidar_udp(cfg->fd, cfg->lidar_ip, cfg->lidar_port, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->alarm_msg, cfg->version);
	}

	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	bool should_publish = false;
	int fan_span = 360;
	while (!cfg->should_quit)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(cfg->fd, &fds);
		struct timeval to = { 1, 0 };
		if (cfg->is_group_listener != 1)
		{
			int ret = select(cfg->fd + 1, &fds, NULL, NULL, &to);
			gettimeofday(&tv, NULL);

			if (tv.tv_sec > tto)
			{
				KeepAlive alive;
				gettimeofday(&tv, NULL);
				alive.world_clock = (tv.tv_sec % 3600) * 1000 + tv.tv_usec / 1000;
				alive.delay = delay;
				// acknowlege device
				// int rt = send_cmd_udp(fd_udp, info->lidar_ip, info->lidar_port, 0x4753, rand(), 0, NULL);
				send_cmd_udp(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 0x4b41, rand(), sizeof(alive), &alive, false);
				if (timeprint)
				{
					DevTimestamp devtimestamp;
					memcpy(devtimestamp.ip, cfg->lidar_ip, sizeof(cfg->lidar_ip));
					devtimestamp.port = cfg->lidar_port;
					devtimestamp.timestamp = alive.world_clock;
					devtimestamp.delay = delay;
					cfg->callback(4, &devtimestamp);
				}
				tto = tv.tv_sec + 1;
			}

			//if (ret == 0)
			//{
			//	//当雷达没有向指定主机传输数据时，使雷达发送数据，其他控制指令可以达到同样的效果，仅兼容老版本使用
			//	//send_cmd_udp(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 0x0043, rand(), 6, "LGCPSH");
			//	//continue;
			//}

			if (ret < 0)
			{
				DEBUG_PR("select error\n");
				break;
			}
		}
		// read UDP data
		if (FD_ISSET(cfg->fd, &fds))
		{
			sockaddr_in addr;
			int sz = sizeof(addr);

			int len = recvfrom(cfg->fd, (char*)buf, BUF_SIZE, 0, (struct sockaddr*)&addr, &sz);

			if (len > 0)
			{
				int res = -1;
				if (zoneFlag == 1)
				{
					res = zonealarm->getZoneRev(buf, zoneSN);
					if (res != 0)
					{
						RecvZoneDatas* rev = new RecvZoneDatas;
						memcpy(rev, zonealarm->getZoneResult(), sizeof(RecvZoneDatas));
						rev->result = res;
						if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)rev, 0))
						{
							DEBUG_PR("threadson post message  get_ZONE_MSG failed,errno:%d\n", ::GetLastError());
						}
						zoneFlag = 0;
					}
				}
				//设置防区
				else if (zoneFlag == 2)
				{
					res = zonealarm->setZoneRev(buf, zoneSN);
					if (res != 0)
					{
						int* rev = new int;
						*rev = res;
						if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)rev, 0))
						{
							DEBUG_PR("threadson post message  Set_ZONE_MSG failed,errno:%d\n", ::GetLastError());
						}
						char tmpbuf[16] = { 0 };
						udp_talk_C_PACK(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 6, "LSTARH", 2, "OK", 11, tmpbuf);
						zoneFlag = 0;
					}

				}//点云数据
				else {

					RawData dat;
					LidarMsgHdr zone;
					memset(&zone, 0, sizeof(LidarMsgHdr));
					int is_pack;
					int consume;
					if (cfg->data_bytes == 3)
					{
						is_pack = ParseAPI::parse_data_x(len, buf,
							fan_span, cfg->unit_is_mm, cfg->with_confidence,
							dat, consume, cfg->with_chk, zone, fan_segs);

					}
					else
					{
						is_pack = ParseAPI::parse_data(len, buf,
							fan_span, cfg->unit_is_mm, cfg->with_confidence,
							dat, consume, cfg->with_chk);
					}
					if (is_pack)
					{
						//！！！CN:用户需要提取数据的操作，可以参考该函数,具体详细的其他打印操作参考user.cpp文件
						//！！！EN:User needs to extract data operation, you can refer to this function，For details about other printing operations, please refer to the user.cpp file
						//memset(&tmp, 0, sizeof(PointData));
						if (cfg->output_scan)
						{
							if (is_pack == 2)
							{
								((void(*)(int, void*))cfg->callback)(2, &zone);
								memcpy(&cfg->zone, &zone, sizeof(LidarMsgHdr));
								continue;
							}
							else if (is_pack == 3)
							{
								continue;
							}

							if (!cfg->output_360)
							{
								//多扇区分次发送
								UserAPI::fan_data_process(dat, cfg->output_file, tmp);
							}
							else
							{
								// 全部点位，全部扇区
								UserAPI::whole_data_process(dat, cfg->from_zero, cfg->collect_angle, cfg->output_file, tmp, whole_datas);
							}
							if (tmp.N > 0)
							{
								((void(*)(int, void*))cfg->callback)(1, &tmp);
								memcpy(&cfg->pointdata, &tmp, sizeof(PointData));


							}
						}
						//证明雷达已经正常运行
						if (isrun == false) //set thread start event 
						{
							if (cfg->hStartEvent == 0 || SetEvent(cfg->hStartEvent) == 0)
							{
								printf("set start event failed,errno:%d\n", ::GetLastError());
								return 1;
							}
							else
							{
								isrun = true;
							}

						}
					}

				}
			}
		}
		if (PeekMessage(&msg, NULL, GetDevInfo_MSG, Set_ZoneSection_MSG, PM_REMOVE)) //get msg from message queue
		{
			/*CMD *cmd = (CMD*)msg.wParam;
			INFO_PR("threadson recv framehead:%x  str:%s addr:%x\n", cmd->framehead,cmd->str, cmd);*/
			switch (msg.message)
			{
			case GetDevInfo_MSG:
			{
				EEpromV101* eepromv101 = new EEpromV101;
				memset(eepromv101, 0, sizeof(EEpromV101));
				if (!udp_talk_GS_PACK(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 6, "LUUIDH", eepromv101))
				{
					DEBUG_PR("GetDevInfo_MSG failed\n");
					break;
					//((void (*)(int, void*))cfg->callback)(2, eepromv101);
				}
				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)eepromv101, 0))//post thread msg
					printf("threadson post message failed,errno:%d\n", ::GetLastError());
				break;
			}
			case SetDevInfo_MSG:
			{
				DevData* devdata = (DevData*)msg.wParam;
				setup_lidar_extre(cfg->type,cfg->fd, cfg->lidar_ip, cfg->lidar_port, *devdata);
				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)devdata, 0))
				{
					printf("threadson post message failed,errno:%d\n", ::GetLastError());
					delete devdata;
				}
				break;
			}
			case ctrl_MSG:
			{
				char* cmd = (char*)msg.wParam;
				send_cmd_udp(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 0x0043, rand(), 6, cmd,false);
				if (!PostThreadMessage(msg.lParam, msg.message, NULL, 0))
				{
					DEBUG_PR("threadson post message failed,errno:%d\n", ::GetLastError());
				}
				printf("%s\n", cmd);
				delete[]cmd;
				break;
			}
			case Print_Point_MSG:
			{
				int* cmd = (int*)msg.wParam;
				cfg->output_scan = *cmd;
				if (!PostThreadMessage(msg.lParam, msg.message, NULL, 0))
				{
					DEBUG_PR("threadson post message  print_MSG failed,errno:%d\n", ::GetLastError());
				}
				delete cmd;
				break;
			}
			case Print_TimeStamp_MSG:
			{
				int* cmd = (int*)msg.wParam;
				timeprint = *cmd;
				int* rev = new int;
				*rev = 0;
				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)rev, 0))
				{
					DEBUG_PR("threadson post message  Print_TimeStamp_MSG failed,errno:%d\n", ::GetLastError());
				}
				delete cmd;
				break;
			}
			// case Get_OnePoint_MSG:
			// {
			// 	PointData* rev = new PointData;
			// 	if (tmp.N>0)
			// 		memcpy(rev, &tmp, sizeof(PointData));

			// 	if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)rev, 0))
			// 	{
			// 		DEBUG_PR("threadson post message  Get_OnePoint_MSG failed,errno:%d\n", ::GetLastError());
			// 	}
			// 	break;
			// }
			case Get_ZONE_MSG:
			{
				zonealarm->getZone(zoneSN);
				zoneFlag = 1;
				break;
			}
			case Set_ZONE_MSG:
			{
				char tmpbuf[12] = { 0 };
				udp_talk_C_PACK(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 6, "LSTOPH", 2, "OK", 11, tmpbuf);
				zones* cmd = (zones*)msg.wParam;
				zonealarm->setZone(*cmd, zoneSN);
				zoneFlag = 2;
				delete[]cmd;
				break;
			}
			case Set_ZoneSection_MSG:
			{
				int* cmd = (int*)msg.wParam;
				char* tmpbuf = new char[3];
				tmpbuf[2] = '\0';
				char tmp[12] = { 0 };
				sprintf(tmp, "LSAZN:%dH", *cmd);
				if (!udp_talk_S_PACK(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 8, tmp, tmpbuf))
				{
					strcpy(tmpbuf, "NG");
				}

				//设置上传的数据类型为  数据加报警，否则切换防区通道没有实际意义
				if (!udp_talk_S_PACK(cfg->fd, cfg->lidar_ip, cfg->lidar_port, sizeof(tmp), "LSPST:3H", tmpbuf))
				{

					strcpy(tmpbuf, "NG");

				}

				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)tmpbuf, 0))
				{
					DEBUG_PR("threadson post message  Print_TimeStamp_MSG failed,errno:%d\n", ::GetLastError());
				}
				delete[]cmd;
				break;
			}
			}
		}

	}
	CloseHandle((void*)cfg->fd);
	WSACleanup();
	delete[]buf;
	return NULL;

}

bool uart_talk(HANDLE hCom, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch)
{
	printf("send command : %s\n", cmd);
	DWORD nr = 0;
	WriteFile(hCom, cmd, n, &nr, NULL);

	char buf[2048];
	ReadFile(hCom, buf, sizeof(buf), &nr, NULL);

	while (nr < (int)sizeof(buf))
	{
		DWORD n;
		ReadFile(hCom, buf + nr, sizeof(buf) - nr, &n, NULL);
		if (n > 0) nr += n;
	}

	for (int i = 0; i < (int)sizeof(buf) - nhdr - nfetch; i++)
	{
		if (memcmp(buf + i, hdr_str, nhdr) == 0)
		{
			if (nfetch > 0)
			{
				memcpy(fetch, buf + i + nhdr, nfetch);
				fetch[nfetch] = 0;
			}
			return true;
		}
	}

	/*char path[256];
	sprintf_s(path, 250, "./tmp/%s.dat", hdr_str);
	FILE* fp;
	if (fopen_s(&fp, path, "wb") == 0)
	{
		fwrite(buf, 1, sizeof(buf), fp);
		fclose(fp);
	}*/

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return false;
}

bool uart_talk2(HANDLE hCom, int mode, int sn, int len, const char* cmd, int nfetch, char* fetch)
{
	printf("USB send command : %s\n", cmd);
	char buffer[2048];
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = mode;
	hdr->sn = sn;
	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), cmd, len);

	int n = sizeof(CmdHeader);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = BaseAPI::stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	DWORD nr = 0;
	WriteFile(hCom, buffer, len2, &nr, NULL);

	char buf[2048];
	int index = 10;
	//4C 48 BC FF   xx xx xx xx  result 
	//读取之后的10*2048个长度，如果不存在即判定失败
	while (index--)
	{
		ReadFile(hCom, buf, sizeof(buf), &nr, NULL);
		while (nr < (int)sizeof(buf))
		{
			DWORD n;
			ReadFile(hCom, buf + nr, sizeof(buf) - nr, &n, NULL);
			if (n > 0) nr += n;
		}

		for (int i = 0; i < (int)sizeof(buf) - nfetch; i++)
		{
			if (buf[i] == 0x4C && buf[i + 1] == 0x48 && buf[i + 2] == (signed char)0xBC && buf[i + 3] == (signed char)0xFF)
			{
				/*int packSN = ((unsigned int)buf[i + 5] << 8) | (unsigned int)buf[i + 4];
					if (packSN != sn)
						continue;*/

				for (int j = 0; j < nfetch; j++)
				{
					if ((buf[i + j + 8] >= 33 && buf[i + j + 8] <= 127))
					{
						fetch[j] = buf[i + j + 8];
					}
					else
					{
						fetch[j] = ' ';
					}
				}
				fetch[nfetch] = 0;
				return true;
			}
		}
	}
	printf("read %d bytes, not found %s\n", nr, cmd);
	return false;
}
bool uart_talk3(HANDLE hCom, int mode, int sn, int len, const char* cmd, int result_len, void* result)
{
	printf("USB send command : %s\n", cmd);
	char buffer[2048];
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = mode;
	hdr->sn = sn;
	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), cmd, len);

	int n = sizeof(CmdHeader);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = BaseAPI::stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	DWORD nr = 0;
	WriteFile(hCom, buffer, len2, &nr, NULL);

	char buf[2048];
	int index = 100;
	//4C 48 BC FF   xx xx xx xx  result 
	//读取之后的10*2048个长度，如果不存在即判定失败
	while (index--)
	{
		ReadFile(hCom, buf, sizeof(buf), &nr, NULL);
		while (nr < (int)sizeof(buf))
		{
			DWORD n;
			ReadFile(hCom, buf + nr, sizeof(buf) - nr, &n, NULL);
			if (n > 0) nr += n;
		}

		for (int i = 0; i < (int)sizeof(buf) - result_len; i++)
		{
			//帧头判定  获取雷达参数/设置雷达参数
			if (buf[i] == 0x4C && buf[i + 1] == 0x48)
			{
				if ((buf[i + 2] == (signed char)0xAC && buf[i + 3] == (signed char)0xB8) || (buf[i + 2] == (signed char)0xAC && buf[i + 3] == (signed char)0xff))
				{
					//printf("%02x  %02x\n", buf[i + 2], buf[i + 3]);
					//随机码判定
					unsigned int packSN = ((unsigned char)buf[i + 5] << 8) | (unsigned char)buf[i + 4];
					if (packSN != sn)
						continue;

					memcpy(result, buf + i + 8, result_len);
					return true;
				}
			}
		}
	}
	printf("read %d bytes, not found %s\n", nr, cmd);
	return false;
}
void send_cmd_uart(int hCom, int mode, int sn, int len, const char* cmd)
{
	char buffer[2048];
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = mode;
	hdr->sn = sn;
	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), cmd, len);

	int n = sizeof(CmdHeader);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = BaseAPI::stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	DWORD nr = 0;
	WriteFile((HANDLE)hCom, buffer, len2, &nr, NULL);
}

int setup_lidar_uart(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char* version)
{

	char buf[32];
	DWORD nr = 0;
	//set  lidar start up
	WriteFile(hCom, "LSTARH", 6, &nr, NULL);

	for (int i = 0; i < 300 && nr <= 0; i++) {
		Sleep(10);
		ReadFile(hCom, buf, sizeof(buf), &nr, NULL);
	}
	if (nr <= 0) {
		printf("serial port seem not working\n");
		return -1;
	}
	//Get  hardware version info
	if (uart_talk(hCom, 6, "LXVERH", 14, "MOTOR VERSION:", 15, buf))
	{
		memcpy(version, buf, 12);
		printf("set LiDAR LXVERH  OK\n");
	}
	//Set the lidar returned data unit   CM or MM
	if (uart_talk(hCom, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 6, "LiDAR ", 5, buf))
	{
		printf("set LiDAR unit OK\n");
	}
	//set lidar confidence state   LNCONH close   LOCONH open
	if (uart_talk(hCom, 6, with_confidence == 0 ? "LNCONH" : "LOCONH", 6, "LiDAR ", 5, buf))
	{
		printf("set LiDAR confidence OK\n");
	}
	//set  de-deshadow state    LFFF0H:close  LFFF1H:open
	if (uart_talk(hCom, 6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 6, "LiDAR ", 0, NULL))
	{
		printf("set deshadow OK\n");
	}
	//set  de-smooth     LSSS0H:close   LSSS1H:open
	if (uart_talk(hCom, 6, with_smooth == 0 ? "LSSS0H" : "LSSS1H", 6, "LiDAR ", 0, NULL))
	{
		printf("set smooth to OK\n");
	}
	//LSRES:000H :set default Angular resolution  LSRES:001H :fix Angular resolution  
	if (resample == 0)
		strcpy_s(buf, 30, "LSRES:000H");
	else if (resample == 1)
		strcpy_s(buf, 30, "LSRES:001H");
	else
		buf[0] = 0;

	if (buf[0]) {
		if (uart_talk(hCom, 10, buf, 15, "set resolution ", 0, NULL))
		{
			printf("set LiDAR resample OK\n");
		}
	}
	// setup rpm  (The specific model range is different)
	if (init_rpm > 300 && init_rpm < 3000)
	{
		for (int i = 0; i < 5; i++)
		{
			char cmd[32];
			sprintf(cmd, "LSRPM:%dH", init_rpm);
			if (uart_talk(hCom, strlen(cmd), cmd, 3, "RPM", 0, NULL))
			{
				printf("set RPM to %d  OK\n", init_rpm);
				break;
			}
		}
	}
	return 0;
}
int setup_lidar_vpc(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char* version)
{
	char buf[64];
	DWORD nr = 0;

	if (uart_talk2(hCom, 0x0043, rand(), 6, "LSTARH", 3, buf))
	{
		printf("set LSTARH ,result:%s\n", buf);
	}

	for (int i = 0; i < 300 && nr <= 0; i++) {
		Sleep(10);

		ReadFile(hCom, buf, sizeof(buf), &nr, NULL);
	}

	if (nr <= 0) {
		printf("serial port seem not working\n");
		return -1;
	}
	if (uart_talk2(hCom, 0x0043, rand(), 6, "LXVERH", 63, buf))
	{
		memcpy(version, buf, 64);
		printf("set LiDAR LXVERH  OK  %s\n", version);
	}
	if (uart_talk2(hCom, 0x0043, rand(), 6, "LUUIDH", 32, buf))
	{
		char tmp[32] = { 0 };
		memcpy(tmp, buf + 10, sizeof(tmp - 10));
		printf("set LiDAR LUUIDH  OK  %s\n", tmp);
		//printf("GetDevInfo_MSG success\n");
	}

	//unsupport
	/*if (!uart_talk2(hCom, 0x0043, rand(), 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 10, "SET LiDAR ", 32, buf))
	{
		printf("set LiDAR unit failed %s\n", buf);
	}
	if (uart_talk2(hCom, 0x0043, rand(), 6, with_confidence == 0 ? "LNCONH" : "LOCONH", 6, "LiDAR ", 5, buf))
	{
		printf("set LiDAR confidence to %s\n", buf);
	}
	*/
	if (uart_talk2(hCom, 0x0043, rand(), 6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 3, buf))
	{
		printf("set deshadow to %d,result:%s\n", with_deshadow, buf);
	}

	if (uart_talk2(hCom, 0x0043, rand(), 6, with_smooth == 0 ? "LSSS0H" : "LSSS1H", 3, buf))
	{
		printf("set smooth to %d,result:%s\n", with_smooth, buf);
	}

	char cmd[32];
	if (resample == 0)
		strcpy_s(cmd, 30, "LSRES:000H");
	else if (resample == 1)
		strcpy_s(cmd, 30, "LSRES:001H");
	else
		cmd[0] = 0;

	if (cmd[0]) {

		if (uart_talk2(hCom, 0x0043, rand(), 10, cmd, 3, buf))
		{
			printf("set LiDAR resample to %d,result:%s\n", resample, buf);
		}
	}

	if (init_rpm > 300 && init_rpm < 3000)
	{
		for (int i = 0; i < 5; i++)
		{
			char cmd[32];
			sprintf(cmd, "LSRPM:%dH", init_rpm);
			if (uart_talk2(hCom, 0x0043, rand(), strlen(cmd), cmd, 3, buf))
			{
				printf("set RPM to %s,,result:%s\n", cmd, buf);
				break;
			}
		}
	}
	return 0;

}

HANDLE open_serial_port(RunConfig& cfg)
{
	cfg.should_quit = false;
	char path[32];
	sprintf_s(path, 30, "\\\\.\\%s", cfg.port);
	// Open the serial port.
	HANDLE hPort = CreateFile(path,
		GENERIC_READ | GENERIC_WRITE, // Access (read-write) mode
		FILE_SHARE_READ | FILE_SHARE_WRITE,            // Share mode
		NULL,         // Pointer to the security attribute
		OPEN_EXISTING,// How to open the serial port
		0,            // Port attributes
		NULL);        // Handle to port with attribute

	if (hPort == NULL || hPort == INVALID_HANDLE_VALUE)
	{
		//MessageBox(0, "can not open port", name, MB_OK);
		return 0;
	}
	DCB PortDCB;
	// Initialize the DCBlength member. 
	PortDCB.DCBlength = sizeof(DCB);
	// Get the default port setting information.
	GetCommState(hPort, &PortDCB);

	// Change the DCB structure settings.
	PortDCB.BaudRate = cfg.baud_rate;// 115200;              // Current baud 
	PortDCB.fBinary = TRUE;               // Binary mode; no EOF check 
	PortDCB.fParity = TRUE;               // Enable parity checking 
	PortDCB.fOutxCtsFlow = FALSE;         // No CTS output flow control 
	PortDCB.fOutxDsrFlow = FALSE;         // No DSR output flow control 
	PortDCB.fDtrControl = DTR_CONTROL_ENABLE;
	// DTR flow control type 
	PortDCB.fDsrSensitivity = FALSE;      // DSR sensitivity 
	PortDCB.fTXContinueOnXoff = TRUE;     // XOFF continues Tx 
	PortDCB.fOutX = FALSE;                // No XON/XOFF out flow control 
	PortDCB.fInX = FALSE;                 // No XON/XOFF in flow control 
	PortDCB.fErrorChar = FALSE;           // Disable error replacement 
	PortDCB.fNull = FALSE;                // Disable null stripping 
	PortDCB.fRtsControl = RTS_CONTROL_ENABLE;
	// RTS flow control 
	PortDCB.fAbortOnError = FALSE;        // Do not abort reads/writes on 
										  // error
	PortDCB.ByteSize = 8;                 // Number of bits/byte, 4-8 
	PortDCB.Parity = NOPARITY;            // 0-4=no,odd,even,mark,space 
	PortDCB.StopBits = ONESTOPBIT;        // 0,1,2 = 1, 1.5, 2 

										  // Configure the port according to the specifications of the DCB 
										  // structure.
	if (!SetCommState(hPort, &PortDCB))
	{
		//MessageBox(0, "Unable to configure the serial port", "error", MB_OK);
		CloseHandle(hPort);
		return NULL;
	}
	// Retrieve the timeout parameters for all read and write operations
	// on the port. 
	COMMTIMEOUTS CommTimeouts;
	GetCommTimeouts(hPort, &CommTimeouts);

	// Change the COMMTIMEOUTS structure settings.
	CommTimeouts.ReadIntervalTimeout = MAXDWORD;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadTotalTimeoutConstant = 0;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;//10;  
	CommTimeouts.WriteTotalTimeoutConstant = 0;//1000;    

											   // Set the timeout parameters for all read and write operations
											   // on the port. 
	if (!SetCommTimeouts(hPort, &CommTimeouts))
	{
		// Could not set the timeout parameters.
		//MessageBox(0, "Unable to set the timeout parameters", "error", MB_OK);
		CloseHandle(hPort);
		return NULL;
	}

	return hPort;
}
DWORD  WINAPI lidar_thread_proc_uart(void* param) 
{
	MSG msg;
	PeekMessage(&msg, NULL, GetDevInfo_MSG, Set_ZONE_MSG, PM_NOREMOVE);
	bool isrun = false;//初始化运行雷达是否正常运行标志位
	int zoneFlag = 0;//读写防区标志位    0为正常运行  1为读  2为写
	int zoneSN = rand();
	RunConfig* cfg = (RunConfig*)param;
	CallBack_Uart = send_cmd_uart;
	ZoneAlarm* zonealarm = new ZoneAlarm(cfg->fd, false, (void*)CallBack_Uart);
	PointData tmp;//临时存储结构体变量
	 //默认启动雷达
	DWORD dwWritenSize = 0;
	FanSegment** fan_segs = new FanSegment*;
	*fan_segs = NULL;
	std::vector<RawData*> whole_datas;
	if (strcmp(cfg->type, "uart") == 0)
		setup_lidar_uart((void*)cfg->fd, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->version);
	else
		setup_lidar_vpc((void*)cfg->fd, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->version);

	INFO_PR("\033[1;32m----> All params set OK ! Start parser data.\033[0m\n");
	/*
	 * 4, read and parser data
	 */
	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;
	bool is_pack;
	int fan_span = 360;	 // 36 degrees
	int index = 0;
	while (!cfg->should_quit)
	{
		DWORD nr = 0;
		if (cfg->fd > 0)
		{
			ReadFile((void*)cfg->fd, buf + buf_len, BUF_SIZE - buf_len, &nr, NULL);
			buf_len += nr;
		}
		if (buf_len > 0)
		{
			//buf_len += new_data;
			int consume = 0; // in order to compute the rest of data after every parser process
			int res = -1;
			//读取防区
			if (zoneFlag == 1)
			{
				res = zonealarm->getZoneRev(buf, buf_len, zoneSN, consume);
				if (res != 0)
				{
					RecvZoneDatas* rev = new RecvZoneDatas;
					memcpy(rev, zonealarm->getZoneResult(), sizeof(RecvZoneDatas));
					rev->result = res;
					if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)rev, 0))
					{
						DEBUG_PR("threadson post message  get_ZONE_MSG failed,errno:%d\n", ::GetLastError());
					}
					zoneFlag = 0;
				}
			}
			//设置防区
			else if (zoneFlag == 2)
			{
				res = zonealarm->setZoneRev(buf, buf_len, zoneSN, consume);
				if (res != 0)
				{
					int* rev = new int;
					*rev = res;
					if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)rev, 0))
					{
						DEBUG_PR("threadson post message  Set_ZONE_MSG failed,errno:%d\n", ::GetLastError());
					}
					char tmpbuf[16] = { 0 };
					uart_talk2((void*)cfg->fd, 0x0043, rand(), 6, "LSTARH", 3, tmpbuf);
					zoneFlag = 0;
				}

			}//点云数据
			else
			{
				//雷达点云数据处理
				RawData dat;
				LidarMsgHdr zone;
				memset(&zone, 0, sizeof(LidarMsgHdr));
				if (cfg->data_bytes == 3)
				{
					is_pack = ParseAPI::parse_data_x(buf_len, buf,
						fan_span, cfg->unit_is_mm, cfg->with_confidence,
						dat, consume, cfg->with_chk, zone, fan_segs);
				}
				else
				{
					is_pack = ParseAPI::parse_data(buf_len, buf,
						fan_span, cfg->unit_is_mm, cfg->with_confidence,
						dat, consume, cfg->with_chk);
				}
				if (is_pack)
				{
					if (cfg->output_scan)
					{
						if (zone.timestamp != 0)
						{
							memcpy(&cfg->zone, &zone, sizeof(LidarMsgHdr));
						}
						memset(&tmp, 0, sizeof(PointData));
						//printf("%d\n", dat.N);
						UserAPI::whole_data_process(dat, cfg->from_zero, cfg->collect_angle, cfg->output_file, tmp, whole_datas);

						//执行回调函数
						if (tmp.N > 0)
						{

							//E100系列
							if (cfg->shadows_filter.enable)
							{
								int nr = AlgorithmAPI_E100::ShadowsFilter(&tmp, cfg->shadows_filter);
							}
							if (cfg->median_filter.enable)
							{
								int nr = AlgorithmAPI_E100::MedianFilter(&tmp, cfg->median_filter);
							}
							((void(*)(int, void*))cfg->callback)(1, &tmp);
							memcpy(&cfg->pointdata, &tmp, sizeof(PointData));
						}
					}
					//证明雷达已经正常运行
					if (isrun == false) //set thread start event 
					{
						if (cfg->hStartEvent == 0 || SetEvent(cfg->hStartEvent) == 0)
						{
							printf("set start event failed,errno:%d\n", ::GetLastError());
							return 1;
						}
						else
						{
							isrun = true;
						}

					}
				}
			}
			if (consume > 0)
			{
				// data is not whole fan,drop it
				if (!is_pack)
				{
					DEBUG_PR("drop %d bytes: %02x %02x %02x %02x %02x %02x",
						consume,
						buf[0], buf[1], buf[2],
						buf[3], buf[4], buf[5]);
				}

				for (int i = consume; i < buf_len; i++)
					buf[i - consume] = buf[i];
				buf_len -= consume;
			}

		}
		if (PeekMessage(&msg, NULL, GetDevInfo_MSG, Set_ZONE_MSG, PM_REMOVE)) //get msg from message queue
		{
			switch (msg.message)
			{
			case GetDevInfo_MSG:
			{
				EEpromV101* eepromv101 = new EEpromV101;
				memset(eepromv101, 0, sizeof(EEpromV101));
				char buf[20] = { 0 };
				if (strcmp(cfg->type, "uart") == 0)
				{
					if (!uart_talk((HANDLE)cfg->fd, 6, "LUUIDH", 11, "PRODUCT SN:", 9, buf))
					{
						DEBUG_PR("uart GetDevInfo_MSG failed\n");
						break;
					}
					memcpy(eepromv101->dev_sn, buf, sizeof(buf));
				}
				else if (strcmp(cfg->type, "vpc") == 0)
				{
					if (!uart_talk3((HANDLE)cfg->fd, 0x4753, rand(), 6, "LUUIDH", sizeof(EEpromV101), eepromv101))
					{
						DEBUG_PR("vpc GetDevInfo_MSG failed\n");
						break;
						//((void (*)(int, void*))cfg->callback)(2, eepromv101);
					}
					memcpy(eepromv101, eepromv101, sizeof(EEpromV101));
				}

				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)eepromv101, 0))//post thread msg
					printf("threadson post message GetDevInfo_MSG failed,errno:%d\n", ::GetLastError());
				break;
			}
			case SetDevInfo_MSG:
			{
				DevData* devdata = (DevData*)msg.wParam;
				setup_lidar_extre(cfg->type,cfg->fd,"",0, * devdata);
				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)devdata, 0))
				{
					printf("threadson post message failed,errno:%d\n", ::GetLastError());
					delete devdata;
				}
				break;
			}
			case ctrl_MSG:
			{
				char* cmd = (char*)msg.wParam;
				char buf1[32] = { 0 };
				if (strcmp(cfg->type, "uart") == 0)
				{
					WriteFile((void*)cfg->fd, cmd, 6, &dwWritenSize, NULL);
				}
				else if (strcmp(cfg->type, "vpc") == 0)
				{
					//特殊说明，虚拟串口的方式，重启命令只能调用一次，即雷达断开连接
					if (uart_talk2((void*)cfg->fd, 0x0043, rand(), 6, cmd, 3, buf1))
					{
						printf("set LiDAR %s to %s\n", cmd, buf1);
					}
				}

				if (!PostThreadMessage(msg.lParam, msg.message, NULL, 0))
				{
					DEBUG_PR("threadson post message ctrl_MSG failed,errno:%d\n", ::GetLastError());
				}
				delete[]cmd;
				break;
			}
			case Print_Point_MSG:
			{
				int* cmd = (int*)msg.wParam;
				cfg->output_scan = *cmd;
				if (!PostThreadMessage(msg.lParam, msg.message, NULL, 0))
				{
					DEBUG_PR("threadson post message  print_MSG failed,errno:%d\n", ::GetLastError());
				}
				delete cmd;
				break;
			}
			/*case Print_TimeStamp_MSG:
			{
				int *cmd = (int*)msg.wParam;
				timeprint = *cmd;
				int *rev =new int;
				*rev = 0;
				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)rev, 0))
				{
					DEBUG_PR("threadson post message  Print_TimeStamp_MSG failed,errno:%d\n", ::GetLastError());
				}
				delete[]cmd;
				break;
			}*/
			case Get_ZONE_MSG:
			{
				zonealarm->getZone(zoneSN);
				zoneFlag = 1;
				break;
			}
			case Set_ZONE_MSG:
			{
				char tmpbuf[3] = { 0 };
				uart_talk2((void*)cfg->fd, 0x0043, rand(), 6, "LSTOPH", 3, tmpbuf);
				zones* cmd = (zones*)msg.wParam;
				zonealarm->setZone(*cmd, zoneSN);
				zoneFlag = 2;
				delete[]cmd;
				break;
			}
			}
		}
	}
	if (cfg->fd > 0)
		CloseHandle((void*)cfg->fd);
	return 0;

}


int open_socket_port(RunConfig& cfg)
{

	cfg.should_quit = false;
	WSADATA   wsda; //   Structure   to   store   info   
	WSAStartup(MAKEWORD(2, 2), &wsda);
	int fd_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd_udp < 0)
	{
		return OPEN_SOCKET_PORT_FAILED;
	}
	// open UDP port
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(cfg.local_port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int rt = ::bind(fd_udp, (struct sockaddr*)&addr, sizeof(addr));
	if (rt != 0)
	{
		DEBUG_PR("\033[1;31m----> bind port %d failed.\033[0m\n", cfg.local_port);
		return BIND_SOCKET_PORT_FAILED;
	}

	INFO_PR("\033[1;32m----> start udp %s:%d udp %d\033[0m\n", cfg.lidar_ip, cfg.lidar_port, fd_udp);
	return fd_udp;
}
#endif
#ifdef __unix__
bool uart_talk(int fd, int n, const char *cmd,
			   int nhdr, const char *hdr_str,
			   int nfetch, char *fetch)
{
	printf("send command : %s\n", cmd);
	write(fd, cmd, n);

	char buf[2048];
	int nr = read(fd, buf, sizeof(buf));
	while (nr < (int)sizeof(buf))
	{
		int n = read(fd, buf + nr, sizeof(buf) - nr);
		if (n > 0)
			nr += n;
	}
	for (int i = 0; i < (int)sizeof(buf) - nhdr - nfetch; i++)
	{
		if (memcmp(buf + i, hdr_str, nhdr) == 0)
		{
			if (nfetch > 0)
			{
				memcpy(fetch, buf + i + nhdr, nfetch);
				fetch[nfetch] = 0;
			}
			return true;
		}
	}
#if 0
	char path[256];
	sprintf(path, "/tmp/%s.dat", hdr_str);
	FILE* fp = fopen(path, "wb");
	if (fp) {
		fwrite(buf, 1, sizeof(buf), fp);
		fclose(fp);
	}
#endif

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return false;
}
bool uart_talk2(int hCom, int mode, int sn, int len, const char *cmd, int nfetch, char *fetch)
{
	printf("USB send command : %s\n", cmd);
	char buffer[2048];
	CmdHeader *hdr = (CmdHeader *)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = mode;
	hdr->sn = sn;
	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), cmd, len);

	int n = sizeof(CmdHeader);
	unsigned int *pcrc = (unsigned int *)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = BaseAPI::stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	int nr = 0;
	write(hCom, buffer, len2);

	char buf[2048];
	int index = 10;
	// 4C 48 BC FF   xx xx xx xx  result
	//读取之后的10*2048个长度，如果不存在即判定失败
	while (index--)
	{
		int nr = read(hCom, buf, sizeof(buf));
		// printf("%d %d \n",sizeof(buf),nr);
		while (nr < sizeof(buf))
		{
			int n = 0;
			n = read(hCom, buf + nr, sizeof(buf) - nr);
			if (n > 0)
				nr += n;
		}
		// if (nr < 0)
		// {
		// 	DEBUG_PR("read port %d error %d\n", hCom, nr);
		// 	break;
		// }
		// if (nr == 0)
		// 	continue;

		for (int i = 0; i < (int)sizeof(buf) - nfetch; i++)
		{
			if (buf[i] == 0x4C && buf[i + 1] == 0x48 && buf[i + 2] == (signed char)0xBC && buf[i + 3] == (signed char)0xFF)
			{

				for (int j = 0; j < nfetch; j++)
				{
					if ((buf[i + j + 8] >= 33 && buf[i + j + 8] <= 127))
					{
						fetch[j] = buf[i + j + 8];
					}
					else
					{
						fetch[j] = ' ';
					}
				}
				fetch[nfetch] = 0;
				return true;
			}
		}
	}
	printf("read %d bytes, not found %s\n", nr, cmd);
	return false;
}
bool uart_talk3(int hCom, int mode, int sn, int len, const char *cmd, int result_len, void *result)
{
	printf("USB send command : %s\n", cmd);
	char buffer[2048];
	CmdHeader *hdr = (CmdHeader *)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = mode;
	hdr->sn = sn;
	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), cmd, len);

	int n = sizeof(CmdHeader);
	unsigned int *pcrc = (unsigned int *)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = BaseAPI::stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	int nr = 0;
	write(hCom, buffer, len2);

	unsigned char buf[2048];
	int index = 10;
	// 4C 48 BC FF   xx xx xx xx  result
	//读取之后的10*2048个长度，如果不存在即判定失败
	while (index--)
	{
		int nr = read(hCom, buf, sizeof(buf));
		// printf("%d %d \n",sizeof(buf),nr);
		while (nr < sizeof(buf))
		{
			int n = 0;
			n = read(hCom, buf + nr, sizeof(buf) - nr);
			if (n <= 0)
				continue;
			nr += n;
		}
		if (nr < 0)
		{
			DEBUG_PR("read port %d error %d\n", hCom, nr);
			break;
		}
		if (nr == 0)
			continue;

		for (int i = 0; i < sizeof(buf) - result_len; i++)
		{
			//帧头判定  获取雷达参数/设置雷达参数
			if (buf[i] == 0x4C && buf[i + 1] == 0x48)
			{
				printf("%02x  %02x  %02x  %02x\n", buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
				if ((buf[i + 2] == 0xAC && buf[i + 3] == 0xB8) || (buf[i + 2] == 0xAC && buf[i + 3] == 0xff))
				{

					//随机码判定
					// unsigned int packSN = ((unsigned char)buf[i + 5] << 8) | (unsigned char)buf[i + 4];
					// printf("12345:%ld  %ld\n", packSN, sn);
					//  if (packSN != sn)
					//  	continue;

					memcpy(result, buf + i + 8, result_len);
					return true;
				}
			}
		}
	}
	printf("read %d bytes, not found %s\n", nr, cmd);
	return false;
}
int strip(const char *s, char *buf)
{
	int len = 0;
	for (int i = 0; s[i] != 0; i++)
	{
		if (s[i] >= 'a' && s[i] <= 'z')
			buf[len++] = s[i];
		else if (s[i] >= 'A' && s[i] <= 'Z')
			buf[len++] = s[i];
		else if (s[i] >= '0' && s[i] <= '9')
			buf[len++] = s[i];
		else if (len > 0)
			break;
	}
	buf[len] = 0;
	return len;
}
void *lidar_thread_proc_uart(void *param)
{
	int msgrec = -1; //消息队列返回值
	USER_MSG msg;	 //消息队列
	bool timeprint = false;
	RunConfig *cfg = (RunConfig *)param;
	PointData tmp; //临时存储结构体变量
	memset(&tmp, 0, sizeof(PointData));
	int zoneFlag = 0; //读写防区标志位    0为正常运行  1为读  2为写
	int zoneSN = rand();
	CallBack_Uart = send_cmd_uart;
    FanSegment**fan_segs=new FanSegment*;
	*fan_segs = NULL;
	std::vector<RawData*> whole_datas;
	ZoneAlarm *zonealarm = new ZoneAlarm(cfg->fd, false, (void *)CallBack_Uart);
	if (strcmp(cfg->type, "uart") == 0)
		setup_lidar_uart(cfg->fd, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->version);
	else
		setup_lidar_vpc(cfg->fd, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->version);
	INFO_PR("\033[1;32m----> All params set OK ! Start parser data.\033[0m\n");

	/*
	 * 4, read and parser data
	 */
	unsigned char *buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;
	bool is_pack;
	FILE *fp_rec = NULL; // fopen("/tmp/rec.dat", "ab");
	int fan_span = 360;	 // 36 degrees
	while (!cfg->should_quit)
	{
		/*
		 * check fd_uart is ok
		 */
		fd_set fds;
		FD_ZERO(&fds);

		FD_SET(cfg->fd, &fds);

		struct timeval to = {1, 1};
		int ret = select(cfg->fd + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
		{
			DEBUG_PR("select error\n");
			return NULL;
		}

		// read data process
		if (cfg->fd > 0 && FD_ISSET(cfg->fd, &fds))
		{
			int nr = read(cfg->fd, buf + buf_len, BUF_SIZE - buf_len);
			if (nr < 0)
			{
				DEBUG_PR("read port %d error %d\n", buf_len, nr);
				break;
			}
			if (nr == 0)
				continue;
			if (nr > 0 && fp_rec)
			{
				fwrite(buf + buf_len, 1, nr, fp_rec);
				fflush(fp_rec);
			}
			buf_len += nr;
		}

		if (buf_len > 0)
		{
			int consume = 0; // in order to compute the rest of data after every parser process
			int res = -1;
			if (zoneFlag == 1)
			{

				res = zonealarm->getZoneRev(buf, buf_len, zoneSN, consume);
				if (res != 0)
				{
					RecvZoneDatas *rev = new RecvZoneDatas;
					memcpy(rev, zonealarm->getZoneResult(), sizeof(RecvZoneDatas));
					rev->result = res;
					USER_MSG msg2;
					msg2.type = 2;
					msg2.cmd.type2 = msg.cmd.type2;
					memcpy(msg2.cmd.str, rev, sizeof(RecvZoneDatas));
					msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
					delete rev;
					zoneFlag = 0;
				}
			}
			//设置防区
			else if (zoneFlag == 2)
			{
				res = zonealarm->setZoneRev(buf, buf_len, zoneSN, consume);
				if (res != 0)
				{
					int *rev = new int;
					*rev = res;
					USER_MSG msg2;
					msg2.type = 2;
					msg2.cmd.type2 = msg.cmd.type2;
					memcpy(msg2.cmd.str, rev, sizeof(int));
					msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
					char tmpbuf[16] = {0};
					uart_talk2(cfg->fd, 0x0043, rand(), 6, "LSTARH", 3, tmpbuf);
					zoneFlag = 0;
				}
			}
			else
			{
				// in order to compute the rest of data after every parser process
				RawData dat;
				LidarMsgHdr zone;
				memset(&zone, 0, sizeof(LidarMsgHdr));
				if (cfg->data_bytes == 3)
				{
					is_pack = ParseAPI::parse_data_x(buf_len, buf,
										   fan_span, cfg->unit_is_mm, cfg->with_confidence,
										   dat, consume, cfg->with_chk, zone,fan_segs);
					//printf("%d %s %d\n", __LINE__, __FUNCTION__, is_pack);
				}
				else
				{
					is_pack = ParseAPI::parse_data(buf_len, buf,
										 fan_span, cfg->unit_is_mm, cfg->with_confidence,
										 dat, consume, cfg->with_chk);
				}
				// data output
				if (is_pack)
				{
					if (cfg->output_scan)
					{
						//！！！User needs to extract data operation, you can refer to this function，For details about other printing operations, please refer to the user.cpp file
						if (zone.timestamp != 0)
						{
							memcpy(&cfg->zone, &zone, sizeof(LidarMsgHdr));
						}
						memset(&tmp, 0, sizeof(PointData));
						UserAPI::whole_data_process(dat,  cfg->from_zero, cfg->collect_angle, cfg->output_file,  tmp,whole_datas);
						//执行回调函数
						if (tmp.N > 0)
						{
                            if (cfg->shadows_filter.enable)
							{
								AlgorithmAPI_E100::ShadowsFilter(&tmp, cfg->shadows_filter);
							}
							if (cfg->median_filter.enable)
							{
								AlgorithmAPI_E100::MedianFilter(&tmp, cfg->median_filter);
							}
							((void (*)(int, void *))cfg->callback)(1, &tmp);
							memcpy(&cfg->pointdata, &tmp, sizeof(PointData));
						}
					}
				}
			}
			if (consume > 0)
			{
				// data is not whole fan,drop it
				if (!is_pack)
				{
					DEBUG_PR("drop %d bytes: %02x %02x %02x %02x %02x %02x",
							 consume,
							 buf[0], buf[1], buf[2],
							 buf[3], buf[4], buf[5]);
				}

				for (int i = consume; i < buf_len; i++)
					buf[i - consume] = buf[i];
				buf_len -= consume;
			}
		}
		if (msgrcv(cfg->thread_ID[1], &msg, sizeof(msg.cmd), 1, IPC_NOWAIT) >= 0)
		{
			switch (msg.cmd.type2)
			{
			case GetDevInfo_MSG:
			{
				EEpromV101 *eepromv101 = new EEpromV101;
				memset(eepromv101, 0, sizeof(EEpromV101));
				char buf[20] = {0};
				if (strcmp(cfg->type, "uart") == 0)
				{
					if (uart_talk(cfg->fd, 6, "LUUIDH", 11, "PRODUCT SN:", 16, buf))
					{
						printf("uart GetDevInfo_MSG %d\n",buf);
					}
					else if (!uart_talk(cfg->fd, 6, "LUUIDH", 10, "VENDOR ID:", 16, buf))
					{
						printf("uart GetDevInfo_MSG %d\n",buf);
					}
					memcpy(eepromv101->dev_sn, buf, sizeof(buf));
				}
				else if (strcmp(cfg->type, "vpc") == 0)
				{
					if (!uart_talk3(cfg->fd, 0x4753, rand(), 6, "LUUIDH", sizeof(EEpromV101), eepromv101))
					{
						DEBUG_PR("vpc GetDevInfo_MSG failed\n");
						break;
						//((void (*)(int, void*))cfg->callback)(2, eepromv101);
					}
					memcpy(eepromv101, eepromv101, sizeof(EEpromV101));
				}
				//printf("%d %s %s\n", __LINE__,__FUNCTION__,eepromv101->dev_sn);
				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				memcpy(msg2.cmd.str, eepromv101, sizeof(EEpromV101));
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				delete eepromv101;
				break;
			}
			case SetDevInfo_MSG:
			{
				DevData devdata;
				memcpy(&devdata, &msg.cmd.str, sizeof(DevData));
				setup_lidar_extre(cfg->type,cfg->fd,"",0, devdata);
				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				memcpy(msg2.cmd.str, &devdata, sizeof(DevData));
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				break;
			}
			case ctrl_MSG:
			{
				char cmd[7] = {0};
				char buf1[32] = {0};
				memcpy(&cmd, &msg.cmd.str, sizeof(cmd));
				if (strcmp(cfg->type, "uart") == 0)
				{
					write(cfg->fd, cmd, 6);
				}
				else if (strcmp(cfg->type, "vpc") == 0)
				{
					//特殊说明，虚拟串口的方式，重启命令只能调用一次，即雷达断开连接
					if (uart_talk2(cfg->fd, 0x0043, rand(), 6, cmd, 3, buf1))
					{
						printf("set LiDAR %s to %s\n", cmd, buf1);
					}
				}

				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				break;
			}
			case Print_Point_MSG:
			{
				char tmp[2] = {0};
				strcpy(tmp, msg.cmd.str);
				if (tmp[0] - '0')
					cfg->output_scan = true;
				else
					cfg->output_scan = false;

				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				msg2.cmd.str[0] = '1';
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				break;
			}
			case Print_TimeStamp_MSG:
			{
				char tmp[2] = {0};
				strcpy(tmp, msg.cmd.str);
				if (tmp[0] - '0')
					timeprint = true;
				else
					timeprint = false;

				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				msg2.cmd.str[0] = '0';
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				break;
			}
			case Get_ZONE_MSG:
			{
				zonealarm->getZone(zoneSN);
				zoneFlag = 1;
				break;
			}
			case Set_ZONE_MSG:
			{
				char tmpbuf[12] = {0};
				uart_talk2(cfg->fd, 0x0043, rand(), 6, "LSTOPH", 3, tmpbuf);
				zones *cmd = new zones;
				memcpy(cmd, &msg.cmd.str, sizeof(zones));

				zonealarm->setZone(*cmd, zoneSN);
				zoneFlag = 2;
				delete[] cmd;
				break;
			}
			}
		}
	}
	return 0;
}
int setup_lidar_vpc(int hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char *version)
{
	char buf[64];
	int nr = 0;

	if (uart_talk2(hCom, 0x0043, rand(), 6, "LSTARH", 3, buf))
	{
		printf("set LSTARH ,result:%s\n", buf);
	}
	for (int i = 0; i < 300 && nr <= 0; i++)
	{
		sleep(1);
		nr = read(hCom, buf, sizeof(buf));
	}
	if (nr <= 0)
	{
		printf("serial port seem not working\n");
		return -1;
	}
	//硬件版本号
	if (uart_talk2(hCom, 0x0043, rand(), 6, "LXVERH", 64, buf))
	{
		memcpy(version, buf, 64);
		printf("set LiDAR LXVERH  OK  %s\n", version);
	}
	if (uart_talk2(hCom, 0x0043, rand(), 6, "LUUIDH", 32, buf))
	{
		char tmp[32] = {0};
		memcpy(tmp, buf + 10, sizeof(tmp - 10));
		printf("set LiDAR LUUIDH  OK  %s\n", tmp);
		// printf("GetDevInfo_MSG success\n");
	}

	// unsupport
	/*if (!uart_talk2(hCom, 0x0043, rand(), 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 10, "SET LiDAR ", 32, buf))
	{
		printf("set LiDAR unit failed %s\n", buf);
	}
	if (uart_talk2(hCom, 0x0043, rand(), 6, with_confidence == 0 ? "LNCONH" : "LOCONH", 6, "LiDAR ", 5, buf))
	{
		printf("set LiDAR confidence to %s\n", buf);
	}
	*/
	if (uart_talk2(hCom, 0x0043, rand(), 6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 3, buf))
	{
		printf("set deshadow to %d,result:%s\n", with_deshadow, buf);
	}

	if (uart_talk2(hCom, 0x0043, rand(), 6, with_smooth == 0 ? "LSSS0H" : "LSSS1H", 3, buf))
	{
		printf("set smooth to %d,result:%s\n", with_smooth, buf);
	}

	char cmd[32];
	if (resample == 0)
		strcpy(cmd, "LSRES:000H");
	else if (resample == 1)
		strcpy(cmd, "LSRES:001H");
	else if (resample > 100 && resample < 1000)
		sprintf(cmd, "LSRES:%03dH", resample);
	else
		cmd[0] = 0;

	if (cmd[0])
	{

		if (uart_talk2(hCom, 0x0043, rand(), 10, cmd, 3, buf))
		{
			printf("set LiDAR resample to %d,result:%s\n", resample, buf);
		}
	}

	if (init_rpm > 300 && init_rpm < 3000)
	{
		for (int i = 0; i < 10; i++)
		{
			char cmd[32];
			sprintf(cmd, "LSRPM:%dH", init_rpm);
			if (uart_talk2(hCom, 0x0043, rand(), strlen(cmd), cmd, 3, buf))
			{
				printf("set RPM to %s,,result:%s\n", cmd, buf);
				break;
			}
		}
	}
	return 0;
}
int setup_lidar_uart(int fd_uart, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char *version)
{
	char buf[32];
	int nr = 0;
	write(fd_uart, "LSTARH", 6);
	for (int i = 0; i < 300 && nr <= 0; i++)
	{
		usleep(10000);
		nr = read(fd_uart, buf, sizeof(buf));
	}
	if (nr <= 0)
	{
		printf("serial port seem not working\n");
		close(fd_uart);
		return READ_UART_FAILED;
	}
	//硬件版本号
	if (uart_talk(fd_uart, 6, "LXVERH", 14, "MOTOR VERSION:", 15, buf))
	{
		memcpy(version, buf, 12);
		printf("set LiDAR LXVERH  OK\n");
	}

	if (uart_talk(fd_uart, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH",
				  10, "SET LiDAR ", 0, NULL))
	{
		printf("set LiDAR unit OK\n");
	}

	if (uart_talk(fd_uart, 6, with_confidence == 0 ? "LNCONH" : "LOCONH",
				  6, "LiDAR ", 0, NULL))
	{
		printf("set LiDAR confidence OK\n");
	}

	if (uart_talk(fd_uart, 6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H",
				  6, "LiDAR ", 0, NULL))
	{
		printf("set deshadow OK\n");
	}

	if (uart_talk(fd_uart, 6, with_smooth == 0 ? "LSSS0H" : "LSSS1H",
				  6, "LiDAR ", 0, NULL))
	{
		printf("set smooth  OK\n");
	}

	if (resample == 0)
		strcpy(buf, "LSRES:000H");
	else if (resample == 1)
		strcpy(buf, "LSRES:001H");
	else if (resample > 100 && resample < 1000)
		sprintf(buf, "LSRES:%03dH", resample);
	else
		buf[0] = 0;

	if (buf[0])
	{
		char buf2[32];
		if (uart_talk(fd_uart, 10, buf, 15, "set resolution ", 1, buf2))
		{
			printf("set LiDAR resample to %d\n", resample);
		}
	}
	// setup rpm
	if (init_rpm > 300 && init_rpm < 3000)
	{
		for (int i = 0; i < 10; i++)
		{
			char cmd[32];
			sprintf(cmd, "LSRPM:%dH", init_rpm);
			if (uart_talk(fd_uart, strlen(cmd), cmd, 3, "RPM", 5, buf))
			{
				printf("set RPM to %d %s\n", init_rpm, buf);
				break;
			}
		}
	}
	return 0;
}
int open_serial_port(RunConfig &cfg)
{
	cfg.should_quit = false;

	int fd = open(cfg.port, O_RDWR | O_NOCTTY | O_NDELAY);
	// INFO_PR("open_serial_port %d\n",fd);
	if (fd < 0)
	{
		DEBUG_PR("\033[1;31m----> Open %s error\033[0m\n", cfg.port);
		return OPEN_SERIAL_PORT_FAILED;
	}

	int ret;
	struct termios attrs;
	tcflush(fd, TCIOFLUSH);

	/* get current attrs */
	ret = tcgetattr(fd, &attrs);
	if (ret < 0)
	{
		DEBUG_PR("get attrs failed");
		return OPEN_ATTRS_FAILED;
	}

	/* set speed */
	int speed = B230400;
	// if (baudrate == 115200) speed = B115200;

	ret = cfsetispeed(&attrs, speed);  //[baudrate]);
	ret |= cfsetospeed(&attrs, speed); //[baudrate]);

	/* enable recieve and set as local line */
	attrs.c_cflag |= (CLOCAL | CREAD);

	/* set data bits */
	attrs.c_cflag &= ~CSIZE;
	attrs.c_cflag |= CS8;

	/* set parity */
	if (1)
	{							  // parity == UART_POFF) {
		attrs.c_cflag &= ~PARENB; // disable parity
		attrs.c_iflag &= ~INPCK;
	}
	else
	{
		attrs.c_cflag |= (PARENB | PARODD); // enable parity
		attrs.c_iflag |= INPCK;
		// if(parity == UART_PEVEN) attrs.c_cflag &= ~PARODD;
	}

	/* set stop bits */
	attrs.c_cflag &= ~CSTOPB; // 1 stop bit
							  // attrs.c_cflag |= CSTOPB;	// 2 stop bits

	// Disable Hardware flowcontrol
	attrs.c_cflag &= ~CRTSCTS;

	/* set to raw mode, disable echo, signals */
	attrs.c_lflag &= ~(ICANON | ECHO | ECHOE | IEXTEN | ISIG);

	/* set no output process, raw mode */
	attrs.c_oflag &= ~OPOST;
	attrs.c_oflag &= ~(ONLCR | OCRNL);

	/* disable CR map  */
	attrs.c_iflag &= ~(ICRNL | INLCR);
	/* disable software flow control */
	attrs.c_iflag &= ~(IXON | IXOFF | IXANY);

	//	attrs.c_cc[VMIN] = 0;
	//	attrs.c_cc[VTIME] = 10;

	/* flush driver buf */
	tcflush(fd, TCIFLUSH);

	/* update attrs now */
	if (tcsetattr(fd, TCSANOW, &attrs) < 0)
	{
		close(fd);
		DEBUG_PR("tcsetattr err");
		return UPDATE_ATTRS_FAILED;
	}

	if (change_baud(fd, cfg.baud_rate))
	{
		close(fd);
		DEBUG_PR("fail to set baudrate %d", cfg.baud_rate);
		return UPDATE_BANDRATE_FIILED;
	}

	return fd;
}
void send_cmd_uart(int fd, int mode, int sn, int len, const char *cmd)
{
	char buffer[2048];
	CmdHeader *hdr = (CmdHeader *)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = mode;
	hdr->sn = sn;
	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), cmd, len);

	int n = sizeof(CmdHeader);
	unsigned int *pcrc = (unsigned int *)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = BaseAPI::stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	write(fd, buffer, len2);
}
int open_socket_port(RunConfig &cfg)
{
	cfg.should_quit = false;
	int fd_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd_udp < 0)
	{
		return OPEN_SOCKET_PORT_FAILED;
	}
	// open UDP port
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(cfg.local_port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int rt = ::bind(fd_udp, (struct sockaddr *)&addr, sizeof(addr));
	if (rt != 0)
	{
		DEBUG_PR("\033[1;31m----> bind port %d failed.\033[0m\n", cfg.local_port);
		return BIND_SOCKET_PORT_FAILED;
	}

	INFO_PR("\033[1;32m----> start udp %s:%d udp %d\033[0m\n", cfg.lidar_ip, cfg.lidar_port, fd_udp);
	return fd_udp;
}
void *lidar_thread_proc_udp(void *param)
{
	int msgrec = -1; //消息队列返回值
	USER_MSG msg;	 //消息队列
	bool timeprint = false;
	RunConfig *cfg = (RunConfig *)param;
	PointData tmp;
	memset(&tmp, 0, sizeof(PointData));
	int zoneFlag = 0; //读写防区标志位    0为正常运行  1为读  2为写
	int zoneSN = rand();
	CallBack_Udp = send_cmd_udp;
	ZoneAlarm *zonealarm = new ZoneAlarm(cfg->fd, true, cfg->lidar_ip, cfg->lidar_port, (void *)CallBack_Udp);
    FanSegment**fan_segs=new FanSegment*;
	*fan_segs = NULL;
	std::vector<RawData*> whole_datas;
	if (cfg->is_group_listener == 1)
	{
		ip_mreq group;
		memset(&group, 0, sizeof(group));
		group.imr_multiaddr.s_addr = inet_addr(cfg->group_ip);
		group.imr_interface.s_addr = INADDR_ANY;

		int rt = setsockopt(cfg->fd, IPPROTO_IP,
							IP_ADD_MEMBERSHIP, (char *)&group,
							sizeof(group));

		if (rt < 0)
		{
			DEBUG_PR("Adding to multicast group %s %s\n", cfg->group_ip, rt < 0 ? "fail!" : "ok");
			return NULL;
		}
		INFO_PR("Adding to multicast group success\n");
	}
	else
	{
		setup_lidar_udp(cfg->fd, cfg->lidar_ip, cfg->lidar_port,
					cfg->unit_is_mm, cfg->with_confidence,
					cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->alarm_msg, cfg->version);
	}

	unsigned char *buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	struct timeval tv;
	gettimeofday(&tv, NULL);
	time_t tto = tv.tv_sec + 1;
	uint32_t delay = 0;

	bool should_publish = false;
	int fan_span = 360;
	int idle = 0;
	while (!cfg->should_quit)
	{

		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(cfg->fd, &fds);
		struct timeval to = {1, 0};
		int ret = select(cfg->fd + 1, &fds, NULL, NULL, &to);
		if (cfg->is_group_listener != 1)
		{
			gettimeofday(&tv, NULL);
			if (tv.tv_sec > tto)
			{
				KeepAlive alive;
				gettimeofday(&tv, NULL);
				alive.world_clock = (tv.tv_sec % 3600) * 1000 + tv.tv_usec / 1000;
				alive.delay = delay;

				// acknowlege device
				// int rt = send_cmd_udp(fd_udp, info->lidar_ip, info->lidar_port, 0x4753, rand(), 0, NULL);
				send_cmd_udp(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 0x4b41, rand(), sizeof(alive), &alive, false);

				if (timeprint)
				{
					DevTimestamp devtimestamp;
					memcpy(devtimestamp.ip, cfg->lidar_ip, sizeof(cfg->lidar_ip));
					devtimestamp.port = cfg->lidar_port;
					devtimestamp.timestamp = alive.world_clock;
					devtimestamp.delay = delay;
					cfg->callback(4, &devtimestamp);
				}

				tto = tv.tv_sec + 1;
			}

			// if (ret == 0)
			// {
			// 	if (!runing) continue;

			// 	if (idle++ > 10) {
			// 		rt = send_cmd_udp(fd_udp, cfg->lidar_ip, cfg->lidar_port, 0x0043, rand(), 6, cmd);
			// 		idle = 0;
			// 	}
			// 	continue;
			// }

			if (ret < 0)
			{
				printf("select error\n");
				break;
			}
		}
		// read UDP data
		if (FD_ISSET(cfg->fd, &fds))
		{
			idle = 0;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			int len = recvfrom(cfg->fd, buf, BUF_SIZE, 0, (struct sockaddr *)&addr, &sz);
			if (len > 0)
			{
				int res = -1;
				if (zoneFlag == 1)
				{
					res = zonealarm->getZoneRev(buf, zoneSN);
					if (res != 0)
					{
						RecvZoneDatas *rev = new RecvZoneDatas;
						memcpy(rev, zonealarm->getZoneResult(), sizeof(RecvZoneDatas));
						rev->result = res;
						USER_MSG msg2;
						msg2.type = 2;
						msg2.cmd.type2 = msg.cmd.type2;
						memcpy(msg2.cmd.str, rev, sizeof(RecvZoneDatas));
						msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
						delete rev;
						zoneFlag = 0;
					}
				}
				//设置防区
				else if (zoneFlag == 2)
				{
					res = zonealarm->setZoneRev(buf, zoneSN);
					if (res != 0)
					{
						int *rev = new int;
						*rev = res;
						USER_MSG msg2;
						msg2.type = 2;
						msg2.cmd.type2 = msg.cmd.type2;
						memcpy(msg2.cmd.str, rev, sizeof(int));
						msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
						char tmpbuf[16] = {0};
						udp_talk_C_PACK(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 6, "LSTARH", 2, "OK", 11, tmpbuf);
						zoneFlag = 0;
					}

				} //点云数据
				else
				{
					RawData dat;
					LidarMsgHdr zone;
					memset(&zone, 0, sizeof(LidarMsgHdr));
					int is_pack;
					int consume;
					if (cfg->data_bytes == 3)
					{
						// printf("%.02x %.02x %.02x %.02x\n ",buf[0],buf[1],buf[2],buf[3]);
						is_pack = ParseAPI::parse_data_x(len, buf,
											   fan_span, cfg->unit_is_mm, cfg->with_confidence,
											   dat, consume, cfg->with_chk, zone,fan_segs);
					}
					else
					{
						is_pack = ParseAPI::parse_data(len, buf,
											 fan_span, cfg->unit_is_mm, cfg->with_confidence,
											 dat, consume, cfg->with_chk);
					}
					if (is_pack)
					{
						//！！！CN:用户需要提取数据的操作，可以参考该函数,具体详细的其他打印操作参考user.cpp文件
						//！！！EN:User needs to extract data operation, you can refer to this function，For details about other printing operations, please refer to the user.cpp file
						if (cfg->output_scan)
						{
							if (is_pack==2)
							{
								((void (*)(int, void *))cfg->callback)(2, &zone);
								memcpy(&cfg->zone, &zone, sizeof(LidarMsgHdr));
								continue;
							}
                            else if (is_pack == 3)
							{
								continue;
							}
							if (!cfg->output_360)
							{
								// 每个扇区发送
								memset(&tmp, 0, sizeof(PointData));
								UserAPI::fan_data_process(dat, cfg->output_file, tmp);
							}
							else
							{
								// 整圈
								UserAPI::whole_data_process(dat, cfg->from_zero, cfg->collect_angle,cfg->output_file, tmp,whole_datas);
							}
							if (tmp.N > 0)
							{

								((void (*)(int, void *))cfg->callback)(1, &tmp);
								memcpy(&cfg->pointdata, &tmp, sizeof(PointData));
							}
						}
					}
				}
			}
		}

		if (msgrcv(cfg->thread_ID[1], &msg, sizeof(msg.cmd), 1, IPC_NOWAIT) >= 0)
		{
			switch (msg.cmd.type2)
			{
			case GetDevInfo_MSG:
			{
				EEpromV101 eepromV101;
				memset(&eepromV101,0,sizeof(EEpromV101));
				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				if (udp_talk_GS_PACK(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 6, "LUUIDH", &eepromV101))
				{
					memcpy(msg2.cmd.str, &eepromV101, sizeof(EEpromV101));
				}
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				printf("%d %s\n", __LINE__,__FUNCTION__);
				break;
			}
			case SetDevInfo_MSG:
			{
				DevData devdata;
				memcpy(&devdata, &msg.cmd.str, sizeof(DevData));
				setup_lidar_extre(cfg->type,cfg->fd, cfg->lidar_ip, cfg->lidar_port, devdata);

				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				memcpy(msg2.cmd.str, &devdata, sizeof(DevData));
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				printf("%d %s\n", __LINE__,__FUNCTION__);
				break;
			}
			case ctrl_MSG:
			{
				char str[7] = {0};
				memcpy(&str, &msg.cmd.str, sizeof(str));
				send_cmd_udp(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 0x0043, rand(), 6, str,false);
				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				break;
			}
			case Print_Point_MSG:
			{
				char tmp[2] = {0};
				strcpy(tmp, msg.cmd.str);
				if (tmp[0] - '0')
					cfg->output_scan = true;
				else
					cfg->output_scan = false;

				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				break;
			}
			case Print_TimeStamp_MSG:
			{
				char tmp[2] = {0};
				strcpy(tmp, msg.cmd.str);
				if (tmp[0] - '0')
					timeprint = true;
				else
					timeprint = false;

				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				break;
			}
			case Get_ZONE_MSG:
			{
				zonealarm->getZone(zoneSN);
				zoneFlag = 1;
				break;
			}
			case Set_ZONE_MSG:
			{
				char tmpbuf[12] = {0};
				udp_talk_C_PACK(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 6, "LSTOPH", 2, "OK", 11, tmpbuf);
				zones *cmd = new zones;
				memcpy(cmd, &msg.cmd.str, sizeof(zones));
				zonealarm->setZone(*cmd, zoneSN);
				zoneFlag = 2;
				delete[] cmd;
				break;
			}
			case Set_ZoneSection_MSG:
			{
				int cmd;
				memcpy(&cmd, msg.cmd.str, sizeof(int));
				char tmp[12] = {0};
				strcpy(tmp, msg.cmd.str);
				char tmpbuf[3] = {0};
				tmpbuf[2] = '\0';
				sprintf(tmp, "LSAZN:%dH", cmd);
				if (!udp_talk_S_PACK(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 8, tmp, tmpbuf))
				{
					strcpy(tmpbuf, "NG");
				}
				//设置上传的数据类型为  数据加报警，否则切换防区通道没有实际意义
				if (!udp_talk_S_PACK(cfg->fd, cfg->lidar_ip, cfg->lidar_port, sizeof(tmp), "LSPST:3H", tmpbuf))
				{

					strcpy(tmpbuf, "NG");
				}
				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				memcpy(msg2.cmd.str, &tmpbuf, sizeof(tmpbuf));
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				break;
			}
			}
		}
	}
	close(cfg->fd);
	delete buf;
	return NULL;
}
#endif
bool readConfig(const char* cfg_file_name, RunConfig& cfg)
{

	std::ifstream infile;
	infile.open(cfg_file_name);
	if (!infile.is_open())
		return false;

	std::string s, t;
	std::string lidar_ip_s, lidar_port_s, local_port_s;
	std::string unit_is_mm_s, with_confidence_s, resample_s;
	std::string with_deshadow_s, with_smooth_s, with_chk_s;
	std::string raw_bytes_s, rpm_s, output_scan_s, output_360_s;
	std::string from_zero_s, collect_angle;
	std::string output_file;
	std::string is_group_listener;
	std::string group_ip;
	std::string type;
	std::string baud_rate, port;
	std::string service_port, is_open_service;
	std::string alarm_msg;
	while (getline(infile, s))
	{
		std::string tmp;
		std::stringstream linestream(s);
		getline(linestream, tmp, ':');

		if (tmp == "type")
		{
			getline(linestream, type, ':');
			strcpy(cfg.type, type.c_str());
		}
		else if (tmp == "baud_rate")
		{
			getline(linestream, baud_rate, ':');
			cfg.baud_rate = atoi(baud_rate.c_str());
		}
		else if (tmp == "port")
		{
			getline(linestream, port, ':');
			strcpy(cfg.port, port.c_str());
		}
		else if (tmp == "local_port")
		{
			getline(linestream, local_port_s, ':');
			cfg.local_port = atoi(local_port_s.c_str());
		}
		else if (tmp == "with_confidence")
		{
			getline(linestream, with_confidence_s, ':');
			cfg.with_confidence = atoi(with_confidence_s.c_str());
		}
		else if (tmp == "raw_bytes")
		{
			getline(linestream, raw_bytes_s, ':');
			cfg.data_bytes = atoi(raw_bytes_s.c_str());
		}
		else if (tmp == "unit_is_mm")
		{
			getline(linestream, unit_is_mm_s, ':');
			cfg.unit_is_mm = atoi(unit_is_mm_s.c_str());
		}
		else if (tmp == "with_chk")
		{
			getline(linestream, with_chk_s, ':');
			cfg.with_chk = atoi(with_chk_s.c_str());
		}
		else if (tmp == "with_smooth")
		{
			getline(linestream, with_smooth_s, ':');
			cfg.with_smooth = atoi(with_smooth_s.c_str());
		}
		else if (tmp == "with_deshadow")
		{
			getline(linestream, with_deshadow_s, ':');
			cfg.with_deshadow = atoi(with_deshadow_s.c_str());
		}
		else if (tmp == "resample")
		{
			getline(linestream, resample_s, ':');
			cfg.resample = atoi(resample_s.c_str());
		}
		else if (tmp == "rpm")
		{
			getline(linestream, rpm_s, ':');
			cfg.rpm = atoi(rpm_s.c_str());
		}
		else if (tmp == "output_scan")
		{
			getline(linestream, output_scan_s, ':');
			cfg.output_scan = atoi(output_scan_s.c_str());
		}
		else if (tmp == "output_360")
		{
			getline(linestream, output_360_s, ':');
			cfg.output_360 = atoi(output_360_s.c_str());
		}
		else if (tmp == "from_zero")
		{
			getline(linestream, from_zero_s, ':');
			cfg.from_zero = atoi(from_zero_s.c_str());
		}
		else if (tmp == "collect_angle")
		{
			getline(linestream, collect_angle, ':');
			cfg.collect_angle = atoi(collect_angle.c_str());
		}
		else if (tmp == "output_file")
		{
			getline(linestream, output_file);
			strcpy(cfg.output_file, output_file.c_str());
		}
		else if (tmp == "is_group_listener")
		{
			getline(linestream, is_group_listener, ':');
			cfg.is_group_listener = atoi(is_group_listener.c_str());
		}
		else if (tmp == "group_ip")
		{
			getline(linestream, group_ip, ':');
			strcpy(cfg.group_ip, group_ip.c_str());
		}
		else if (tmp == "service_port")
		{
			getline(linestream, service_port, ':');
			cfg.service_port = atoi(service_port.c_str());;
		}
		else if (tmp == "is_open_service")
		{
			getline(linestream, is_open_service, ':');
			cfg.is_open_service = atoi(is_open_service.c_str());
		}
		else if (tmp == "alarm_msg")
		{
			getline(linestream, alarm_msg, ':');
			cfg.alarm_msg = atoi(alarm_msg.c_str());
		}
		else if (tmp == "shadow_filter.enable")
		{
			getline(linestream, t, ':');
			cfg.shadows_filter.enable = atoi(t.c_str());
		}
		else if (tmp == "shadow_filter.max_range")
		{
			getline(linestream, t, ':');
			cfg.shadows_filter.max_range = atoi(t.c_str());
		}
		else if (tmp == "shadow_filter.min_angle")
		{
			getline(linestream, t, ':');
			cfg.shadows_filter.min_angle = atoi(t.c_str());
		}
		else if (tmp == "shadow_filter.max_angle")
		{
			getline(linestream, t, ':');
			cfg.shadows_filter.max_angle = atoi(t.c_str());
		}
		else if (tmp == "shadow_filter.window")
		{
			getline(linestream, t, ':');
			cfg.shadows_filter.window = atoi(t.c_str());
		}
		else if (tmp == "median_filter.enable")
		{
			getline(linestream, t, ':');
			cfg.median_filter.enable = atoi(t.c_str());
		}
		else if (tmp == "median_filter.window")
		{
			getline(linestream, t, ':');
			cfg.median_filter.window = atoi(t.c_str());
		}
		else if (tmp == "lidar_ip")
		{
			getline(linestream, lidar_ip_s, ':');
			strcpy(cfg.lidar_ip, lidar_ip_s.c_str());
		}
		else if (tmp == "lidar_port")
		{
			getline(linestream, lidar_port_s, ':');
			cfg.lidar_port = atoi(lidar_port_s.c_str());
		}
	}
	return true;
}


