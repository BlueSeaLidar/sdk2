/**

 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved

 * File name:      udp_linux.cpp

 * Author:  	   *
     
 * Version:        1.0  
   
 * Date:		   2022.3.29

 * Description:    The Linux platform calls the function interface of the hardware    udp
 */

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
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "udp_linux.h"
#include "../error.h"
#include "ZoneAlarm.h"

send_cmd_udp_ptr CallBack_Udp;

void send_cmd_udp_f(int fd_udp, const char *dev_ip, int dev_port,
					int cmd, int sn,
					int len, const void *snd_buf, bool bpr)
{
	char buffer[2048];
	CmdHeader *hdr = (CmdHeader *)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd;
	hdr->sn = sn;

	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), snd_buf, len);

	int n = sizeof(CmdHeader);
	unsigned int *pcrc = (unsigned int *)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);

	sockaddr_in to;
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = inet_addr(dev_ip);
	to.sin_port = htons(dev_port);

	int len2 = len + sizeof(CmdHeader) + 4;

	sendto(fd_udp, buffer, len2, 0, (struct sockaddr *)&to, sizeof(struct sockaddr));

	if (bpr)
	{
		char s[3096];
		for (int i = 0; i < len2; i++)
			sprintf(s + 3 * i, "%02x ", (unsigned char)buffer[i]);

		printf("send to %s:%d 0x%04x sn[%d] L=%d : %s\n",
			   dev_ip, dev_port, cmd, sn, len, s);
	}
}

void send_cmd_udp(int fd_udp, const char *dev_ip, int dev_port,
				  int cmd, int sn,
				  int len, const void *snd_buf)
{
	send_cmd_udp_f(fd_udp, dev_ip, dev_port, cmd, sn, len, snd_buf, false);
}

bool udp_talk_C_PACK(int fd_udp, const char *lidar_ip, int lidar_port,
					 int n, const char *cmd,
					 int nhdr, const char *hdr_str,
					 int nfetch, char *fetch)
{
	printf("send command : \'%s\' \n", cmd);

	unsigned short sn = rand();
	send_cmd_udp(fd_udp, lidar_ip, lidar_port, 0x0043, sn, n, cmd);

	time_t t0 = time(NULL);
	int ntry = 0;
	while (time(NULL) < t0 + 3 && ntry < 1000)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd_udp, &fds);

		struct timeval to = {3, 0};
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

			char buf[1024] = {0};
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader *hdr = (CmdHeader *)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;

				char *payload = buf + sizeof(CmdHeader);
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

bool udp_talk_GS_PACK(int fd_udp, const char *ip, int port, int n, const char *cmd, void *result)
{
	unsigned short sn = rand();
	send_cmd_udp(fd_udp, ip, port, 0x4753, sn, n, cmd);

	int nr = 0;
	for (int i = 0; i < 100; i++)
	{
		fd_set fds;
		FD_ZERO(&fds);

		FD_SET(fd_udp, &fds);

		struct timeval to = {1, 0};
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

			char buf[1024] = {0};
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader *hdr = (CmdHeader *)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;
				memcpy(result, buf + 8, sizeof(EEpromV101));
				return true;
			}
		}
	}

	printf("read %d packets, not response\n", nr);
	return false;
}
//配置信息设置
bool udp_talk_S_PACK(int fd_udp, const char *ip, int port, int n, const char *cmd, void *result)
{
	unsigned short sn = rand();
	send_cmd_udp(fd_udp, ip, port, 0x0053, sn, n, cmd);

	int nr = 0;
	for (int i = 0; i < 100; i++)
	{
		fd_set fds;
		FD_ZERO(&fds);

		FD_SET(fd_udp, &fds);

		struct timeval to = {3, 0};
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
			char buf[1024] = {0};
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader *hdr = (CmdHeader *)buf;
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
bool setup_lidar(int fd_udp, const char *ip, int port,
				 int unit_is_mm, int with_confidence, int resample, int with_deshadow,
				 int with_smooth, int init_rpm, int should_post, char *version)
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
	EEpromV101 *eepromv101 = new EEpromV101;
	memset(eepromv101, 0, sizeof(EEpromV101));
	char result[3] = {0};
	if (!udp_talk_GS_PACK(fd_udp, ip, port, 6, "LUUIDH", eepromv101))
	{
		DEBUG_PR("GetDevInfo_MSG failed\n");
		return false;
	}
	if (eepromv101->with_filter != with_deshadow)
	{
		char cmd[12] = {0};
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
		char cmd[12] = {0};
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
		// resample == 0  非固定角分辨率不适用于网络包计算
		if (resample == 1 || (resample > 100 && resample <= 1500))
			sprintf(buf, "LSRES:%04dH", resample);
		else
			buf[0] = 0;

		if (buf[0])
		{
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
		char cmd[12] = {0};
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
int setup_lidar_extre(int fd_udp, const char *ip, int port, DevData &data)
{
	//检测需要设置的参数项
	// char set[17] = {0};
	char result[3] = {0};
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
		if (flag == 1)
		{
			int index = 3;
			//发送命令合成
			while (index--)
			{

				bool ret = udp_talk_S_PACK(fd_udp, ip, port, sizeof(cmd), cmd, result);
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
					sleep(2);
				}
			}
			flag = 0;
		}
	}

	return 0;
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
		setup_lidar(cfg->fd, cfg->lidar_ip, cfg->lidar_port,
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
				send_cmd_udp_f(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 0x4b41, rand(), sizeof(alive), &alive, false);

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
						is_pack = parse_data_x(len, buf,
											   fan_span, cfg->unit_is_mm, cfg->with_confidence,
											   dat, consume, cfg->with_chk, zone);
					}
					else
					{
						is_pack = parse_data(len, buf,
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
								fan_data_process(dat, cfg->output_file, tmp);
							}
							else
							{
								// 整圈
								whole_data_process(dat, cfg->from_zero, cfg->collect_angle,cfg->output_file, tmp);
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
				setup_lidar_extre(cfg->fd, cfg->lidar_ip, cfg->lidar_port, devdata);

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
				send_cmd_udp(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 0x0043, rand(), 6, str);
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
// 连接雷达，开始接收数据
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
