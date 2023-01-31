// 解析后的数据可以在 data_process 分析
// 请根据实际连接方式修改main函数中对应参数


#include"udp_win32.h"
#include"../data.h"
#include"../error.h"
#include <time.h>
#include"ZoneAlarm.h"

send_cmd_udp_ptr CallBack_Udp;

bool setup_lidar(int fd_udp, const char* ip, int port, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, int should_post,char* version)
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
		if ( resample == 1|| (resample > 100 && resample <= 1500))
			sprintf_s(buf, 30, "LSRES:%04dH", resample);
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
				printf("set RPM to %d  %s\n", init_rpm,result);
				break;
			}
			else
			{
				printf("set RPM to %d  NG   index=%d\n", init_rpm,i+1);
			}
		}
	}
	if (eepromv101->should_post != (should_post==1?3:1))
	{
		char cmd[12] = { 0 };
		sprintf(cmd, "LSPST:%dH", should_post == 1 ? 3 : 1);
		if (udp_talk_S_PACK(fd_udp, ip, port, sizeof(cmd), cmd, result))
		{
			printf("set LiDAR %s %s\n", cmd,result);
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


// 连接雷达，开始接收数据
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
int setup_lidar_extre(int fd_udp, const char* ip, int port, DevData &data)
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
	send_cmd_udp(fd_udp, ip, port, 0x4753, sn, n, cmd);

	int nr = 0;
	for (int i = 0; i < 1000; i++)
	{
		fd_set fds;
		FD_ZERO(&fds);

		FD_SET(fd_udp, &fds);

		struct timeval to = { 1,0};
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
			int sz = sizeof(addr);

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
	send_cmd_udp(fd_udp, ip, port, 0x0053, sn, n, cmd);

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
			int sz = sizeof(addr);
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
	send_cmd_udp(fd_udp, lidar_ip, lidar_port, 0x0043, sn, n, cmd);

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
			int sz = sizeof(addr);

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


void send_cmd_udp_f(int fd_udp, const char* dev_ip, int dev_port,
	int cmd, int sn,
	int len, const void* snd_buf, bool bpr)
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
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	sockaddr_in to;
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = inet_addr(dev_ip);
	to.sin_port = htons(dev_port);

	int len2 = len + sizeof(CmdHeader) + 4;

	sendto(fd_udp, buffer, len2, 0, (struct sockaddr*)&to, sizeof(struct sockaddr));

	if (bpr) {
		char s[3096];
		for (int i = 0; i < len2; i++)
			sprintf_s(s + 3 * i, 8, "%02x ", (unsigned char)buffer[i]);

		printf("send to %s:%d 0x%04x sn[%d] L=%d : %s\n",
			dev_ip, dev_port, cmd, sn, len, s);
	}
}

void send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port,
	int cmd, int sn,
	int len, const void* snd_buf)
{
	 send_cmd_udp_f(fd_udp, dev_ip, dev_port, cmd, sn, len, snd_buf, false);
}


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
	ZoneAlarm* zonealarm = new ZoneAlarm(cfg->fd,true, cfg->lidar_ip,cfg->lidar_port, CallBack_Udp);
	PointData tmp;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	time_t tto = tv.tv_sec + 1;
	uint32_t delay = 0;
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
		setup_lidar(cfg->fd, cfg->lidar_ip, cfg->lidar_port, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->alarm_msg,cfg->version);
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
					if (cfg->data_bytes==3)
					{
						//printf("%.02x %.02x %.02x %.02x\n ", buf[0], buf[1], buf[2], buf[3]);
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
						//memset(&tmp, 0, sizeof(PointData));
						if (cfg->output_scan)
						{
							if (is_pack==2)
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
								fan_data_process(dat, cfg->output_file, tmp);
							}
							else
							{
								// 全部点位，全部扇区
								whole_data_process(dat, cfg->from_zero,cfg->collect_angle, cfg->output_file, tmp);
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
				EEpromV101 *eepromv101 = new EEpromV101;
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
				DevData *devdata = (DevData*)msg.wParam;
				setup_lidar_extre(cfg->fd, cfg->lidar_ip, cfg->lidar_port, *devdata);
				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)devdata, 0))
				{
					printf("threadson post message failed,errno:%d\n", ::GetLastError());
					delete devdata;
				}
				break;
			}
			case ctrl_MSG:
			{
				char *cmd = (char *)msg.wParam;
				send_cmd_udp(cfg->fd, cfg->lidar_ip, cfg->lidar_port, 0x0043, rand(), 6, cmd);
				if (!PostThreadMessage(msg.lParam, msg.message, NULL, 0))
				{
					DEBUG_PR("threadson post message failed,errno:%d\n", ::GetLastError());
				}
				printf("%s\n", cmd);
				delete []cmd;
				break;
			}
			case Print_Point_MSG:
			{
				int *cmd = (int*)msg.wParam;
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
				int *cmd = (int*)msg.wParam;
				timeprint = *cmd;
				int *rev = new int;
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
				char *tmpbuf=new char[3];
				tmpbuf[2] ='\0';
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
	delete []buf;
	return NULL;
}