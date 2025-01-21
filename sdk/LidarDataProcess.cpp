#include "LidarDataProcess.h"
#include "error.h"
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

bool setup_lidar_udp(int fd_udp, RunScript *arg)
{
	char buf[32];
	if (arg->ats >= 0)
	{
		char tmp[3] = {0};
		if (CommunicationAPI::udp_talk_S_PACK(fd_udp, arg->connectArg, arg->connectArg2, 10, "LSATS:001H", tmp))
		{
			printf("set LSATS:1H ,result:%s\n", tmp);
		}
	}
	// 初始化默认开始旋转
	if (arg->with_start >= 0)
	{
		if (CommunicationAPI::udp_talk_C_PACK(fd_udp, arg->connectArg, arg->connectArg2, 6, "LSTARH", 2, "OK", 0, NULL))
		{
			printf("set LiDAR LSTARH  OK \n");
		}
	}
	// 硬件版本号
	if (arg->version >= 0)
	{
		if (CommunicationAPI::udp_talk_C_PACK(fd_udp, arg->connectArg, arg->connectArg2, 6, "LVERSH", 14, "MOTOR VERSION:", 15, buf))
		{
			printf("set LiDAR LXVERH  OK %.12s\n", buf);
		}
	}
	char result[3] = {0};
	if (arg->with_deshadow >= 0)
	{
		char cmd[12] = {0};
		sprintf(cmd, "LSDSW:%dH", arg->with_deshadow);
		if (CommunicationAPI::udp_talk_S_PACK(fd_udp, arg->connectArg, arg->connectArg2, sizeof(cmd), cmd, result))
		{
			printf("set LiDAR deshadow %s %s\n", cmd, result);
		}
		else
		{
			printf("set LiDAR deshadow %s NG\n", cmd);
		}
	}
	if (arg->with_smooth >= 0)
	{
		char cmd[12] = {0};
		sprintf(cmd, "LSSMT:%dH", arg->with_smooth);
		if (CommunicationAPI::udp_talk_S_PACK(fd_udp, arg->connectArg, arg->connectArg2, 6, cmd, result))
		{
			printf("set LiDAR with_smooth %s\n", result);
		}
		else
		{
			printf("set LiDAR with_smooth NG\n");
		}
	}

	if (arg->ntp_enable >= 0)
	{
		bool ret = judgepcIPAddrIsValid(arg->ntp_ip);
		if (!ret)
		{
			printf("ntp ip set error!");
		}
		else
		{
			char cmd[64];
			char ip_1[4];
			char ip_2[4];
			char ip_3[4];
			char ip_4[4];
			ip_1[3] = '\0';
			ip_2[3] = '\0';
			ip_3[3] = '\0';
			ip_4[3] = '\0';

			int idx[3];
			int index = 0;
			int ip_len = strlen(arg->ntp_ip);
			for (int i = 0; i < ip_len; i++)
			{
				if (arg->ntp_ip[i] == '.')
				{
					idx[index] = i;
					index++;
				}
			}
			memcpy(ip_1, &arg->ntp_ip[0], idx[0]);
			memcpy(ip_2, &arg->ntp_ip[idx[0] + 1], idx[1] - idx[0] - 1);
			memcpy(ip_3, &arg->ntp_ip[idx[1] + 1], idx[2] - idx[1] - 1);
			memcpy(ip_4, &arg->ntp_ip[idx[2] + 1], ip_len - idx[2]);
			sprintf(cmd, "LSNTP:%d,%03d.%03d.%03d.%03d,%05dH", arg->ntp_enable, atoi(ip_1), atoi(ip_2), atoi(ip_3), atoi(ip_4), arg->ntp_port);
			// printf(" 1:%s 2:%s 3:%s 4:%s   5:%s\n",ip_1,ip_2,ip_3,ip_4,cmd);
			if (CommunicationAPI::udp_talk_S_PACK(fd_udp, arg->connectArg, arg->connectArg2, strlen(cmd), cmd, result))
			{
				printf("set LiDAR ntp %s\n", result);
			}
			else
			{
				printf("set LiDAR ntp NG\n");
			}
		}
	}

	if (arg->resample_res > 0)
	{
		// resample == 0  非固定角分辨率不适用于网络包计算
		if (arg->resample_res == 1 || (arg->resample_res > 100 && arg->resample_res <= 1500))
			sprintf(buf, "LSRES:%03dH", arg->resample_res);
		else
			buf[0] = 0;

		if (buf[0])
		{
			if (CommunicationAPI::udp_talk_S_PACK(fd_udp, arg->connectArg, arg->connectArg2, strlen(buf), buf, result))
			{
				printf("%s set LiDAR resample %d %s\n", buf, arg->resample_res, result);
			}
			else
			{
				printf("%s set LiDAR resample %d %s\n", buf, arg->resample_res, result);
			}
		}
	}
	if (arg->rpm >= 0)
	{
		char cmd[16];
		sprintf(cmd, "LSRPM:%dH", arg->rpm);
		if (CommunicationAPI::udp_talk_S_PACK(fd_udp, arg->connectArg, arg->connectArg2, strlen(cmd), cmd, result))
		{
			printf("set RPM to %d  %s\n", arg->rpm, result);
		}
		else
		{
			printf("set RPM to %d  NG  \n", arg->rpm);
		}
	}
	if (arg->alarm_msg >= 0)
	{
		char cmd[12] = {0};
		sprintf(cmd, "LSPST:%dH", arg->alarm_msg == 1 ? 3 : 1);
		if (CommunicationAPI::udp_talk_S_PACK(fd_udp, arg->connectArg, arg->connectArg2, sizeof(cmd), cmd, result))
		{
			printf("set LiDAR %s %s\n", cmd, result);
		}
		else
		{
			printf("set LiDAR should_post NG\n");
		}
	}
	if (arg->direction >= 0)
	{
		char cmd[12] = {0};
		sprintf(cmd, "LSCCW:%dH", arg->direction);
		if (CommunicationAPI::udp_talk_S_PACK(fd_udp, arg->connectArg, arg->connectArg2, sizeof(cmd), cmd, result))
		{
			printf("set LiDAR %s %s\n", cmd, result);
		}
		else
		{
			printf("set LiDAR Rotation direction NG\n");
		}
	}
	return true;
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
	RunConfig *cfg = (RunConfig *)param;
	if (cfg->runscript.output_360)
		cfg->userdata.type = FRAMEDATA;
	else
		cfg->userdata.type = SPANDATA;
	strcpy(cfg->userdata.connectArg1, cfg->runscript.connectArg);
	cfg->userdata.connectArg2 = cfg->runscript.connectArg2;
	FanSegment_AA **fan_segs = new FanSegment_AA *;
	*fan_segs = NULL;
	std::vector<RawData> whole_datas;
	int error_num = 0;
	int is_pack;
	int data_bytes = 3;
	int collect_angle = -1, state = -1;
	std::string error;
	UartState uartstate;
	CmdHeader cmdheader;
	char result[512] = {0};
	char info[512] = {0};

	if (strcmp(cfg->runscript.type, "uart") == 0)
		setup_lidar_uart(cfg->fd, &cfg->runscript, &cfg->eepromv101, cfg->hardwareVersion);
	else
		setup_lidar_vpc(cfg->fd, &cfg->runscript);

	// sprintf(info, "All params set OK ! Start parser data");
	// cfg->callback(8, info, strlen(info) + 1);

	/*
	 * 4, read and parser data
	 */
	unsigned char *buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;
	int ret;
	struct timeval start_tv;
	gettimeofday(&start_tv, NULL);

	while (cfg->state != STOP_ALL)
	{

#ifdef _WIN32
		if (cfg->fd > 0)
#elif __linux
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(cfg->fd, &fds);
		struct timeval to = {1, 1};
		ret = select(cfg->fd + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
		{
			printf("select error\n");
			return NULL;
		}
		if (cfg->fd > 0 && FD_ISSET(cfg->fd, &fds))
#endif // _WIN32
		{
			int nr = read(cfg->fd, buf + buf_len, BUF_SIZE - buf_len);
			if (nr < 0)
			{
				sprintf(info, "read port %d error %d", buf_len, nr);
				cfg->callback(9, info, strlen(info) + 1);
				break;
			}
			if (nr == 0)
			{
#ifdef _WIN32
				msleep(10);
#endif
				continue;
			}
			if (nr > 0)
			{
				buf_len += nr;
			}
		}

		if (buf_len > 0)
		{
			int consume = 0; // in order to compute the rest of data after every parser process
			// int res = -1;
			RawData dat;
			if (data_bytes == 3)
			{
				is_pack = ParseAPI::parse_data_x(buf_len, buf, &uartstate, dat, consume, true, data_bytes, result, &cmdheader, (void **)fan_segs);
			}
			else
			{
				is_pack = ParseAPI::parse_data(buf_len, buf, &uartstate, dat, consume, true);
			}
			switch (is_pack)
			{
			case 1:
			{
				if (collect_angle == -1)
				{
					// 获取当前雷达的起始统计角度
					if (state < 0)
					{
						state = UserAPI::autoGetFirstAngle(dat, cfg->runscript.from_zero, whole_datas, error);
					}
					if (state >= 0)
					{
						collect_angle = state;
						cfg->action = ONLINE;
						sprintf(info, "Lidar start work,first span angle is %d", collect_angle / 10);
						cfg->callback(8, info, strlen(info) + 1);
					}
					break;
				}
				if (cfg->runscript.output_360)
				{
					ret = UserAPI::whole_data_process(dat, collect_angle, whole_datas, error);
					if (cfg->action >= RUN && ret == -1)
					{
						timeval tv;
						gettimeofday(&tv, NULL);
						if (tv.tv_sec - start_tv.tv_sec > 2)
						{
							sprintf(info, "%ld %ld span err  code:%d value:%s ", tv.tv_sec, tv.tv_usec, ret, error.c_str());
							cfg->callback(9, info, strlen(info) + 1);
							error = "";
						}
					}
					if (ret == 1)
					{
						cfg->userdata.framedata.data.clear();
						for (std::size_t i = 0; i < whole_datas.size(); i++)
						{
							for (int j = 0; j < whole_datas.at(i).N; j++)
							{
								cfg->userdata.framedata.data.push_back(whole_datas.at(i).points[j]);
							}
						}
						// 执行回调函数
						if (cfg->userdata.framedata.data.size() > 0)
						{
							if (cfg->runscript.shadows_filter.enable)
							{
								AlgorithmAPI::ShadowsFilter(&cfg->userdata, cfg->runscript.shadows_filter);
							}
							if (cfg->runscript.median_filter.enable)
							{
								AlgorithmAPI::MedianFilter(&cfg->userdata, cfg->runscript.median_filter);
							}
							if (checkPointsLengthZero(&cfg->userdata, cfg->runscript.error_scale))
								error_num++;
							else
								error_num = 0;
							if (cfg->runscript.error_circle <= error_num)
							{
								sprintf(info, "%s %d There are many points with a distance of 0 in the current lidar operation", cfg->runscript.connectArg, cfg->runscript.connectArg2);
								cfg->callback(3, info, strlen(info) + 1);
								error_num = 0;
							}
						}
						if (cfg->runscript.separation_filter.filter_open)
						{
							double angle_increment = 2 * PI / cfg->userdata.framedata.data.size();
							AlgorithmAPI::filter(cfg->userdata.framedata.data,
												 cfg->runscript.separation_filter.max_range,
												 cfg->runscript.separation_filter.min_range,
												 cfg->runscript.separation_filter.max_range_difference,
												 cfg->runscript.separation_filter.filter_window, angle_increment);
						}
						cfg->userdata.idx++;
						if (strcmp(cfg->runscript.type, "uart") == 0)
						{
							struct timeval tv;
							gettimeofday(&tv, NULL);
							cfg->userdata.framedata.ts[0] = tv.tv_sec;
							cfg->userdata.framedata.ts[1] = tv.tv_usec;
						}
						else if (strcmp(cfg->runscript.type, "vpc") == 0)
						{
							RawData data = whole_datas.at(whole_datas.size() - 1);
							cfg->userdata.framedata.ts[0] = data.ts[0];
							cfg->userdata.framedata.ts[1] = data.ts[1];
						}
						whole_datas.clear();
						cfg->action = RUN;
					}
					else
						break;
				}
				// 单独扇区输出
				else
				{

					cfg->userdata.idx++;
					memcpy(&cfg->userdata.spandata.data, &dat, sizeof(RawData));
					if (strcmp(cfg->runscript.type, "uart") == 0)
					{
						struct timeval tv;
						gettimeofday(&tv, NULL);
						cfg->userdata.spandata.data.ts[0] = tv.tv_sec;
						cfg->userdata.spandata.data.ts[1] = tv.tv_usec;
					}
					else if (strcmp(cfg->runscript.type, "vpc") == 0)
					{
						// 网络款扇区自带时间戳
					}
					cfg->action = RUN;
				}

				((void (*)(int, void *))cfg->callback)(1, &cfg->userdata);
				// 避免累加越界
				if (cfg->userdata.idx >= MAX_FRAMEIDX)
					cfg->userdata.idx = 0;
				break;
			}
			case 2:
			{
				// 报警信息
				memcpy(&cfg->zonemsg, &result, sizeof(LidarMsgHdr));
				((void (*)(int, void *))cfg->callback)(2, &result);
				cfg->action = FINISH;
				break;
			}
			case 3:
			{
				// 全局参数
				memcpy(&cfg->eepromv101, &result, sizeof(EEpromV101));
				((void (*)(int, void *))cfg->callback)(3, &result);
				cfg->action = FINISH;
				break;
			}
			case 4:
			{
				// 时间同步返回的应答
				break;
			}
			case 5:
			{
				// C_PACK
				cfg->action = FINISH;
				break;
			}
			case 6:
			{
				// S_PACK
				cfg->action = FINISH;
				break;
			}
			case 7:
			{
				// readzone
				break;
			}
			case 8:
			{
				// writezone
				break;
			}
			case 9:
			{
				// 串口每圈头发送的状态信息
				// uartstate.with_fitter,uartstate.with_smooth
				break;
			}
			}
			if (consume > 0)
			{
				// data is not whole fan,drop it
				if (!is_pack)
				{
					printf("drop %d bytes: %02x %02x %02x %02x %02x %02x",
						   consume,
						   buf[0], buf[1], buf[2],
						   buf[3], buf[4], buf[5]);
				}

				for (int i = consume; i < buf_len; i++)
					buf[i - consume] = buf[i];
				buf_len -= consume;
			}
		}
		// 存在需要操作的指令
		switch (cfg->action)
		{
		case CONTROL:
		{
			if (strcmp(cfg->runscript.type, "uart") == 0)
			{
				write(cfg->fd, cfg->send_cmd, cfg->send_len);
			}
			else if (strcmp(cfg->runscript.type, "vpc") == 0)
			{
				CommunicationAPI::send_cmd_vpc(cfg->fd, 0x0043, rand(), cfg->send_len, cfg->send_cmd);
			}
			//((void (*)(int, void *))cfg->callback)(6, (char*)"OK");
			cfg->action = FINISH;

			break;
		}
		case GETALLPARAMS:
		{
			char buf[20] = {0};
			if (strcmp(cfg->runscript.type, "uart") == 0)
			{
				if (CommunicationAPI::uart_talk(cfg->fd, 6, "LUUIDH", 11, "PRODUCT SN:", 16, buf))
				{
					printf("get LiDAR uuid info:  %s\n", buf);
					std::string str = BaseAPI::stringfilter(buf, 16);
					memcpy(cfg->eepromv101.dev_sn, str.c_str(), str.length());
				}
			}
			else if (strcmp(cfg->runscript.type, "vpc") == 0)
			{
				if (!CommunicationAPI::vpc_talk(cfg->fd, 0x4753, rand(), 6, "LUUIDH", sizeof(EEpromV101), &cfg->eepromv101))
				{
					printf("vpc GetDevInfo_MSG failed\n");
					strcpy(cfg->recv_cmd, "NG");
				}
				else
					strcpy(cfg->recv_cmd, "OK");
			}
			cfg->action = FINISH;
			break;
		}
		case SETPARAM:
		{
			if (strcmp(cfg->runscript.type, "vpc") == 0)
			{
				cfg->recv_len = 2;
				if (CommunicationAPI::vpc_talk(cfg->fd, 0x0053, rand(), cfg->send_len, cfg->send_cmd, cfg->recv_len, cfg->recv_cmd))
				{
					printf("cmd:%s  recv: %d %s \n", cfg->send_cmd, cfg->recv_len, cfg->recv_cmd);
					strcpy(cfg->recv_cmd, "OK");
				}
				else
					strcpy(cfg->recv_cmd, "NG");
			}
			else if (strcmp(cfg->runscript.type, "uart") == 0)
			{
				if (CommunicationAPI::uart_talk(cfg->fd, cfg->send_len, cfg->send_cmd, 2, "OK", 0, NULL))
				{
					printf("set %s OK\n", cfg->send_cmd);
					strcpy(cfg->recv_cmd, "OK");
				}
				else
					strcpy(cfg->recv_cmd, "NG");
			}

			cfg->action = FINISH;
			break;
		}
		case READZONE:
		{
			break;
		}
		case WRITEZONE:
		{
			break;
		}
		case ONLINE:
		{
			// 说明运行正常
			break;
		}
		default:
			break;
		}
	}
	SystemAPI::closefd(cfg->fd, false);
	return 0;
}

int setup_lidar_vpc(int hCom, RunScript *arg)
{
	char buf[64];
	int nr = 0;

	if (arg->ats >= 0)
	{
		char tmp[3] = {0};
		if (CommunicationAPI::vpc_talk(hCom, 0x0053, rand(), 10, "LSATS:002H", 3, tmp))
		{
			printf("set LSATS:2H ,result:%s\n", tmp);
		}
	}
	if (arg->with_start >= 0)
	{
		if (CommunicationAPI::vpc_talk(hCom, 0x0043, rand(), 6, "LSTARH", 3, buf))
		{
			printf("set LSTARH ,result:%s\n", buf);
		}
	}

	for (int i = 0; i < 300 && nr <= 0; i++)
	{
		msleep(10);
		nr = read(hCom, buf, sizeof(buf));
	}
	if (nr <= 0)
	{
		printf("serial port seem not working\n");
		return -1;
	}
	// 硬件版本号
	if (arg->version >= 0)
	{
		if (CommunicationAPI::vpc_talk(hCom, 0x0043, rand(), 6, "LXVERH", 64, buf))
		{
			printf("set LiDAR LXVERH  OK  %.16s\n", buf);
		}
	}
	if (arg->uuid >= 0)
	{
		if (CommunicationAPI::vpc_talk(hCom, 0x0043, rand(), 6, "LUUIDH", 32, buf))
		{
			printf("set LiDAR LUUIDH  OK  %.16s\n", buf + 10);
		}
	}
	if (arg->with_deshadow >= 0)
	{
		if (CommunicationAPI::vpc_talk(hCom, 0x0043, rand(), 6, arg->with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 3, buf))
		{
			printf("set deshadow to %d,result:%s\n", arg->with_deshadow, buf);
		}
	}
	if (arg->with_smooth >= 0)
	{
		if (CommunicationAPI::vpc_talk(hCom, 0x0043, rand(), 6, arg->with_smooth == 0 ? "LSSS0H" : "LSSS1H", 3, buf))
		{
			printf("set smooth to %d,result:%s\n", arg->with_smooth, buf);
		}
	}

	if (arg->resample_res >= 0)
	{
		char cmd[32];
		sprintf(cmd, "LSRES:%03dH", arg->resample_res);
		if (CommunicationAPI::vpc_talk(hCom, 0x0053, rand(), sizeof(cmd), cmd, 3, buf))
		{
			printf("set LiDAR resample to %d,result:%s\n", arg->resample_res, buf);
		}
	}
	if (arg->rpm >= 0)
	{
		if (arg->rpm > 300 && arg->rpm <= 3000)
		{
			for (int i = 0; i < 5; i++)
			{
				char cmd[32];
				sprintf(cmd, "LSRPM:%dH", arg->rpm);
				if (CommunicationAPI::vpc_talk(hCom, 0x0043, rand(), strlen(cmd), cmd, 3, buf))
				{
					printf("set RPM to %s,,result:%s\n", cmd, buf);
					break;
				}
			}
		}
	}
	return 0;
}
int setup_lidar_uart(int fd_uart, RunScript *arg, EEpromV101 *eepromv101, char *version)
{
	char buf[32];
	int index = 3;
	int nr = 0;

	if (arg->with_start >= 0)
	{
		write(fd_uart, "LSTARH", 6);
	}
	for (int i = 0; i < 300 && nr <= 0; i++)
	{
		msleep(10);
		nr = read(fd_uart, buf, sizeof(buf));
	}
	if (nr <= 0)
	{
		printf("serial port seem not working\n");
		SystemAPI::closefd(fd_uart, false);
		return READ_UART_FAILED;
	}
	if (arg->uuid >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, "LUUIDH", 11, "PRODUCT SN:", 12, buf))
			{

				strcpy((char *)eepromv101->dev_sn, buf);
				printf("get LiDAR uuid info:  %s\n", eepromv101->dev_sn);
				break;
			}
		}
	}
	// 硬件版本号
	if (arg->version >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, "LXVERH", 14, "MOTOR VERSION:", 12, buf))
			{
				printf("get LiDAR version info:  %.12s\n", buf);
				strcpy(version, buf);
				break;
			}
		}
	}
	if (arg->model >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, "LTYPEH", 8, "TYPE ID:", 16, buf))
			{
				std::string tmp = BaseAPI::stringfilter(buf, 16);
				memcpy((char *)eepromv101->dev_type, tmp.c_str(), tmp.length());
				printf("set LiDAR LTYPEH2  %s \n", eepromv101->dev_type);
				break;
			}
		}
	}
	// Set the lidar returned data unit   CM or MM
	if (arg->unit_is_mm >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, arg->unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 6, "LiDAR ", 12, buf))
			{
				printf("set LiDAR unit %s\n", buf);
				break;
			}
		}
	}
	// set lidar confidence state   LNCONH close   LOCONH open
	if (arg->with_confidence >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, arg->with_confidence == 0 ? "LNCONH" : "LOCONH", 6, "LiDAR ", 12, buf))
			{
				printf("set LiDAR confidence %.02s\n", buf);
				break;
			}
		}
	}
	// set  de-deshadow state    LFFF0H:close  LFFF1H:open
	if (arg->with_deshadow >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, arg->with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 6, "LiDAR ", 12, buf))
			{
				printf("set deshadow %.02s\n", buf);
				break;
			}
		}
	}
	// set  de-smooth     LSSS0H:close   LSSS1H:open
	if (arg->with_smooth >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, arg->with_smooth == 0 ? "LSSS0H" : "LSSS1H", 6, "LiDAR ", 12, buf))
			{
				printf("set smooth %.02s\n", buf);
				break;
			}
		}
	}
	// LSRES:000H :set default Angular resolution  LSRES:001H :fix Angular resolution
	if (arg->resample_res >= 0)
	{
		char cmd[16];
		sprintf(cmd, "LSRES:%dH", arg->resample_res);
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, strlen(cmd), cmd, 15, "set resolution ", 12, buf))
			{
				printf("set LiDAR resample %.02s\n", buf);
				break;
			}
		}
	}

	// setup rpm  (The specific model range is different)
	if (arg->rpm >= 0)
	{
		char cmd[16];
		sprintf(cmd, "LSRPM:%dH", arg->rpm);
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, strlen(cmd), cmd, 8, "Set RPM:", 12, buf))
			{
				printf("set RPM to %d  %.02s\n", arg->rpm, buf);
				break;
			}
		}
	}
	return 0;
}

void *lidar_thread_proc_udp(void *param)
{
	RunConfig *cfg = (RunConfig *)param;
	if (cfg->runscript.output_360)
		cfg->userdata.type = FRAMEDATA;
	else
		cfg->userdata.type = SPANDATA;

	strcpy(cfg->userdata.connectArg1, cfg->runscript.connectArg);
	cfg->userdata.connectArg2 = cfg->runscript.connectArg2;

	FanSegment_AA **fan_segs = new FanSegment_AA *;
	*fan_segs = NULL;
	std::vector<RawData> whole_datas;
	int error_num = 0;
	int is_pack;
	int collect_angle = -1, state = -1;
	int data_bytes = 3;
	std::string error;
	UartState uartstate;
	CmdHeader cmdheader;
	char result[512] = {0};
	char info[512] = {0};
	if (cfg->runscript.is_group_listener == 1)
	{
		ip_mreq group;
		memset(&group, 0, sizeof(group));
		group.imr_multiaddr.s_addr = inet_addr(cfg->runscript.group_ip);
		group.imr_interface.s_addr = INADDR_ANY;

		int rt = setsockopt(cfg->fd, IPPROTO_IP,
							IP_ADD_MEMBERSHIP, (char *)&group,
							sizeof(group));

		if (rt < 0)
		{
			sprintf(info, "Adding to multicast group %s %s", cfg->runscript.group_ip, rt < 0 ? "fail!" : "ok");
			cfg->callback(9, info, strlen(info) + 1);
			return NULL;
		}
		sprintf(info, "Adding to multicast group success");
		cfg->callback(8, info, strlen(info) + 1);
	}
	else
	{
		setup_lidar_udp(cfg->fd, &cfg->runscript);
	}

	// sprintf(info, "All params set OK ! Start parser data");
	// cfg->callback(8, info, strlen(info) + 1);
	unsigned char *buf = new unsigned char[BUF_SIZE];
	struct timeval tv, start_tv;
	gettimeofday(&tv, NULL);
	gettimeofday(&start_tv, NULL);
	time_t tto = tv.tv_sec + 1;
	uint32_t delay = 0;
	while (cfg->state != STOP_ALL)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(cfg->fd, &fds);
		struct timeval to = {1, 0};
		if (cfg->runscript.is_group_listener != 1)
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
				CommunicationAPI::send_cmd_udp(cfg->fd, cfg->runscript.connectArg, cfg->runscript.connectArg2, 0x4b41, rand(), sizeof(alive), &alive);
				tto = tv.tv_sec + 1;
				DevTimestamp devtimestamp;
				memcpy(devtimestamp.ip, cfg->runscript.connectArg, sizeof(cfg->runscript.connectArg));
				devtimestamp.port = cfg->runscript.connectArg2;
				devtimestamp.timestamp = alive.world_clock;
				devtimestamp.delay = delay;
				cfg->callback(4, &devtimestamp, sizeof(DevTimestamp));
			}
			if (ret < 0)
			{
				sprintf(info, "select error");
				cfg->callback(9, info, strlen(info) + 1);
				break;
			}
		}
		// read UDP data
		if (FD_ISSET(cfg->fd, &fds))
		{
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);
			int buf_len = recvfrom(cfg->fd, (char *)buf, BUF_SIZE, 0, (struct sockaddr *)&addr, &sz);

			// printf("IP:%s IP2:%s\n", (char*)inet_ntoa(addr.sin_addr), cfg->runscript.connectArg);

			if (strcmp(cfg->runscript.connectArg, (char *)inet_ntoa(addr.sin_addr)) != 0)
				continue;

			if (buf_len > 0)
			{
				int consume = 0; // in order to compute the rest of data after every parser process
				// int res = -1;
				RawData dat;
				if (data_bytes == 3)
				{
					is_pack = ParseAPI::parse_data_x(buf_len, buf, &uartstate, dat, consume, true, data_bytes, result, &cmdheader, (void **)fan_segs);
				}
				else if (data_bytes == 2)
				{
					is_pack = ParseAPI::parse_data(buf_len, buf, &uartstate, dat, consume, true);
				}
				switch (is_pack)
				{
				case 1:
				{
					if (collect_angle == -1)
					{
						// 获取当前雷达的起始统计角度
						if (state < 0)
							state = UserAPI::autoGetFirstAngle(dat, cfg->runscript.from_zero, whole_datas, error);
						if (state >= 0)
						{
							whole_datas.clear();
							collect_angle = state;
							cfg->action = ONLINE;
							sprintf(info, "Lidar start work,first span angle is %d", collect_angle / 10);
							cfg->callback(8, info, strlen(info) + 1);
						}
						break;
					}
					if (cfg->runscript.output_360)
					{
						int ret = UserAPI::whole_data_process(dat, collect_angle, whole_datas, error);
						if (cfg->action >= RUN && ret == -1)
						{
							timeval tmp_tv;
							gettimeofday(&tmp_tv, NULL);
							if (tmp_tv.tv_sec - start_tv.tv_sec > 2)
							{
								sprintf(info, "%ld %ld span err  code:%d value:%s ", tv.tv_sec, tv.tv_usec, ret, error.c_str());
								cfg->callback(9, info, strlen(info) + 1);
							}
							error = "";
						}
						if (ret == 1)
						{
							cfg->userdata.framedata.data.clear();
							for (unsigned int i = 0; i < whole_datas.size(); i++)
							{
								for (int j = 0; j < whole_datas.at(i).N; j++)
								{
									cfg->userdata.framedata.data.push_back(whole_datas.at(i).points[j]);
								}
							}
							if (cfg->userdata.framedata.data.size() > 0)
							{
								if (checkPointsLengthZero(&cfg->userdata, cfg->runscript.error_scale))
									error_num++;
								else
									error_num = 0;
								if (cfg->runscript.error_circle <= error_num)
								{
									sprintf(info, "%s %d There are many points with a distance of 0 in the current lidar operation", cfg->runscript.connectArg, cfg->runscript.connectArg2);
									cfg->callback(9, info, strlen(info) + 1);
									error_num = 0;
								}
							}
							if (cfg->runscript.separation_filter.filter_open)
							{
								double angle_increment = 2 * PI / cfg->userdata.framedata.data.size();
								AlgorithmAPI::filter(cfg->userdata.framedata.data,
													 cfg->runscript.separation_filter.max_range,
													 cfg->runscript.separation_filter.min_range,
													 cfg->runscript.separation_filter.max_range_difference,
													 cfg->runscript.separation_filter.filter_window, angle_increment);
							}
							cfg->userdata.idx++;
							RawData data = whole_datas.at(0);
							cfg->userdata.framedata.ts[0] = data.ts[0];
							cfg->userdata.framedata.ts[1] = data.ts[1];
							whole_datas.clear();
							cfg->action = RUN;
							cfg->callback(1, &cfg->userdata, sizeof(UserData));
						}
					}
					// 单独扇区输出
					else
					{
						cfg->userdata.idx++;
						memcpy(&cfg->userdata.spandata.data, &dat, sizeof(RawData));
						cfg->action = RUN;
						cfg->callback(1, &cfg->userdata, sizeof(UserData));
					}

					// 避免累加越界
					if (cfg->userdata.idx >= MAX_FRAMEIDX)
						cfg->userdata.idx = 0;
					break;
				}
				case 2:
				{
					// 报警信息
					memcpy(&cfg->zonemsg, &result, sizeof(LidarMsgHdr));
					cfg->callback(2, &result, sizeof(LidarMsgHdr));
					cfg->action = FINISH;
					break;
				}
				case 3:
				{
					// 全局参数
					// memcpy(&cfg->eepromv101, &result, sizeof(EEpromV101));
					//((void (*)(int, void *))cfg->callback)(3, &result);
					// cfg->action = FINISH;
					break;
				}
				case 4:
				{
					// 时间同步返回的应答
					break;
				}
				case 5:
				{
					// C_PACK
					cfg->action = FINISH;
					break;
				}
				case 6:
				{
					// S_PACK
					cfg->action = FINISH;
					break;
				}
				case 7:
				{
					// readzone
					break;
				}
				case 8:
				{
					// writezone
					break;
				}
				case 9:
				{
					// 串口每圈头发送的状态信息
					break;
				}
				}
			}
		}
		switch (cfg->action)
		{
		case CONTROL:
		{
			CommunicationAPI::udp_talk_C_PACK(cfg->fd, cfg->runscript.connectArg, cfg->runscript.connectArg2, cfg->send_len, cfg->send_cmd, 2, "OK", 0, NULL);
			cfg->action = FINISH;
			break;
		}
		case GETALLPARAMS:
		{
			if (!CommunicationAPI::udp_talk_GS_PACK(cfg->fd, cfg->runscript.connectArg, cfg->runscript.connectArg2, cfg->send_len, cfg->send_cmd, &cfg->eepromv101))
			{
				printf("GetDevInfo_MSG failed\n");
				strcpy(cfg->recv_cmd, "NG");
			}
			else
				strcpy(cfg->recv_cmd, "OK");
			cfg->action = FINISH;
			break;
		}
		case SETPARAM:
		{
			if (cfg->mode == S_PACK)
			{
				if (CommunicationAPI::udp_talk_S_PACK(cfg->fd, cfg->runscript.connectArg, cfg->runscript.connectArg2, cfg->send_len, cfg->send_cmd, result))
				{
					printf("set LiDAR  %s %s\n", cfg->send_cmd, result);
					strcpy(cfg->recv_cmd, result);
				}
				else
					strcpy(cfg->recv_cmd, "NG");
			}
			else if (cfg->mode == C_PACK)
			{
				if (CommunicationAPI::udp_talk_C_PACK(cfg->fd, cfg->runscript.connectArg, cfg->runscript.connectArg2, cfg->send_len, cfg->send_cmd, 2, "OK", 0, NULL))
				{
					printf("set LiDAR  %s OK\n", cfg->send_cmd);
					strcpy(cfg->recv_cmd, "OK");
				}
				else
					strcpy(cfg->recv_cmd, "NG");
			}
			cfg->action = FINISH;
			break;
		}
		case READZONE:
		{
			break;
		}
		case WRITEZONE:
		{
			break;
		}
		default:
			break;
		}
	}
	printf("%d\n", cfg->state);
	SystemAPI::closefd(cfg->fd, true);
	return NULL;
}

#include <cctype>
#include <algorithm>
bool readConfig(const char *cfg_file_name, RunScript &cfg)
{
	std::ifstream infile;
	infile.open(cfg_file_name);
	if (!infile.is_open())
		return false;

	std::string str, key, value;
	while (getline(infile, str))
	{
		std::stringstream linestream(str);
		getline(linestream, key, ':');
		if (key.find("#") != std::string::npos || key.find(" ") != std::string::npos)
			continue;
		getline(linestream, value);
		if (!key.empty())
			value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());

		if (key == "type")
			strcpy(cfg.type, value.c_str());
		else if (key == "connectArg")
			strcpy(cfg.connectArg, value.c_str());
		else if (key == "connectArg2")
			cfg.connectArg2 = atoi(value.c_str());
		else if (key == "from_zero")
			cfg.from_zero = atoi(value.c_str());
		else if (key == "error_circle")
			cfg.error_circle = atoi(value.c_str());
		else if (key == "error_scale")
			cfg.error_scale = atof(value.c_str());
		else if (key == "output_360")
			cfg.output_360 = atoi(value.c_str());
		else if (key == "service_port")
			cfg.service_port = atoi(value.c_str());
		else if (key == "is_open_service")
			cfg.is_open_service = atoi(value.c_str());
		else if (key == "is_group_listener")
			cfg.is_group_listener = atoi(value.c_str());
		else if (key == "group_ip")
			strcpy(cfg.group_ip, value.c_str());
		else if (key == "shadow_filter.enable")
			cfg.shadows_filter.enable = atoi(value.c_str());
		else if (key == "shadow_filter.max_range")
			cfg.shadows_filter.max_range = atoi(value.c_str());
		else if (key == "shadow_filter.min_angle")
			cfg.shadows_filter.min_angle = atoi(value.c_str());
		else if (key == "shadow_filter.max_angle")
			cfg.shadows_filter.max_angle = atoi(value.c_str());
		else if (key == "shadow_filter.window")
			cfg.shadows_filter.window = atoi(value.c_str());
		else if (key == "median_filter.enable")
			cfg.median_filter.enable = atoi(value.c_str());
		else if (key == "median_filter.window")
			cfg.median_filter.window = atoi(value.c_str());
		else if (key == "uuid")
			cfg.uuid = atoi(value.c_str());
		else if (key == "model")
			cfg.model = atoi(value.c_str());
		else if (key == "version")
			cfg.version = atoi(value.c_str());
		else if (key == "rpm")
			cfg.rpm = atoi(value.c_str());
		else if (key == "resample_res")
			cfg.resample_res = atoi(value.c_str());
		else if (key == "with_smooth")
			cfg.with_smooth = atoi(value.c_str());
		else if (key == "with_deshadow")
			cfg.with_deshadow = atoi(value.c_str());
		else if (key == "with_start")
			cfg.with_start = atoi(value.c_str());
		else if (key == "with_confidence")
			cfg.with_confidence = atoi(value.c_str());
		else if (key == "unit_is_mm")
			cfg.unit_is_mm = atoi(value.c_str());
		else if (key == "alarm_msg")
			cfg.alarm_msg = atoi(value.c_str());
		else if (key == "direction")
			cfg.direction = atoi(value.c_str());
		else if (key == "ats")
			cfg.ats = atoi(value.c_str());
		else if (key == "local_port")
			cfg.local_port = atoi(value.c_str());
		else if (key == "filter_open")
			cfg.separation_filter.filter_open = atoi(value.c_str());
		else if (key == "max_range")
			cfg.separation_filter.max_range = atof(value.c_str());
		else if (key == "min_range")
			cfg.separation_filter.min_range = atof(value.c_str());
		else if (key == "max_range_difference")
			cfg.separation_filter.max_range_difference = atof(value.c_str());
		else if (key == "filter_window")
			cfg.separation_filter.filter_window = atoi(value.c_str());
		else if (key == "ntp_ip")
			strcpy(cfg.ntp_ip, value.c_str());
		else if (key == "ntp_port")
			cfg.ntp_port = atoi(value.c_str());
		else if (key == "ntp_enable")
			cfg.ntp_enable = atoi(value.c_str());
	}
	if (cfg.error_scale == 0)
		cfg.error_scale = 0.9;

	if (cfg.error_circle == 0)
		cfg.error_circle = 3;

	return true;
}

bool checkPointsLengthZero(UserData *tmp, float scale)
{
	int lengthZeroNum = 0;
	for (unsigned int i = 0; i < tmp->framedata.data.size(); i++)
	{
		if (tmp->framedata.data[i].distance == 0)
		{
			lengthZeroNum++;
		}
	}
	// printf("lengthZeroNum:%d N:%d scale:%f\n", lengthZeroNum, tmp.N, tmp.N * scale);
	if (tmp->framedata.data.size() * scale < lengthZeroNum)
		return true;
	return false;
}
