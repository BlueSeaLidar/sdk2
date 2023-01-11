/**

 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved

 * File name:      uart_linux.cpp

 * Author:  	   *
     
 * Version:        1.0  
   
 * Date:		   2022.3.28

 * Description:    The Linux(Ubuntu) platform calls the function interface of the hardware    Serial/USB
 */

#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include "uart_linux.h"
#include "../error.h"
#include <ZoneAlarm.h>

send_cmd_uart_ptr CallBack_Uart;

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
int setup_lidar2(int hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char *version)
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
int setup_lidar_extre(int fd, DevData &data)
{
	//检测需要设置的参数项
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
				bool ret = uart_talk3(fd, 0x0053, rand(), sizeof(cmd), cmd, 3, result);
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
	pcrc[0] = stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	write(fd, buffer, len2);
}
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
	pcrc[0] = stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);

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
	pcrc[0] = stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);

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
int setup_lidar(int fd_uart, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char *version)
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
	ZoneAlarm *zonealarm = new ZoneAlarm(cfg->fd, false, (void *)CallBack_Uart);
	if (strcmp(cfg->type, "uart") == 0)
		setup_lidar(cfg->fd, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->version);
	else
		setup_lidar2(cfg->fd, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->version);
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
					is_pack = parse_data_x(buf_len, buf,
										   fan_span, cfg->unit_is_mm, cfg->with_confidence,
										   dat, consume, cfg->with_chk, zone);
					//printf("%d %s %d\n", __LINE__, __FUNCTION__, is_pack);
				}
				else
				{
					is_pack = parse_data(buf_len, buf,
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
						whole_data_process(dat,  cfg->from_zero, cfg->collect_angle, cfg->output_file,  tmp);
						//执行回调函数
						if (tmp.N > 0)
						{
                            if (cfg->shadows_filter.enable)
							{
								ShadowsFilter(&tmp, cfg->shadows_filter);
							}
							if (cfg->median_filter.enable)
							{
								MedianFilter(&tmp, cfg->median_filter);
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
				setup_lidar_extre(cfg->fd, devdata);
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
