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
// serial port handle
int g_port = -1;

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

int uart_talk(int fd, int n, const char *cmd,
			  int nhdr, const char *hdr_str,
			  int nfetch, char *fetch)
{
	// printf("send command : %s\n", cmd);
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
			memcpy(fetch, buf + i + nhdr, nfetch);
			fetch[nfetch] = 0;
			return 0;
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
	return -1;
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
int setup_lidar(int fd_uart, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, printfMsg callback)
{
	char buf[32];
	int nr = 0;
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
	EEpromV101 eepromV101;
	memset(&eepromV101, 0, sizeof(EEpromV101));
	// if (g_sigmanage.getDevData)
	// {
	// 	if (uart_talk(fd_uart, 6, "LUUIDH", 11, "PRODUCT SN:", 16, buf) == 0)
	// 	{
	// 		strip(buf, g_uuid);

	// 	}
	// 	else if (uart_talk(fd_uart, 6, "LUUIDH", 10, "VENDOR ID:", 16, buf) == 0)
	// 	{
	// 		strip(buf, g_uuid);
	// 	}
	// 	memcpy(eepromV101.dev_sn, buf, sizeof(buf));
	// 	((void (*)(int, void *))callback)(2, &eepromV101);
	// }

	if (uart_talk(fd_uart, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH",
				  10, "SET LiDAR ", 9, buf) == 0)
	{
		printf("set LiDAR unit to %s\n", buf);
	}

	if (uart_talk(fd_uart, 6, with_confidence == 0 ? "LNCONH" : "LOCONH",
				  6, "LiDAR ", 5, buf) == 0)
	{
		printf("set LiDAR confidence to %s\n", buf);
	}

	if (uart_talk(fd_uart, 6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H",
				  6, "LiDAR ", 5, buf) == 0)
	{
		printf("set deshadow to %d\n", with_deshadow);
	}

	if (uart_talk(fd_uart, 6, with_smooth == 0 ? "LSSS0H" : "LSSS1H",
				  6, "LiDAR ", 5, buf) == 0)
	{
		printf("set smooth to %d\n", with_smooth);
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
		if (uart_talk(fd_uart, 10, buf, 15, "set resolution ", 1, buf2) == 0)
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
			if (uart_talk(fd_uart, strlen(cmd), cmd, 3, "RPM", 5, buf) == 0)
			{
				printf("set RPM to %s\n", buf);
				break;
			}
		}
	}
	return 0;
}
void *lidar_thread_proc_uart(void *param)
{
	int msgrec = -1;		//消息队列返回值
	USER_MSG msg;			//消息队列
	bool dataprint = false; //是否打印数据标志位
	bool timeprint = false;
	RunConfig *cfg = (RunConfig *)param;
	PointData tmp; //临时存储结构体变量
	memset(&tmp, 0, sizeof(PointData));
	//默认启动雷达
	write(cfg->fd, "LSTARH", 6);
	setup_lidar(cfg->fd, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->callback);
	INFO_PR("\033[1;32m----> All params set OK ! Start parser data.\033[0m\n");

	/*
	 * 4, read and parser data
	 */
	unsigned char *buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

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

		// if (ret == 0)
		// {
		// 	DEBUG_PR("read data timeout\n");
		// 	continue;
		// }

		if (ret < 0)
		{
			DEBUG_PR("select error\n");
			return NULL;
		}

		// read data process
		int new_data = -1;
		if (cfg->fd > 0 && FD_ISSET(cfg->fd, &fds))
		{
			int nr = read(cfg->fd, buf + buf_len, BUF_SIZE - buf_len);
			if (nr <= 0)
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

			new_data = nr; // recevied data length
		}

		/*
		 * do parser process
		 */
		if (new_data > 0)
		{
			buf_len += new_data;

			int consume = 0; // in order to compute the rest of data after every parser process
			RawData dat;
			bool is_pack;
			// if (unit_is_mm)// && with_confidence)
			if (cfg->data_bytes == 3)
			{
				is_pack = parse_data_x(buf_len, buf,
									   fan_span, cfg->unit_is_mm, cfg->with_confidence,
									   dat, consume, cfg->with_chk);
			}
			else
			{
				is_pack = parse_data(buf_len, buf,
									 fan_span, cfg->unit_is_mm, cfg->with_confidence,
									 dat, consume, cfg->with_chk);
			}
			// data output
			if (is_pack && dataprint)
			{
				if (cfg->output_scan)
				{
					//！！！CN:用户需要提取数据的操作，可以参考该函数,具体详细的其他打印操作参考user.cpp文件
					//！！！EN:User needs to extract data operation, you can refer to this function，For details about other printing operations, please refer to the user.cpp file
					if (cfg->output_360)
					{
						memset(&tmp, 0, sizeof(PointData));
						fan_data_process(dat, cfg->output_file, tmp);
					}
					else
					{
						whole_data_process(dat, cfg->from_zero, cfg->output_file, tmp);
					}
					if (tmp.N > 0)
					{
						((void (*)(int, void *))cfg->callback)(1, &tmp);
						memcpy(&cfg->pointdata, &tmp, sizeof(PointData));
					}
				}
			}

			if (consume > 0)
			{
				// data is not whole fan,drop it
				if (!is_pack)
				{
#if 0
					FILE* fp = fopen("/tmp/bad.dat", "ab");
					if (fp) {
						fwrite(buf, 1, consume, fp);
						fclose(fp);
					}
#endif
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
				EEpromV101 eepromV101;
				char buf[20] = {0};
				if (uart_talk(cfg->fd, 6, "LUUIDH", 11, "PRODUCT SN:", 9, buf) != 0)
				{
					DEBUG_PR("GetDevInfo_MSG failed\n");
					break;
				}
				else
				{
					memcpy(&eepromV101.dev_sn, buf, sizeof(buf));
					USER_MSG msg2;
					msg2.type = 2;
					msg2.cmd.type2 = msg.cmd.type2;
					memcpy(msg2.cmd.str, &eepromV101, sizeof(EEpromV101));
					msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				}

				break;
			}
			case SetDevInfo_MSG:
			{
				// DevData devdata;
				// memcpy(&devdata,&msg.cmd.str,sizeof(DevData));
				// setup_lidar_extre(cfg->fd, cfg->lidar_ip, cfg->lidar_port, devdata);
				USER_MSG msg2;
				msg2.type = 2;
				msg2.cmd.type2 = msg.cmd.type2;
				msgsnd(cfg->thread_ID[1], &msg2, sizeof(msg2.cmd), 0);
				break;
			}
			case ctrl_MSG:
			{
				char str[7] = {0};
				memcpy(&str, &msg.cmd.str, sizeof(str));
				write(cfg->fd, str, 6);
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
					dataprint = true;
				else
					dataprint = false;

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
			}
		}
	}

	// close(fd);
	return 0;
}