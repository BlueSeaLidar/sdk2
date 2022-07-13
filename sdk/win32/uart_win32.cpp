// 解析后的数据可以在 data_process 分析
// 请根据实际连接方式修改main函数中对应参数

#include "../data.h"
#include"../error.h"
#include"uart_win32.h"


int uart_talk(HANDLE hCom, int n, const char* cmd,
	int nhdr, const char* hdr_str,
	int nfetch, char* fetch)
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
			memcpy(fetch, buf + i + nhdr, nfetch);
			fetch[nfetch] = 0;
			return 0;
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
	return -1;
}

int setup_lidar(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char* version)
{

	char buf[32];
	DWORD nr = 0;
	for (int i = 0; i < 300 && nr <= 0; i++) {
		Sleep(10);

		ReadFile(hCom, buf, sizeof(buf), &nr, NULL);
	}
	if (nr <= 0) {
		printf("serial port seem not working\n");
		return -1;
	}
	if (uart_talk(hCom, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH",
		10, "SET LiDAR ", 9, buf) == 0)
	{
		printf("set LiDAR unit to %s\n", buf);
	}
	if (uart_talk(hCom, 6, with_confidence == 0 ? "LNCONH" : "LOCONH",
		6, "LiDAR ", 5, buf) == 0)
	{
		printf("set LiDAR confidence to %s\n", buf);
	}

	if (uart_talk(hCom, 6, with_deshadow == 0 ? "LFFF0H" : "LFFF1H",
		6, "LiDAR ", 5, buf) == 0)
	{
		printf("set deshadow to %d\n", with_deshadow);
	}

	if (uart_talk(hCom, 6, with_smooth == 0 ? "LSSS0H" : "LSSS1H",
		6, "LiDAR ", 5, buf) == 0)
	{
		printf("set smooth to %d\n", with_smooth);
	}

	if (resample == 0)
		strcpy_s(buf, 30, "LSRES:000H");
	else if (resample == 1)
		strcpy_s(buf, 30, "LSRES:001H");
	else if (resample > 100 && resample < 1000)
		sprintf_s(buf, 30, "LSRES:%03dH", resample);
	else
		buf[0] = 0;

	if (buf[0]) {
		char buf2[32];
		if (uart_talk(hCom, 10, buf, 15, "set resolution ", 1, buf2) == 0)
		{
			printf("set LiDAR resample to %d\n", resample);
		}
	}
	if (init_rpm > 300 && init_rpm < 3000)
	{
		for (int i = 0; i < 10; i++)
		{
			char cmd[32];
			sprintf(cmd, "LSRPM:%dH", init_rpm);
			if (uart_talk(hCom, strlen(cmd), cmd, 3, "RPM", 5, buf) == 0)
			{
				printf("set RPM to %s\n", buf);
				break;
			}
		}
	}
	return 0;
}

HANDLE open_serial_port(RunConfig & cfg)
{
	cfg.should_quit = false;
	char path[32];
	sprintf_s(path, 30, "\\\\.\\%s", cfg.port);
	// Open the serial port.
	HANDLE hPort = CreateFile(path,
		GENERIC_READ | GENERIC_WRITE, // Access (read-write) mode
		FILE_SHARE_READ| FILE_SHARE_WRITE,            // Share mode
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

DWORD  WINAPI  lidar_thread_proc_uart(void*  param)
{
	MSG msg;
	PeekMessage(&msg, NULL, GetDevInfo_MSG, Print_TimeStamp_MSG, PM_NOREMOVE);
	bool isrun = false;//初始化运行雷达是否正常运行标志位
	bool timeprint = false;//是否打印数据标志位
	RunConfig* cfg = (RunConfig*)param;
	PointData tmp;//临时存储结构体变量
	memset(&tmp, 0, sizeof(PointData));
	 //默认启动雷达
	DWORD dwWritenSize = 0;
	DWORD dwReadSize = 0;
	WriteFile((void*)cfg->fd, "LSTARH", 6, &dwWritenSize, NULL);
	char buf1[32] = { 0 };
	if (uart_talk((void*)cfg->fd, 6, "LSTARH", 4, "STOP", 9, buf1) == 0)
	{
		printf("set LiDAR unit to %s\n", buf1);
	}
	if (uart_talk((HANDLE)cfg->fd, 6, "LUUIDH", 11, "PRODUCT SN: ", 9, buf1) == 0)
	{
		DEBUG_PR("GetDevInfo_MSG failed\n");
	}

	setup_lidar((void*)cfg->fd, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->version);
	INFO_PR("\033[1;32m----> All params set OK ! Start parser data.\033[0m\n");

	
	/*
	 * 4, read and parser data
	 */
	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;
	FILE* fp_rec = NULL; // fopen("/tmp/rec.dat", "ab");
	int fan_span = 360;	 // 36 degrees
	while (!cfg->should_quit)
	{
		// read data process
		int new_data = -1;
		DWORD nr = 0;
		if (cfg->fd > 0)
		{
			ReadFile((HANDLE)cfg->fd, buf + buf_len, BUF_SIZE - buf_len, &nr, NULL);
			/*if (nr == 0)
				continue;*/
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
			// data output and print is open
			if (is_pack)
			{
				if (cfg->output_scan)
				{
					memset(&tmp, 0, sizeof(DataPoint));
					data_process(dat, cfg->output_file, tmp);
					//执行回调函数
					if (tmp.N > 0)
					{
						((void(*)(int, void*))cfg->callback)(1, &tmp);
						memcpy(&cfg->pointdata, &tmp, sizeof(PointData));
					}
				}
				////证明雷达已经正常运行
				//if ((isrun == false) && (cfg->hStartEvent == 0 || SetEvent(cfg->hStartEvent) == 0)) //set thread start event 
				//{
				//	isrun = true;
				//	printf("set start event failed,errno:%d\n", ::GetLastError());
				//	return 1;
				//}
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

		if (PeekMessage(&msg, NULL, GetDevInfo_MSG, Get_OnePoint_MSG, PM_REMOVE)) //get msg from message queue
		{
			/*CMD *cmd = (CMD*)msg.wParam;
			INFO_PR("threadson recv framehead:%x  str:%s addr:%x\n", cmd->framehead,cmd->str, cmd);*/
			switch (msg.message)
			{
			case GetDevInfo_MSG:
			{
				EEpromV101 *eepromv101 = new EEpromV101;
				memset(eepromv101, 0, sizeof(EEpromV101));
				char buf[20] = { 0 };
				if (uart_talk((HANDLE)cfg->fd, 6, "LUUIDH", 11, "PRODUCT SN:", 9, buf) != 0)
				{
					DEBUG_PR("GetDevInfo_MSG failed\n");
					break;
				}
				memcpy(eepromv101->dev_sn, buf, sizeof(buf));
				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)eepromv101, 0))//post thread msg
					printf("threadson post message GetDevInfo_MSG failed,errno:%d\n", ::GetLastError());
				break;
			}
			case SetDevInfo_MSG:
			{
				DevData *devdata = (DevData*)msg.wParam;
				//setup_lidar_extre(cfg->fd, cfg->lidar_ip, cfg->lidar_port, *devdata);
				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)devdata, 0))
				{
					printf("threadson post message failed,errno:%d\n", ::GetLastError());
					delete devdata;
				}
				break;
			}
			case ctrl_MSG:
			{
				char *cmd = (char*)msg.wParam;
				WriteFile((void*)cfg->fd, cmd, 6, &dwWritenSize, NULL);
				char buf1[32] = { 0 };
				if (uart_talk((void*)cfg->fd, 6, cmd, 10, "LiDAR", 9, buf1) == 0)
				{
					printf("set LiDAR unit to %s\n", buf1);
				}

				if (!PostThreadMessage(msg.lParam, msg.message, NULL, 0))
				{
					DEBUG_PR("threadson post message ctrl_MSG failed,errno:%d\n", ::GetLastError());
				}
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
				int *rev =new int;
				*rev = 0;
				if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)rev, 0))
				{
					DEBUG_PR("threadson post message  Print_TimeStamp_MSG failed,errno:%d\n", ::GetLastError());
				}
				delete[]cmd;
				break;
			}
			// case Get_OnePoint_MSG:
			// {
			// 	PointData* rev = new PointData;
			// 	//uart没有时间戳
			// 	if (tmp.N > 0)
			// 	{
			// 		tmp.ts[0] = { 0 };
			// 		tmp.ts[1] = { 0 };
			// 		if (tmp.N > 5000)
			// 			tmp.N = 0;
			// 	}
			// 	memcpy(rev, &tmp, sizeof(PointData));
		
			// 	if (!PostThreadMessage(msg.lParam, msg.message, (WPARAM)rev, 0))
			// 	{
			// 		DEBUG_PR("threadson post message  Print_TimeStamp_MSG failed,errno:%d\n", ::GetLastError());
			// 	}
			// 	break;
			// }
			}
		}
	}
	if(cfg->fd>0)
		CloseHandle((void*)cfg->fd);
	return 0;
}