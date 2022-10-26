#include "../data.h"
#include"../error.h"
#include"uart_win32.h"
#include"ZoneAlarm.h"

send_cmd_uart_ptr CallBack_Uart;

bool uart_talk(HANDLE hCom, int n, const char* cmd,int nhdr, const char* hdr_str,int nfetch, char* fetch)
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
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

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

		for (int i = 0; i < (int)sizeof(buf)- nfetch; i++)
		{
			if (buf[i] == 0x4C && buf[i + 1] == 0x48&& buf[i + 2] == (signed char)0xBC && buf[i + 3] == (signed char)0xFF)
			{
			/*int packSN = ((unsigned int)buf[i + 5] << 8) | (unsigned int)buf[i + 4];
				if (packSN != sn)
					continue;*/

				for (int j = 0;j < nfetch; j++)
				{
					if ((buf[i+j+8] >= 33 && buf[i+j + 8] <= 127))
					{
						fetch[j] = buf[i+j + 8];
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
bool uart_talk3(HANDLE hCom, int mode, int sn, int len, const char* cmd, int result_len,void *result)
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
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

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

int setup_lidar_extre(int fd, DevData& data)
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
				bool ret = uart_talk3((void*)fd,0x0053,rand(), sizeof(cmd), cmd,3, result);
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
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	DWORD nr = 0;
	WriteFile((HANDLE)hCom, buffer, len2, &nr, NULL);
}

int setup_lidar(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char* version)
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
int setup_lidar2(HANDLE hCom, int unit_is_mm, int with_confidence, int resample, int with_deshadow, int with_smooth, int init_rpm, char* version)
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
		memcpy(tmp, buf+10, sizeof(tmp-10));
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
	PeekMessage(&msg, NULL, GetDevInfo_MSG, Set_ZONE_MSG, PM_NOREMOVE);
	bool isrun = false;//初始化运行雷达是否正常运行标志位
	int zoneFlag = 0;//读写防区标志位    0为正常运行  1为读  2为写
	int zoneSN = rand();
	RunConfig* cfg = (RunConfig*)param;
	CallBack_Uart = send_cmd_uart;
	ZoneAlarm *zonealarm=new ZoneAlarm(cfg->fd,false, (void*)CallBack_Uart);
	PointData tmp;//临时存储结构体变量
	 //默认启动雷达
	DWORD dwWritenSize = 0;
	if(strcmp(cfg->type, "uart") == 0)
		setup_lidar((void*)cfg->fd, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->version);
	else
		setup_lidar2((void*)cfg->fd, cfg->unit_is_mm, cfg->with_confidence, cfg->resample, cfg->with_deshadow, cfg->with_smooth, cfg->rpm, cfg->version);
	
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
			else if (zoneFlag==2)
			{
				res = zonealarm->setZoneRev(buf, buf_len, zoneSN, consume);
				if (res !=0)
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
					is_pack = parse_data_x(buf_len, buf,
						fan_span, cfg->unit_is_mm, cfg->with_confidence,
						dat, consume, cfg->with_chk, zone);
				}
				else
				{
					is_pack = parse_data(buf_len, buf,
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
						data_process(dat, cfg->output_file, tmp, cfg->from_zero);
						//执行回调函数
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
				else if (strcmp(cfg->type, "vpc")==0)
				{
					if (!uart_talk3((HANDLE)cfg->fd, 0x4753,rand(), 6, "LUUIDH", sizeof(EEpromV101),eepromv101))
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
				setup_lidar_extre(cfg->fd, *devdata);
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
				else if(strcmp(cfg->type, "vpc")==0)
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
	if(cfg->fd>0)
		CloseHandle((void*)cfg->fd);
	return 0;
}