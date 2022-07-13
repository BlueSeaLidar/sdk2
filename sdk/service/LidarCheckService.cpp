#include "LidarCheckService.h"

int m_listening_port;
bool isCloseService;
std::vector<DevConnInfo> m_infos;

LidarCheckService::LidarCheckService(int port)
{
	m_listening_port = port;
	isCloseService = false;
}

LidarCheckService::~LidarCheckService()
{
	isCloseService = true;
}

void LidarCheckService::openService()
{
	if(m_dwThreadUdp==0)
		CreateThread(NULL, 0, UDPThreadProc, 0, 0, &m_dwThreadUdp);
	
}

void LidarCheckService::closeService()
{
	isCloseService = true;
}

std::vector<DevConnInfo> LidarCheckService::getLidarsList()
{
	if (m_dwThreadUdp == 0)
		CreateThread(NULL, 0, UDPThreadProc, 0, 0, &m_dwThreadUdp);
	uartDevInfo();
	return m_infos;
}
void LidarCheckService::clearLidarsCache()
{
	m_infos.clear();
}
bool LidarCheckService::RegQueryValueString(ATL::CRegKey& key, LPCTSTR lpValueName, ULONG nBytes, char* sValue)
{
	ULONG nChars = 0;
	LSTATUS nStatus = key.QueryStringValue(lpValueName, nullptr, &nChars);
	if (nStatus != ERROR_SUCCESS)
	{
		SetLastError(nStatus);
		return false;
	}

	//We will use RegQueryValueEx directly here because ATL::CRegKey::QueryStringValue does not handle non-null terminated data
	DWORD dwType = 0;
	//#pragma warning(suppress: 26446 26489 26490)
	nStatus = RegQueryValueEx(key, lpValueName, nullptr, &dwType, (LPBYTE)sValue, &nBytes);
	if (nStatus != ERROR_SUCCESS)
	{
		SetLastError(nStatus);
		return false;
	}
	if ((dwType != REG_SZ) && (dwType != REG_EXPAND_SZ))
	{
		SetLastError(ERROR_INVALID_DATA);
		return false;
	}
	if ((nBytes % sizeof(TCHAR)) != 0)
	{
		SetLastError(ERROR_INVALID_DATA);
		return false;
	}
	//#pragma warning(suppress: 26446 26489)
	if (sValue[(nBytes / sizeof(TCHAR)) - 1] != _T('\0'))
	{
		//Forcibly null terminate the data ourselves
//#pragma warning(suppress: 26446 26489)
		sValue[(nBytes / sizeof(TCHAR))] = _T('\0');
	}

	return true;
}

bool LidarCheckService::QueryDeviceDescription(HDEVINFO hDevInfoSet, SP_DEVINFO_DATA& devInfo, DWORD dwSize, char* sFriendlyName)
{
	DWORD dwType = 0;

	//#pragma warning(suppress: 26446 26490)
	if (!SetupDiGetDeviceRegistryProperty(hDevInfoSet, &devInfo, SPDRP_DEVICEDESC, &dwType,
		(PBYTE)sFriendlyName, dwSize, &dwSize))
		return false;
	if (dwType != REG_SZ)
	{
		SetLastError(ERROR_INVALID_DATA);
		return false;
	}
	return true;
}

bool LidarCheckService::enum_ports(const GUID& guid, CMapStringToString& ports)
{
	DWORD dwFlags = DIGCF_PRESENT | DIGCF_DEVICEINTERFACE;
	//Create a "device information set" for the specified GUID
	HDEVINFO hDevInfoSet = SetupDiGetClassDevs(&guid,
		nullptr, nullptr, dwFlags);

	if (hDevInfoSet == INVALID_HANDLE_VALUE) return false;

	//Finally do the enumeration
	for (int nIndex = 0; ; nIndex++)
	{
		//Enumerate the current device
		SP_DEVINFO_DATA devInfo = { 0 };
		devInfo.cbSize = sizeof(SP_DEVINFO_DATA);
		BOOL bMoreItems = SetupDiEnumDeviceInfo(hDevInfoSet, nIndex, &devInfo);
		if (!bMoreItems) break;

		//Get the registry key which stores the ports settings
		HKEY hk = SetupDiOpenDevRegKey(hDevInfoSet, &devInfo,
			DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);

		if (hk != INVALID_HANDLE_VALUE)
		{
			ATL::CRegKey deviceKey;
			deviceKey.Attach(hk);

			char sPortName[128];
			if (RegQueryValueString(deviceKey, _T("PortName"), 100, sPortName))
			{
				char sName[512];
				if (QueryDeviceDescription(hDevInfoSet, devInfo, 500, sName)) {
					ports.SetAt(sPortName, sName);
				}
			}
		}
	}
	SetupDiDestroyDeviceInfoList(hDevInfoSet);
	return true;
}

HANDLE LidarCheckService::OpenPort(const char* name, int speed)
{
	HANDLE hPort = CreateFile(name,
		GENERIC_READ | GENERIC_WRITE, // Access (read-write) mode
		FILE_SHARE_READ | FILE_SHARE_WRITE,            // Share mode
		NULL,         // Pointer to the security attribute
		OPEN_EXISTING,// How to open the serial port
		0,            // Port attributes
		NULL);        // Handle to port with attribute

	if (hPort == NULL || hPort == INVALID_HANDLE_VALUE)
	{
		//MessageBox(0, "can not open port", name, MB_OK);
		int n=GetLastError();
		return 0;
	}
	DCB PortDCB;
	// Initialize the DCBlength member. 
	PortDCB.DCBlength = sizeof(DCB);
	// Get the default port setting information.
	GetCommState(hPort, &PortDCB);

	// Change the DCB structure settings.
	PortDCB.BaudRate = speed;// 115200;              // Current baud 
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

BOOL LidarCheckService::GetDevInfoByCom(const char* port_str, int speed)
{
	HANDLE hPort = OpenPort(port_str, speed);
	if (hPort == NULL) {
		//	AfxMessageBox("open port failed");
		return FALSE;
	}

	char cmd[] = "LVERSH";
	DWORD dw;
	WriteFile(hPort, cmd, sizeof(cmd), &dw, NULL);

	BOOL bOK = FALSE;
	char* buf = new char[1024 * 1024];
	if (ReadFile(hPort, buf, 1024 * 1024, &dw, NULL))
	{
		if (dw > 0) {
			bOK = TRUE;
		}
	}

	CloseHandle(hPort);
	delete buf;
	return bOK;
}

void LidarCheckService::uartDevInfo()
{
	CMapStringToString ports;
	if (enum_ports(GUID_DEVINTERFACE_COMPORT, ports))
	{
		POSITION pos = ports.GetStartPosition();

		CString key, val;
		while (pos)
		{
			ports.GetNextAssoc(pos, key, val);
			DevConnInfo* conn = new DevConnInfo;
			memset(conn, 0, sizeof(DevConnInfo));
			conn->com_speed = 0;
			conn->type = TYPE_COM;
			strcpy(conn->com_port, key);
#if 0
			CString s;
			s.Format("%s(%s)", key, val);
			HTREEITEM h = m_dev_tree.InsertItem(s, hcom);

			m_dev_tree.SetItemData(h, (DWORD_PTR)conn);
#endif

			if (GetDevInfoByCom(conn->com_port, 230400))
			{
				conn->com_speed = 230400;
				DevConnInfo* con2 = new DevConnInfo;
				memcpy(con2, conn, sizeof(DevConnInfo));
				uptodate(con2);
			}
			else if (GetDevInfoByCom(conn->com_port, 256000))
			{
				conn->com_speed = 256000;
				DevConnInfo* con2 = new DevConnInfo;
				memcpy(con2, conn, sizeof(DevConnInfo));
				uptodate(con2);
			}
			/*else if (GetDevInfoByCom(conn->com_port, 768000))
			{
				conn->com_speed = 768000;
				DevConnInfo* con2 = new DevConnInfo;
				memcpy(con2, conn, sizeof(DevConnInfo));
				uptodate(con2);
			}*/
			else {
				conn->com_speed = 0;
				DevConnInfo* con2 = new DevConnInfo;
				memcpy(con2, conn, sizeof(DevConnInfo));
				uptodate(con2);
			}
		}
	}
}
void uptodate(DevConnInfo* data)
{
	if (m_infos.size() == 0)
	{
		m_infos.push_back(*data);
		return ;
	}
	int i = 0;
	for (i = 0; i < m_infos.size(); i++)
	{
		//如果是同一个雷达则覆盖
		if ((data->type == ConnType::TYPE_COM && strcmp(data->com_port,m_infos.at(i).com_port)==0)
			|| (data->type != ConnType::TYPE_COM && strcmp(data->conn_ip, m_infos.at(i).conn_ip) == 0))
		{
			memcpy(&m_infos.at(i), data, sizeof(DevConnInfo));
			return;
		}
	}
	m_infos.push_back(*data);
}
DWORD __stdcall UDPThreadProc(void* p)
{
	WSADATA   wsda; //   Structure   to   store   info   
	WSAStartup(MAKEWORD(2, 2), &wsda);
	SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);

	int yes = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes)) < 0)
	{
		return -1;
	}

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(m_listening_port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int iResult = ::bind(sock, (SOCKADDR*)&addr, sizeof(addr));
	if (iResult != 0)
		return -1;


	struct ip_mreq mreq;
	mreq.imr_multiaddr.s_addr = inet_addr("225.225.225.225");
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0)
	{
		return -1;
	}

	FILE* fp = NULL;
	while(!isCloseService)
	{
		int sz = sizeof(addr);
		char raw[4096];
		int dw = recvfrom(sock, raw, sizeof(raw), 0, (SOCKADDR*)&addr, &sz);
		if (dw != sizeof(DevInfo))
		{
#if 0
			TRACE("raw %d\n", dw);
			char path[256];
			sprintf(path, "%d-%d.dat", idx, dw);
			FILE* fp = fopen(path, "wb");
			if (fp) {
				fwrite(raw, 1, dw, fp);
				fclose(fp);
			}
			continue;
#endif
		}
		if (dw == sizeof(DevInfoV101))
		{
			DevInfoV101* dvi = (DevInfoV101*)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0 && dvi->proto_version == 0x101)
			{
				DevConnInfo* conn = new DevConnInfo;
				memset(conn, 0, sizeof(DevConnInfo));
				conn->type = TYPE_UDP_V101;
				strcpy(conn->conn_ip, inet_ntoa(addr.sin_addr));
				conn->conn_port = ntohs(addr.sin_port);

				memcpy(&conn->info.v101, dvi, sizeof(DevInfoV101));

#if 0
				// 标签	4个字节
				memcpy(conn->info.sign, dvi->sign, sizeof(dvi->sign));
				// 机器序号	20个字节
				memcpy(conn->info.dev_sn, dvi->dev_sn, sizeof(dvi->dev_sn));
				// 机器类型	11个字节
				memcpy(conn->info.dev_type, dvi->dev_type, sizeof(conn->info.dev_type));
				//版本号	2个字节
				conn->info.version = dvi->version;
				// ip地址	4个 字节
				memcpy(conn->info.ip, dvi->ip, sizeof(dvi->ip));
				// 子网掩码	4个字节
				memcpy(conn->info.mask, dvi->mask, sizeof(dvi->mask));
				// 网关地址	4个字节
				memcpy(conn->info.gateway, dvi->gateway, sizeof(dvi->gateway));
				// 默认目标IP	4个字节
				memcpy(conn->info.remote_ip, dvi->remote_ip, sizeof(dvi->remote_ip));
				//默认目标udp端口号	2个字节
				conn->info.remote_udp = dvi->remote_udp;
				// 默认UDP对外服务端口号	2个字节
				conn->info.port = dvi->port;
#endif

				GetLocalTime(&(conn->ts));
				uptodate(conn);
			}
		}
		if (dw == sizeof(DevInfo2))
		{
			DevInfo2* dvi = (DevInfo2*)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0)
			{
				DevConnInfo* conn = new DevConnInfo;
				memset(conn, 0, sizeof(DevConnInfo));
				conn->type = TYPE_UDP_V2;
				strcpy(conn->conn_ip, inet_ntoa(addr.sin_addr));
				conn->conn_port = ntohs(addr.sin_port);

				memcpy(&conn->info.v2, dvi, sizeof(DevInfo2));
#if 0
				// 标签	4个字节
				memcpy(conn->info.sign, dvi->sign, sizeof(dvi->sign));
				// 机器序号	20个字节
				memcpy(conn->info.dev_sn, dvi->dev_sn, sizeof(dvi->dev_sn));
				// 机器类型	11个字节
				memcpy(conn->info.dev_type, dvi->dev_type, sizeof(dvi->dev_type));
				//版本号	2个字节
				conn->info.version = dvi->version;
				// ip地址	4个 字节
				memcpy(conn->info.ip, dvi->ip, sizeof(dvi->ip));
				// 子网掩码	4个字节
				memcpy(conn->info.mask, dvi->mask, sizeof(dvi->mask));
				// 网关地址	4个字节
				memcpy(conn->info.gateway, dvi->gateway, sizeof(dvi->gateway));
				// 默认目标IP	4个字节
				memcpy(conn->info.remote_ip, dvi->remote_ip, sizeof(dvi->remote_ip));
				//默认目标udp端口号	2个字节
				conn->info.remote_udp = dvi->remote_udp;
				// 默认UDP对外服务端口号	2个字节
				conn->info.port = dvi->port;
#endif

				GetLocalTime(&(conn->ts));
				uptodate(conn);
			}
		}
		if (dw == sizeof(DevInfo))
		{
			DevInfo* dvi = (DevInfo*)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0)
			{
				DevConnInfo* conn = new DevConnInfo;
				conn->type = TYPE_UDP_V1;
				conn->info.v1 = *dvi;

				strcpy(conn->conn_ip, inet_ntoa(addr.sin_addr));
				conn->conn_port = ntohs(addr.sin_port);

				GetLocalTime(&(conn->ts));
				uptodate(conn);
			}
		}
	}
	closesocket(sock);

	return 0;
}
