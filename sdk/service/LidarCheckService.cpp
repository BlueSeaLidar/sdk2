#include <pthread.h>
#include "LidarCheckService.h"

std::vector<DevConnInfo> m_infos;

LidarCheckService::LidarCheckService()
{
	m_close_service = false;
	m_thread_heart = 0;
}

LidarCheckService::~LidarCheckService()
{
	m_close_service = true;
}

void LidarCheckService::run()
{
	if (m_thread_heart != 0)
		return;
#ifdef __unix__
	if (pthread_create(&m_thread_heart,NULL , thread_heart,&m_close_service) != 0)
		return ;
#elif _WIN32
	if ((int)CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)thread_heart, &m_close_service, 0, &m_thread_heart) < 0)
		return ;
#endif
	
}

void LidarCheckService::stop()
{
	m_close_service = true;
}

std::vector<DevConnInfo> LidarCheckService::getLidarsList()
{
	run();
	uartDevInfo();
	return m_infos;
}
void LidarCheckService::getTime_HMS(char* data)
{
	time_t t0 = time(NULL);
	int hh = t0 % (3600 * 24) / 3600;
	int mm = t0 % 3600 / 60;
	int ss = t0 % 60;
	sprintf(data, "%d-%d-%d", hh, mm, ss);
}
void LidarCheckService::clear()
{
	m_infos.clear();
}


void LidarCheckService::uartDevInfo()
{
	std::vector<UARTARG>list;
	SystemAPI::GetComList(list);
	for (unsigned int i = 0; i < list.size(); i++)
	{
		DevConnInfo tmp;
		memset(&tmp, 0, sizeof(DevConnInfo));
		strcpy(tmp.com_port, list.at(i).portName);
		tmp.com_speed = list.at(i).port;
		strcpy(tmp.timeStr,"0");
		uptodate(tmp);
	}
	

	
}
void uptodate(DevConnInfo data)
{
	if (m_infos.size() == 0)
	{
		m_infos.push_back(data);
		return ;
	}
	int i = 0;
	for (unsigned i = 0; i < m_infos.size(); i++)
	{
		//如果是同一个雷达则覆盖
		if ((data.type == TYPE_COM && strcmp(data.com_port,m_infos.at(i).com_port)==0)
			|| (data.type != TYPE_COM && strcmp(data.conn_ip, m_infos.at(i).conn_ip) == 0))
		{
			memcpy(&m_infos.at(i), &data, sizeof(DevConnInfo));
			return;
		}
	}
	m_infos.push_back(data);
}
void* thread_heart(void* p)
{
	bool closeflag = *(bool*)p;
#ifdef _WIN32
	WSADATA   wsda; //   Structure   to   store   info   
	WSAStartup(MAKEWORD(2, 2), &wsda);
#endif
	int sock = socket(AF_INET, SOCK_DGRAM, 0);

	int yes = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes)) < 0)
	{
		return NULL;
	}

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(CHECKSERVICE);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int iResult = ::bind(sock, (struct sockaddr*)&addr, sizeof(addr));
	if (iResult != 0)
		return NULL;


	struct ip_mreq mreq;
	mreq.imr_multiaddr.s_addr = inet_addr("225.225.225.225");
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0)
	{
		return NULL;
	}
	while(!closeflag)
	{
		socklen_t sz = sizeof(addr);
		char raw[4096];
		int dw = recvfrom(sock, raw, sizeof(raw), 0, (struct sockaddr*)&addr, &sz);
		if (dw == sizeof(DevInfoV101))
		{
			DevInfoV101* dvi = (DevInfoV101*)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0 && dvi->proto_version == 0x101)
			{
				DevConnInfo conn;
				memset(&conn, 0, sizeof(DevConnInfo));
				conn.type = TYPE_UDP_V101;
				strcpy(conn.conn_ip, inet_ntoa(addr.sin_addr));
				conn.conn_port = ntohs(addr.sin_port);
				memcpy(&conn.info.v101, dvi, sizeof(DevInfoV101));
				LidarCheckService::getTime_HMS(conn.timeStr);
				uptodate(conn);
			}
		}
		if (dw == sizeof(DevInfo2))
		{
			DevInfo2* dvi = (DevInfo2*)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0)
			{
				DevConnInfo conn;
				memset(&conn, 0, sizeof(DevConnInfo));
				conn.type = TYPE_UDP_V2;
				strcpy(conn.conn_ip, inet_ntoa(addr.sin_addr));
				conn.conn_port = ntohs(addr.sin_port);

				memcpy(&conn.info.v2, dvi, sizeof(DevInfo2));

				LidarCheckService::getTime_HMS(conn.timeStr);
				uptodate(conn);
			}
		}
		if (dw == sizeof(DevInfo))
		{
			DevInfo* dvi = (DevInfo*)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0)
			{
				DevConnInfo conn;
				conn.type = TYPE_UDP_V1;
				conn.info.v1 = *dvi;

				strcpy(conn.conn_ip, inet_ntoa(addr.sin_addr));
				conn.conn_port = ntohs(addr.sin_port);

				LidarCheckService::getTime_HMS(conn.timeStr);
				uptodate(conn);
			}
		}
	}
	SystemAPI::closefd(sock,true);
	return NULL;
}
