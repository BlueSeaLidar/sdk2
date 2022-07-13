#include "LidarCheckService.h"
#include <string.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
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
	if (m_dwThreadUdp == 0)
		pthread_create(&m_dwThreadUdp, NULL, UDPThreadProc, 0);
}

void LidarCheckService::closeService()
{
	isCloseService = true;
}

std::vector<DevConnInfo> LidarCheckService::getLidarsList()
{
	if (m_dwThreadUdp == 0)
		pthread_create(&m_dwThreadUdp, NULL, UDPThreadProc, 0);

	uartDevInfo();
	return m_infos;
}
void LidarCheckService::clearLidarsCache()
{
	m_infos.clear();
}
void LidarCheckService::getTime_HMS(char *data)
{
	time_t t0 = time(NULL);
	int hh = t0 % (3600 * 24) / 3600;
	int mm = t0 % 3600 / 60;
	int ss = t0 % 60;
	sprintf(data, "%d-%d-%d", hh, mm, ss);
}
void uptodate(DevConnInfo *data)
{
	if (m_infos.size() == 0)
	{
		m_infos.push_back(*data);
		return;
	}
	int i = 0;
	for (i = 0; i < m_infos.size(); i++)
	{
		//如果是同一个雷达则覆盖
		if ((data->type == 0 && strcmp(data->com_port, m_infos.at(i).com_port) == 0) || (data->type != 0 && strcmp(data->conn_ip, m_infos.at(i).conn_ip) == 0))
		{
			memcpy(&m_infos.at(i), data, sizeof(DevConnInfo));
			return;
		}
	}
	m_infos.push_back(*data);
}

void *UDPThreadProc(void *p)
{
	int fd_udp = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd_udp < 0)
	{
		return NULL;
	}
	int yes = 1;
	if (setsockopt(fd_udp, SOL_SOCKET, SO_REUSEADDR, (char *)&yes, sizeof(yes)) < 0)
	{
		return NULL;
	}
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(m_listening_port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int iResult = ::bind(fd_udp, (struct sockaddr *)&addr, sizeof(addr));
	if (iResult != 0)
		return NULL;

	struct ip_mreq mreq;
	mreq.imr_multiaddr.s_addr = inet_addr("225.225.225.225");
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(fd_udp, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0)
	{
		return NULL;
	}
	while (!isCloseService)
	{
		socklen_t sz = sizeof(addr);
		char raw[4096];
		int dw = recvfrom(fd_udp, raw, sizeof(raw), 0, (struct sockaddr *)&addr, &sz);
		if (dw != sizeof(DevInfo))
		{
		}
		if (dw == sizeof(DevInfoV101))
		{
			DevInfoV101 *dvi = (DevInfoV101 *)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0 && dvi->proto_version == 0x101)
			{
				DevConnInfo *conn = new DevConnInfo;
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
				LidarCheckService::getTime_HMS(conn->timeStr);
				uptodate(conn);
			}
		}
		if (dw == sizeof(DevInfo2))
		{
			DevInfo2 *dvi = (DevInfo2 *)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0)
			{
				DevConnInfo *conn = new DevConnInfo;
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

				// GetLocalTime(&(conn->ts));
				LidarCheckService::getTime_HMS(conn->timeStr);
				uptodate(conn);
			}
		}
		if (dw == sizeof(DevInfo))
		{
			DevInfo *dvi = (DevInfo *)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0)
			{
				DevConnInfo *conn = new DevConnInfo;
				conn->type = TYPE_UDP_V1;
				conn->info.v1 = *dvi;

				strcpy(conn->conn_ip, inet_ntoa(addr.sin_addr));
				conn->conn_port = ntohs(addr.sin_port);

				LidarCheckService::getTime_HMS(conn->timeStr);
				uptodate(conn);
			}
		}
	}
	close(fd_udp);
	return NULL;
}
void LidarCheckService::uartDevInfo()
{
	FILE *fp = NULL;
	char buf[1024] = {0};
	char uart[16]={0};
	fp = popen("/bin/ls -l /sys/class/tty/ttyUSB*", "r");
	if (fp)
	{
		int ret = fread(buf, 1, sizeof(buf) - 1, fp);
		if (ret > 0)
		{
			for (int i = 0; i < 10; i++)
			{
				sprintf(uart,"ttyUSB%d",i);
				if (strstr(buf, uart) != NULL)
				{
					DevConnInfo* con2 = new DevConnInfo;
					strcpy(con2->com_port,uart);
					con2->type=TYPE_COM;
					strcpy(con2->timeStr,"");
					uptodate(con2);
				}
			}
		}
		pclose(fp);
	}
}