#include"ZoneAlarm.h"
#include"Global.h"
ZoneAlarm::ZoneAlarm(int fd, bool isUDP, void* ptr)
{
	m_fd = fd;
	m_fdType = isUDP;
	m_ptr = ptr;
	m_recvBuf = new RecvBuf;
	m_zoneData = new RecvZoneDatas;
	m_zoneDef = new ZoneDef;
	

}

ZoneAlarm::ZoneAlarm(int fd, bool isUDP, char* dev_ip, int dev_port, void* ptr)
{
	m_fd = fd;
	m_fdType = isUDP;
	m_ptr = ptr;
	strcpy(m_devIP, dev_ip);
	m_devPort = dev_port;
	m_recvBuf = new RecvBuf;
	m_zoneData = new RecvZoneDatas;
	m_zoneDef = new ZoneDef;
}

ZoneAlarm::~ZoneAlarm()
{
	if (m_zoneDef) delete m_zoneDef;
	if (m_resndBuf) delete m_resndBuf;
	if (m_recvBuf) delete m_recvBuf;
}
int ZoneAlarm::getZone(int sn)
{
	memset(m_zoneData, 0, sizeof(RecvZoneDatas));
	memset(m_zoneDef, 0, sizeof(ZoneDef));
	memset(m_recvBuf, 0, sizeof(RecvBuf));
	SendZoneReq(OP_ZONE_QUERY, 0, sn);
	return 0;
}
int ZoneAlarm::getZoneRev(unsigned char* buf, int cmd_len, int sn, int& consume)
{
	
	int idx = 0;
	ZonePacket pac;
	while (idx < cmd_len - 180)
	{
		if (buf[idx] == 0x4C && buf[idx + 1] == 0x48 && buf[idx + 2] == 0xB8 && buf[idx + 3] == 0XA5)
		{
			CmdHeader* hdr = (CmdHeader*)(buf + idx);
			if (hdr->sign != 0x484c)
				continue;
			if (m_resndBuf && m_resndBuf->sn == sn)
			{
				delete m_resndBuf;
				m_resndBuf = NULL;
				m_fail_num = 0;
			}

			memcpy(&pac, buf + sizeof(CmdHeader) + idx, hdr->len);
			if (pac.proto_version != 0x101)
			{
				printf("version:%x \n", pac.proto_version);
				return -1;
			}
			//printf("offset:%x \n", pac.op_offset);
			consume = sizeof(CmdHeader) + idx + hdr->len + 4;

			if (pac.op_offset == OP_ZONE_INFO)
			{
				// got zone info
				memset(m_recvBuf, 0, sizeof(RecvBuf));
				m_recvBuf->total = pac.total_crc[0];
				m_recvBuf->crc = pac.total_crc[1];

				if (m_recvBuf->total != sizeof(ZoneDef)) {
					printf("size %x != %x", pac.total_crc[0], sizeof(ZoneDef));
					return -1;
				}
				// begin to require
				SendZoneReq(0, 512, rand());
				return 0;
			}
			if (pac.op_offset + pac.len > m_recvBuf->total)
			{
				printf("%d + %d > %d", pac.op_offset, pac.len, m_recvBuf->total);
				return -1;
			}
			if (pac.len != 512) {
				printf("payload %d", pac.len);
				return  -1;
			}
			if (pac.op_offset + pac.len <= m_recvBuf->total)
			{
				memcpy(m_recvBuf->buf + pac.op_offset, pac.buf, pac.len);
				m_recvBuf->filled[pac.op_offset / 512] = 1;
				printf("get %x / %x\n", pac.op_offset, m_recvBuf->total);
				if (pac.op_offset + pac.len == m_recvBuf->total)
				{
					if (m_resndBuf) delete m_resndBuf;
					m_resndBuf = NULL;

					if (PackAll())
						return 1;
					else
						return -1;
				}
				else {
					SendZoneReq(pac.op_offset + pac.len, 512, rand());
					return 0;
				}
			}
		}
		else
		{
			idx++;
		}
	}
	if (idx >= cmd_len - 180)
	{
		consume = idx;
		return 0;
	}
	return -1;

}

int ZoneAlarm::getZoneRev(unsigned char* buf,int sn)
{
	CmdHeader* hdr = (CmdHeader*)buf;
	unsigned short cmdr = ~(hdr->cmd);
	if (hdr->sign != 0x484c || cmdr != ZG_PACK)
		return 0;

	unsigned int crc = BaseAPI::stm32crc((unsigned int*)buf, hdr->len / 4 + 2);
	unsigned int* pcrc = (unsigned int*)(buf + sizeof(CmdHeader) + hdr->len);
	if (crc != *pcrc)
		return 0;

	if (m_resndBuf != NULL && m_resndBuf->timeout < time(NULL))
	{
		printf("retry %d\n", m_resndBuf->tried);
		m_resndBuf->timeout = time(NULL) + 3;

		if (m_fdType)
			((void(*)(int, const char*, int, int, int, int, const void*))m_ptr)(m_fd, m_devIP, m_devPort, ZG_PACK, m_resndBuf->sn, m_resndBuf->len, m_resndBuf->buf);
		else
			((void(*)(int, int, int, int, const char*))m_ptr)(m_fd, ZG_PACK, m_resndBuf->sn, m_resndBuf->len, m_resndBuf->buf);

		m_resndBuf->tried++;
		return 0;
	}

	int idx = 0;
	ZonePacket pac;

	if (m_resndBuf)
	{
		delete m_resndBuf;
		m_resndBuf = NULL;
	}
	memcpy(&pac, buf + sizeof(CmdHeader) + idx, hdr->len);
	if (pac.proto_version != 0x101)
	{
		printf("version:%x \n", pac.proto_version);
		return -1;
	}

	if (pac.op_offset == OP_ZONE_INFO)
	{
		// got zone info
		memset(m_recvBuf, 0, sizeof(RecvBuf));
		m_recvBuf->total = pac.total_crc[0];
		m_recvBuf->crc = pac.total_crc[1];

		if (m_recvBuf->total != sizeof(ZoneDef)) {
			printf("size %x != %x", pac.total_crc[0], sizeof(ZoneDef));
			return -1;
		}
		// begin to require
		SendZoneReq(0, 512, rand());
		return 0;
	}
	if (pac.op_offset + pac.len > m_recvBuf->total)
	{
		printf("%d + %d > %d", pac.op_offset, pac.len, m_recvBuf->total);
		return -1;
	}
	if (pac.len != 512) {
		printf("payload %d", pac.len);
		return  -1;
	}
	if (pac.op_offset + pac.len <= m_recvBuf->total)
	{
		memcpy(m_recvBuf->buf + pac.op_offset, pac.buf, pac.len);
		m_recvBuf->filled[pac.op_offset / 512] = 1;
		printf("get %x / %x", pac.op_offset, m_recvBuf->total);
		if (pac.op_offset + pac.len == m_recvBuf->total)
		{
			if (PackAll())
				return 1;
		}
		else {
			SendZoneReq(pac.op_offset + pac.len, 512, rand());
			return 0;
		}
	}
	return -1;
}
void ZoneAlarm::SendZoneReq(uint32_t offset, uint32_t size, uint16_t sn)
{
	ZoneReq req;
	memset(&req, 0, sizeof(ZoneReq));
	req.proto_version = 0x101;
	req.op_offset = offset;
	req.size = size;

	if (m_fdType)
		((void(*)(int, const char*,int,int,int,int,const void*))m_ptr)(m_fd, m_devIP, m_devPort, ZG_PACK, sn, sizeof(ZoneReq), (char*)&req);
	else
		((void(*)(int, int, int, int, const char*))m_ptr)(m_fd, ZG_PACK, sn, sizeof(ZoneReq), (char*)&req);
	SavePacket(ZG_PACK, sn, sizeof(ZoneReq), &req);
}

void ZoneAlarm::SendZoneWrite(uint32_t offset, uint32_t size, uint16_t sn)
{
	ZonePacket pac;
	memset(&pac, 0, sizeof(ZonePacket));
	pac.proto_version = 0x101;
	pac.len = size;
	pac.op_offset = offset;
	if (offset + size <= m_transBuf->total)
	{
		memcpy(pac.buf, m_transBuf->buf + offset, size);
	}
	else if (offset == OP_ZONE_ERASE) {
		pac.total_crc[0] = m_transBuf->total;
		pac.total_crc[1] = m_transBuf->crc;
	}

	if (m_fdType)
		((void(*)(int, const char*, int, int, int, int, const void*))m_ptr)(m_fd, m_devIP, m_devPort, ZS_PACK, sn, sizeof(ZonePacket), (char*)&pac);
	else
		((void(*)(int, int, int, int, const char*))m_ptr)(m_fd, ZS_PACK, sn, sizeof(ZonePacket), (char*)&pac);

	SavePacket(ZS_PACK, sn, sizeof(ZonePacket), &pac);
}
void ZoneAlarm::SavePacket(uint16_t cmd, uint16_t sn, uint16_t len, void* buf)
{
	if (m_resndBuf == NULL)
		m_resndBuf = new ResendPack;

	if (m_resndBuf) {
		m_resndBuf->timeout = time(NULL) + 3;
		m_resndBuf->tried = 1;
		m_resndBuf->cmd = cmd;
		m_resndBuf->sn = sn;
		m_resndBuf->len = len;
		memcpy(m_resndBuf->buf, buf, len);
	}
}
PolygonX* ZoneAlarm::NewPolygon(int zone_id, POLYGON_TYPE type, int output_id)
{
	PolygonX* po = new PolygonX;
	memset(po, 0, sizeof(PolygonX));
	po->zone_id = zone_id;
	po->type = type;
	po->output_id = output_id;

	if (m_sketch == NULL) {
		m_sketch = po;
	}
	else {
		PolygonX* px = m_sketch;
		while (px->next != NULL) {
			//px->selected = 0;
			px = px->next;
		}
		px->next = po;
	}
	return po;
}
int ZoneAlarm::GetAllSketch(int zone_id, Sketch** pps)
{
	int n = 0;
	PolygonX* px = m_sketch;
	while (px) {
		if (px->zone_id == zone_id)
			n++;
		px = px->next;
	}

	Sketch* sketch = new Sketch[n + 1];

	n = 0;
	px = m_sketch;
	while (px)
	{
		if (px->zone_id == zone_id)
		{
			if (Poly2Sketch(px, sketch[n]))
				n++;
		}
		px = px->next;
	}
	*pps = sketch;
	return n;
}
int ZoneAlarm::GetGraph(int zone_id, unsigned char* buf)
{
	int nl = 0;
	PolygonX* px = m_sketch;
	while (px && nl < GRAPH_BUF_SIZE - 8)
	{
		if (px->zone_id == zone_id)
		{
			buf[nl++] = px->type;
			buf[nl++] = zone_id;
			buf[nl++] = px->output_id;

			if (px->type == POLY_RAW)
			{
				buf[nl] = 0;
				for (PolyPoint* ppt = px->first; ppt; ppt = ppt->next)
					buf[nl] += 2;
				nl++;
				for (PolyPoint* ppt = px->first; ppt; ppt = ppt->next)
				{
					memcpy(buf + nl, &(ppt->pt.x), 4); nl += 4;
					memcpy(buf + nl, &(ppt->pt.y), 4); nl += 4;
				}
			}
			else {
				buf[nl++] = 4;
				memcpy(buf + nl, &(px->first->pt.x), 4); nl += 4;
				memcpy(buf + nl, &(px->first->pt.y), 4); nl += 4;
				memcpy(buf + nl, &(px->first->next->pt.x), 4); nl += 4;
				memcpy(buf + nl, &(px->first->next->pt.y), 4); nl += 4;
			}
		}
		px = px->next;
	}
	return nl;
}
bool ZoneAlarm::PackAll()
{
	int hole = 0;
	for (int i = 0; i < m_recvBuf->total / 512; i++) {
		if (m_recvBuf->filled[i] == 0)
			hole++;
	}
	if (hole > 0) {

		printf("data hole %d", hole);
		return false;
	}

	uint32_t crc = BaseAPI::stm32crc((uint32_t*)(m_recvBuf->buf), m_recvBuf->total / 4);
	if (crc != m_recvBuf->crc)
	{
		printf("zone crc %x != %x", crc, m_recvBuf->crc);
		return false;
	}
	memcpy(m_zoneDef, m_recvBuf->buf, sizeof(ZoneDef));

	if (m_zoneDef->hdr.code != 0x454e5a50 // must be "PZON"
		|| m_zoneDef->hdr.size != sizeof(ZoneDef)
		|| m_zoneDef->hdr.proto_ver != 0x101)
	{
		printf("bad zone header %x %x %x", m_zoneDef->hdr.code, m_zoneDef->hdr.size, m_zoneDef->hdr.proto_ver);
		return false;
	}

	crc = BaseAPI::stm32crc((uint32_t*)(m_recvBuf->buf + 16), (m_recvBuf->total - 16) / 4);
	if (crc != m_zoneDef->hdr.crc)
	{
		printf("crc %x != %x", crc, m_zoneDef->hdr.crc);
		return false;
	}

	for (int i = 0; i < 16; i++)
	{
		SetGraph(i, GRAPH_BUF_SIZE, m_zoneDef->hdr.graphics + i * GRAPH_BUF_SIZE);
	}
	for (int i = 0; i < 16; i++)
	{
		ProtectLine* lines = (ProtectLine*)(&m_zoneDef->rays[i * 1024]);
		SetProtectLines(i, lines);
	}

	printf("read zone ok\n");
	return true;
}
int ZoneAlarm::setZone(zones& data,int sn)
{ 
	m_selected_poly = NULL;
	m_selected_point = NULL;
	while (m_sketch)
	{
		PolygonX* po = m_sketch;
		m_sketch = po->next;
		FreePolygon(po);
	}

	for (int i = 0; i < data.num; i++)
	{
		if (data.zoneArr[i].type == POLY_RECT)
		{
			m_selected_poly = NewPolygon(data.zoneArr[i].zoneID, data.zoneArr[i].type, data.zoneArr[i].outputID);
			m_selected_point = NULL;
			PolygonAppend(m_selected_poly, data.zoneArr[i].arg1, data.zoneArr[i].arg2);
			PolygonAppend(m_selected_poly, data.zoneArr[i].arg3, data.zoneArr[i].arg4);
		}
		else if (data.zoneArr[i].type == POLY_ELLIPSE)
		{
			m_selected_poly = NewPolygon(data.zoneArr[i].zoneID, POLY_ELLIPSE, data.zoneArr[i].outputID);
			m_selected_point = NULL;
			PolygonAppend(m_selected_poly, data.zoneArr[i].arg1- data.zoneArr[i].arg3/2, data.zoneArr[i].arg2- data.zoneArr[i].arg4 / 2);
			PolygonAppend(m_selected_poly, data.zoneArr[i].arg1 + data.zoneArr[i].arg3 / 2, data.zoneArr[i].arg2 + data.zoneArr[i].arg4 / 2);
		}
		else if (data.zoneArr[i].type == POLY_FAN)
		{
			m_selected_poly = NewPolygon(data.zoneArr[i].zoneID, POLY_FAN, data.zoneArr[i].outputID);
			m_selected_point = NULL;

			PolygonAppend(m_selected_poly,
				data.zoneArr[i].arg3 * cos(data.zoneArr[i].arg1 * 3.1415926 / 1800),
				data.zoneArr[i].arg3 * sin(data.zoneArr[i].arg1 * 3.1415926 / 1800));

			POINT pt;
			pt.x = data.zoneArr[i].arg4 * cos(data.zoneArr[i].arg2 * 3.1415926 / 1800);
			pt.y = data.zoneArr[i].arg4 * sin(data.zoneArr[i].arg2 * 3.1415926 / 1800);
			PolyPoint* ppt = PolygonAppend(m_selected_poly, pt.x, pt.y);
			UpdatePolygon(m_selected_poly);
		}
		else if (data.zoneArr[i].type == POLY_RAW)
		{
			return  -1;
		}

	}

	int sz = sizeof(ZoneDef);
	memset(m_zoneDef, 0, sizeof(ZoneDef));

	m_zoneDef->hdr.code = 0x454e5a50; // must be "PZON"
	m_zoneDef->hdr.proto_ver = 0x101;
	m_zoneDef->hdr.size = sizeof(ZoneDef);
	m_zoneDef->hdr.last_modified = time(NULL);
	m_zoneDef->hdr.ray_cnt = 12 * 16 * 1024;

	for (int i = 0; i < 16; i++)
	{
		GetGraph(i, m_zoneDef->hdr.graphics + i * GRAPH_BUF_SIZE);

		Sketch* pos = NULL;
		int npo = GetAllSketch(i, &pos);

		ProtectLine* lines = (ProtectLine*)(&m_zoneDef->rays[i * 1024]);

		memset(lines, 0, sizeof(ProtectLine) * 1024);

		for (int j = 0; j < MAX_ZONE_LINES; j++)
		{
			int ang = j * 3600 / MAX_ZONE_LINES;
			hit_test(ang, npo, pos, lines[j].seg);
		}
		delete pos;
	}

	uint32_t* buf1 = (uint32_t*)m_zoneDef;
	m_zoneDef->hdr.crc = BaseAPI::stm32crc(buf1 + 4, (sz - 16) / 4);

	//FILE* fp = fopen("2.txt", "w");
	//if (fp)
	//{
	//	for (int i = 0; i < sizeof(ZoneDef); i++)
	//	{
	//		fprintf(fp, "%02x \n", ((char*)m_zoneDef)[i]);
	//	}
	//	fclose(fp);
	//}
	m_transBuf = new TransBuf;
	memcpy(m_transBuf->buf, m_zoneDef, sz);
	m_transBuf->total = sz;
	m_transBuf->crc = BaseAPI::stm32crc((uint32_t*)(m_transBuf->buf), sz / 4);
	memset(m_transBuf->filled, 0, sizeof(m_transBuf->filled));
	SendZoneWrite(OP_ZONE_ERASE, 0, sn);
	return 0;
}

int ZoneAlarm::setZoneRev(unsigned char* buf,int cmd_len, int sn,int &consume)
{
	if (m_resndBuf != NULL && m_resndBuf->timeout < time(NULL))
	{
		printf("retry %d\n", m_resndBuf->tried);
		m_resndBuf->timeout = time(NULL) + 3;
		if (m_fdType)
			((void(*)(int, const char*, int, int, int, int, const void*))m_ptr)(m_fd, m_devIP, m_devPort, ZS_PACK, m_resndBuf->sn, m_resndBuf->len, m_resndBuf->buf);
		else
			((void(*)(int, int, int, int, const char*))m_ptr)(m_fd, ZS_PACK, m_resndBuf->sn, m_resndBuf->len, m_resndBuf->buf);
		m_resndBuf->tried++;
		return 0;
	}
	int idx = 0;
	ZoneWriteResp resp;
	while (idx < cmd_len - 152)
	{
		if (buf[idx] == 0x4C && buf[idx + 1] == 0x48 && buf[idx + 2] == 0xAC && buf[idx + 3] == 0XA5) 
		{
			CmdHeader* hdr = (CmdHeader*)(buf + idx);
			if (hdr->sign != 0x484c)
				continue;
			if (m_resndBuf && m_resndBuf->sn == sn) 
			{
				delete m_resndBuf;
				m_resndBuf = NULL;
				m_fail_num = 0;
			}

			memcpy(&resp, buf + sizeof(CmdHeader)+ idx, hdr->len);
			printf("offset:%x %s \n", resp.op_offset, resp.msg);
			consume = sizeof(CmdHeader) + idx + hdr->len + 4;

			if (resp.op_offset == OP_ZONE_ERASE) 
			{
				SendZoneWrite(0, 512, sn);
				return 0;
			}
			if (resp.op_offset == OP_ZONE_RELOAD) 
			{
				return 1;
			}

			uint32_t pos = resp.op_offset + 512;
			if (pos < m_transBuf->total)
			{
				SendZoneWrite(pos, 512, sn);
				return 0;
			}
			else if (pos >= m_transBuf->total)
			{
				SendZoneWrite(OP_ZONE_RELOAD, 0, sn);
				return 0;
			}
			
			break;
		}
		else
		{
			idx++;
		}
	}
	if (idx >= cmd_len - 152)
	{
		consume = idx;
		return 0;
	}
	return -1;
}
int ZoneAlarm::setZoneRev(unsigned char* buf, int sn)
{
	CmdHeader* hdr = (CmdHeader*)buf;
	unsigned short cmdr = ~(hdr->cmd);
	if (hdr->sign != 0x484c || cmdr != ZS_PACK)
		return 0;

	unsigned int crc = BaseAPI::stm32crc((unsigned int*)buf, hdr->len / 4 + 2);
	unsigned int* pcrc = (unsigned int*)(buf + sizeof(CmdHeader) + hdr->len);
	if (crc != *pcrc)
		return 0;

	if (m_resndBuf != NULL && m_resndBuf->timeout < time(NULL))
	{
		printf("retry %d\n", m_resndBuf->tried);
		m_resndBuf->timeout = time(NULL) + 3;
		if (m_fdType)
			((void(*)(int, const char*, int, int, int, int, const void*))m_ptr)(m_fd, m_devIP, m_devPort, ZS_PACK, m_resndBuf->sn, m_resndBuf->len, m_resndBuf->buf);
		else
			((void(*)(int, int, int, int, const char*))m_ptr)(m_fd, ZS_PACK, m_resndBuf->sn, m_resndBuf->len, m_resndBuf->buf);
		m_resndBuf->tried++;
		if (m_resndBuf->tried > ZONE_RETRY_NUMBER)
			return -2;
		return 0;
	}
	int idx = 0;
	ZoneWriteResp resp;

	if (m_resndBuf && m_resndBuf->sn == hdr->sn) {
		delete m_resndBuf;
		m_resndBuf = NULL;
	}

	memcpy(&resp, buf + sizeof(CmdHeader), hdr->len);
	printf("offset:%x %s \n", resp.op_offset, resp.msg);

	if (resp.op_offset == OP_ZONE_ERASE) {
		SendZoneWrite(0, 512, rand());
		return 0;
	}
	if (resp.op_offset == OP_ZONE_RELOAD) {
		return 1;
	}

	uint32_t pos = resp.op_offset + 512;
	if (pos < m_transBuf->total)
	{
		SendZoneWrite(pos, 512, rand());
	}
	else if (pos >= m_transBuf->total)
	{
		SendZoneWrite(OP_ZONE_RELOAD, 0, rand());
	}
	return 0;
}
int ZoneAlarm::SetGraph(int zone_id, int ngl, unsigned char* graph)
{
	int nl = 0;
	int n3 = 0;
	m_selected_poly = NULL;
	m_selected_point = NULL;
	
	if (zone_id == 0)
	{
		while (m_sketch)
		{
			PolygonX* po = m_sketch;
			m_sketch = po->next;
			FreePolygon(po);
		}
	}
	while (nl < ngl)
	{
		RecvZoneData zonedata;
		memset(&zonedata, 0, sizeof(RecvZoneData));
		zonedata.type = (POLYGON_TYPE)(graph[nl++]);
		zonedata.zoneID = graph[nl++];
		zonedata.outputID = graph[nl++];
		zonedata.cnt = graph[nl++];

		if (zonedata.type == 0 || zonedata.type > POLY_FAN) break;
		if (zonedata.zoneID > 2) break;
		if (zonedata.cnt < 2) break;
		if ((zonedata.cnt % 2) != 0) break;
		if (zonedata.cnt * 4 + nl > GRAPH_BUF_SIZE) break;
		
		n3 = 0;
		if (zonedata.type == POLY_RECT)
		{
			for (int i = 0; i < zonedata.cnt; i += 2)
			{

				memcpy(&zonedata.value[n3], graph + nl, 4); nl += 4;
				memcpy(&zonedata.value[n3 + 1], graph + nl, 4); nl += 4;
				n3 += 2;
			}
		}
		else if (zonedata.type == POLY_FAN)
		{
			int x1, y1,x2,y2;
			memcpy(&x1, graph + nl, 4); nl += 4;
			memcpy(&y1, graph + nl, 4); nl += 4;
			memcpy(&x2, graph + nl, 4); nl += 4;
			memcpy(&y2, graph + nl, 4); nl += 4;

			int r1 = sqrt(x1 * x1 + y1 * y1);
			int r2 = sqrt(x2 * x2 + y2 * y2);

			int from = atan2(y1, x1) / (2 * 3.1415926) * 3600;
			if (from < 0) from += 3600;

			int to = atan2(y2, x2)/(2 * 3.1415926) * 3600;
			if (to < 0) to += 3600;
			zonedata.value[n3] = from;
			zonedata.value[n3+1] = to;
			zonedata.value[n3+2] = r1;
			zonedata.value[n3+3] = r2;
		}
		memcpy(&m_zoneData->zoneData[m_zoneData->N], &zonedata, sizeof(RecvZoneData));
		m_zoneData->N++;	
	}
	
	return 0;
}
void ZoneAlarm::UpdatePolygon(PolygonX* po)
{
		if (po->type == POLY_FAN)
		{
			Point pt1 = po->first->pt;
			Point pt2 = po->first->next->pt;

			double ang1 = atan2(pt1.y, pt1.x);
			double r1 = sqrt(pt1.y * pt1.y + pt1.x * pt1.x);

			double ang2 = atan2(pt2.y, pt2.x);
			double r2 = sqrt(pt2.y * pt2.y + pt2.x * pt2.x);

			if (po->helper == NULL)
			{
				po->helper = new PolyPoint;
				po->helper->next = new PolyPoint;
				po->helper->next->next = NULL;
			}

			po->helper->pt.x = r2 * cos(ang1);
			po->helper->pt.y = r2 * sin(ang1);
			po->helper->next->pt.x = r1 * cos(ang2);
			po->helper->next->pt.y = r1 * sin(ang2);
		}

}
void FreePolygon(PolygonX* po)
{
	while (po->first)
	{
		PolyPoint* pt = po->first;
		po->first = pt->next;
		delete pt;
	}
	delete po;
}
void ZoneAlarm::SetProtectLines(int zone_id, const ProtectLine* lines)
{
	if (zone_id >= 0 && zone_id < 16)
		memcpy(m_proLines + zone_id, lines, sizeof(ProtectLine) * MAX_ZONE_LINES);
}

int Rect2Vertex(const PolygonX* po, int* xs, int* ys)
{
	Point lt = po->first->pt;
	Point rb = po->first->next->pt;

	xs[0] = lt.x;
	xs[1] = rb.x;
	xs[2] = rb.x;
	xs[3] = lt.x;

	ys[0] = lt.y;
	ys[1] = lt.y;
	ys[2] = rb.y;
	ys[3] = rb.y;

	return 4;
}
bool Poly2Sketch(const PolygonX* po, Sketch& sketch)
{
	int* xs = sketch.xs;
	int* ys = sketch.ys;

	sketch.output_id = po->output_id;
	if (po->type == POLY_FAN)
	{
		sketch.n = Fan2Sketch(po, sketch.xs, sketch.ys);
	}
	else if (po->type == POLY_ELLIPSE)
	{
		sketch.n = Ellipse2Vertex(po, 72, sketch.xs, sketch.ys);
	}
	else if (po->type == POLY_RECT)
	{
		sketch.n = Rect2Vertex(po, sketch.xs, sketch.ys);
	}
	else if (po->type == POLY_RAW)
	{
		sketch.n = Raw2Vertex(po, sketch.xs, sketch.ys);
	}
	else
		return false;

	return true;
}

int pnpoly(int npol, int* xs, int* ys, float x, float y)
{
	int i, j, c = 0;
	for (i = 0, j = npol - 1; i < npol; j = i++)
	{
		if ((((ys[i] <= y) && (y < ys[j])) ||
			((ys[j] <= y) && (y < ys[i]))) &&
			(x < (xs[j] - xs[i]) * (y - ys[i]) / (ys[j] - ys[i]) + xs[i]))
			c = !c;
	}
	return c;
}

int hit_test(int ang, int outpid, int npo, Sketch* pos, ProtectSeg* seg)
{
	float fx = cos(ang * 3.14159265 / 1800) * 8;
	float fy = sin(ang * 3.14159265 / 1800) * 8;

	int from = -1;
	int blob = -1;
	for (int i = 1; i < 0x1FFF; i++)
	{
		//float x = fx * i, y = fy * i;

		int hit = 0;
		for (int j = 0; j < npo; j++)
		{
			if (pos[j].output_id != outpid)
				continue;

			if (pos[j].n == 3600)
			{
				// is Fan
				if (i * 8 > pos[j].xs[ang] && i * 8 < pos[j].ys[ang])
				{
					hit = 1;
				}
			}
			else if (pnpoly(pos[j].n, pos[j].xs, pos[j].ys, fx * i, fy * i))
			{
				hit = 1;
			}
		}
		if (hit != 0) {
			if (from == -1) {
				from = 1;
				for (int i = 0; i < 3; i++) {
					if (seg[i].from == 0) {
						blob = i;
						break;
					}
				}
				if (blob != -1) {
					seg[blob].from = i | (1 << (outpid + 13));
				}
			}
			else if (blob != -1)
			{
				seg[blob].to = i;
			}
		}
		else {
			from = -1;
			blob = -1;
		}
	}

	return 0;
}

int hit_test(int ang, int npo, Sketch* pos, ProtectSeg* seg)
{
	for (int i = 2; i >= 0; i--)
		hit_test(ang, i, npo, pos, seg);
	return 0;
}




PolyPoint* PolygonAppend(PolygonX* po, int x, int y)
{
	PolyPoint* ppt = new PolyPoint;
	ppt->next = NULL;
	ppt->pt.x = x;
	ppt->pt.y = y;
	ppt->selected = 1;

	if (po->first == NULL)
		po->first = ppt;
	else {
		PolyPoint* p = po->first;
		while (p->next != NULL) {
			p->selected = 0;
			p = p->next;
		}
		p->next = ppt;
	}
	return ppt;
}
int Fan2Sketch(const PolygonX* po, int* xs, int* ys)
{
	Fan fan;
	CalcFan(po->first->pt, po->first->next->pt, fan);

	if (fan.r1 > fan.r2)
	{
		double d = fan.r1;
		fan.r1 = fan.r2;
		fan.r2 = d;
	}

	for (int i = 0; i < 3600; i++)
	{
		double ang = i * 3.1415926 / 1800;

		if (fan.from > fan.to)
		{
			if (ang < fan.to)
			{
				xs[i] = fan.r1;
				ys[i] = fan.r2;
			}
			else if (ang > fan.from && ang < 2 * 3.1415926 + fan.to)
			{
				xs[i] = fan.r1;
				ys[i] = fan.r2;
			}
			else {
				xs[i] = ys[i] = 0;
			}
		}
		else if (ang > fan.from && ang < fan.to)
		{
			xs[i] = fan.r1;
			ys[i] = fan.r2;
		}
		else {
			xs[i] = ys[i] = 0;
		}
	}
	return 3600;
}
void CalcFan(Point pt1, Point pt2, Fan& fan)
{
	fan.from = atan2(pt1.y, pt1.x);
	if (fan.from < 0) fan.from += 2 * 3.1415926;

	fan.to = atan2(pt2.y, pt2.x);
	if (fan.to < 0) fan.to += 2 * 3.1415926;

	fan.r1 = sqrt(pt1.x * pt1.x + pt1.y * pt1.y);
	fan.r2 = sqrt(pt2.x * pt2.x + pt2.y * pt2.y);
}
int Ellipse2Vertex(const PolygonX* po, int n, int* xs, int* ys)
{
	Point lt = po->first->pt;
	Point rb = po->first->next->pt;

	int xc = (lt.x + rb.x) / 2;
	int yc = (lt.y + rb.y) / 2;

	int a = fabs(lt.x - rb.x) / 2;
	int b = fabs(lt.y - rb.y) / 2;

	for (int i = 0; i < n; i++)
	{
		xs[i] = xc + a * cos(i * 3.14159265 * 2 / n);
		ys[i] = yc + b * sin(i * 3.14159265 * 2 / n);
	}

	return n;
}
int Raw2Vertex(const PolygonX* po, int* xs, int* ys)
{
	PolyPoint* ppt = po->first;

	int n = 0;
	while (ppt != NULL)
	{
		xs[n] = ppt->pt.x;
		ys[n] = ppt->pt.y;
		ppt = ppt->next;
		n++;
	}

	return n;
}