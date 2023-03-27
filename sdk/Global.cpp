#include "Global.h"
double getAngleWithViewpoint(float r1, float r2, float included_angle)
{
	return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
}
int int_cmper(const void* p1, const void* p2)
{
	int* d1 = (int*)p1;
	int* d2 = (int*)p2;
	return *d1 - *d2;
}
unsigned int BaseAPI::stm32crc(unsigned int* ptr, unsigned int len)
{
	unsigned int xbit, data;
	unsigned int crc32 = 0xFFFFFFFF;
	const unsigned int polynomial = 0x04c11db7;

	for (unsigned int i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (unsigned int bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}
#ifdef _WIN32
void gettimeofday(timeval* tv, void*)
{
	SYSTEMTIME st;
	GetLocalTime(&st);

	tv->tv_sec = time(NULL);
	tv->tv_usec = st.wMilliseconds;
}
#endif

int AlgorithmAPI_E100::ShadowsFilter(PointData* scan_in, const ShadowsFilterParam& param)
{
	double angle_increment = PI * 2 / scan_in->N;

	std::set<int> indices_to_delete;

	for (unsigned int i = 0; i < scan_in->N - param.window - 1; i++)
	{
		if (scan_in->points[i].distance < 0.002) continue;

		for (int y = 1; y < param.window + 1; y++)
		{
			int j = i + y;
			if (j < 0 || j >= (int)scan_in->N || (int)i == j)
				continue;
			if (scan_in->points[j].distance < 0.002)
				continue;
			if (fabs(scan_in->points[i].distance -
				scan_in->points[j].distance) < 0.2)
				continue;

			double rad = getAngleWithViewpoint(
				scan_in->points[i].distance,
				scan_in->points[j].distance,
				y * angle_increment);

			double angle = abs(rad * 180 / PI);
			if (angle < param.min_angle || angle > param.max_angle)
			{
				int from, to;
				if (scan_in->points[i].distance < scan_in->points[j].distance)
				{
					from = i + 1;
					to = j;
				}
				else {
					from = j - 1;
					to = i;
				}

				if (from > to) {
					int t = from;
					from = to;
					to = t;
				}
				for (int index = from; index <= to; index++)
				{
					indices_to_delete.insert(index);
				}
			}
		}
	}

	int nr = 0;
	for (std::set<int>::iterator it = indices_to_delete.begin(); it != indices_to_delete.end(); ++it)
	{
		//scan_in->points[*it].distance = std::numeric_limits<float>::quiet_NaN();
		scan_in->points[*it].distance = 0;
		nr++;
	}

	return nr;
}


int AlgorithmAPI_E100::MedianFilter(PointData* scan_in, const MedianFilterParam& param)
{
	int* dists = new int[scan_in->N];
	int* buf = new int[param.window * 2 + 1];

	for (int i = 0; i < scan_in->N; i++)
		dists[i] = scan_in->points[i].distance * 1000;

	for (int i = param.window; i < scan_in->N - param.window - 1; i++)
	{
		if (dists[i] == 0) continue;

		int n = 0;
		for (int j = -param.window; j <= param.window; j++)
		{
			if (dists[i + j] > 0) {
				buf[n++] = dists[i + j];
			}
		}

		if (n > 2)
		{
			qsort(buf, n, sizeof(int), int_cmper);
			scan_in->points[i].distance = buf[param.window] / 1000.0;
		}
	}

	delete[] dists;
	delete[] buf;

	return 0;
}


static FanSegment* GetFanSegment(const RawDataHdr7& hdr, uint8_t* pdat, bool with_chk)
{
	FanSegment* fan_seg = new FanSegment;
	if (!fan_seg) {
		printf("out of memory\n");
		return NULL;
	}
	fan_seg->hdr = hdr;
	fan_seg->next = NULL;

	uint16_t sum = 0;
	//if (with_chk)
	{
		uint16_t* pchk = (uint16_t*)pdat;
		for (int i = 1; i < HDR7_SIZE / 2; i++)
			sum += pchk[i];
	}


	uint8_t* pDist = pdat + HDR7_SIZE;
	uint8_t* pAngle = pdat + HDR7_SIZE + 2 * hdr.N;
	uint8_t* energy = pdat + HDR7_SIZE + 4 * hdr.N;

	for (int i = 0; i < hdr.N; i++, pDist += 2, pAngle += 2)
	{
		fan_seg->dist[i] = ((uint16_t)(pDist[1]) << 8) | pDist[0];
		fan_seg->angle[i] = ((uint16_t)(pAngle[1]) << 8) | pAngle[0];
		fan_seg->energy[i] = energy[i];

		sum += fan_seg->dist[i];
		sum += fan_seg->angle[i];
		sum += energy[i];
	}

	uint8_t* pchk = pdat + HDR7_SIZE + 5 * hdr.N;
	uint16_t chksum = ((uint16_t)(pchk[1]) << 8) | pchk[0];
	if (chksum != sum) {
		DEBUG_PR("checksum error\n");
		delete fan_seg;
		return NULL;
	}

	return fan_seg;
}


void DecTimestamp(uint32_t ts, uint32_t* ts2)
{
	timeval tv;
	gettimeofday(&tv, NULL);

	uint32_t sec = tv.tv_sec % 3600;
	if (sec < 5 && ts / 1000 > 3595)
	{
		ts2[0] = (tv.tv_sec / 3600 - 1) * 3600 + ts / 1000;
	}
	else {
		ts2[0] = (tv.tv_sec / 3600) * 3600 + ts / 1000;
	}

	ts2[1] = (ts % 1000) * 1000;
}

static void PackFanData(FanSegment* seg, RawData& rdat)
{
	RawData* dat = &rdat;

	dat->code = 0xfac7;
	dat->N = seg->hdr.whole_fan;
	dat->angle = seg->hdr.beg_ang / 100; // 0.1 degree
	dat->span = (seg->hdr.end_ang - seg->hdr.beg_ang) / 100; // 0.1 degree
	dat->fbase = 0;
	dat->first = 0;
	dat->last = 0;
	dat->fend = 0;

	DecTimestamp(seg->hdr.timestamp, dat->ts);

	int count = 0;
	while (seg)
	{
		double s = PI / 180000.0;
		for (int i = 0; i < seg->hdr.N; i++, count++)
		{
			dat->points[count].confidence = seg->energy[i];
			dat->points[count].distance = seg->dist[i] / 1000.0;
			//dat->points[count].angle = (seg->angle[i] + seg->hdr.beg_ang) * s;
			float tmp = (seg->angle[i] + seg->hdr.beg_ang) ;
			dat->points[count].angle = (tmp>=360*1000?tmp-360*1000:tmp) * s;
		}
		seg = seg->next;
	}
	//return dat;
}

static int GetFanPointCount(FanSegment* seg)
{
	int n = 0;

	while (seg) { n += seg->hdr.N;  seg = seg->next; }

	return n;
}

static bool GetData0xC7(const RawDataHdr7& hdr, uint8_t* pdat, bool with_chk, RawData& dat, FanSegment** last_fan_seg)
{
	bool got = false;

	FanSegment* fan_seg = GetFanSegment(hdr, pdat, with_chk);
	if (!fan_seg) {
		return NULL;
	}
	if (*last_fan_seg != NULL)
	{
		FanSegment* seg = (*last_fan_seg);

		//printf("%d,%d  %d,%d\n", seg->hdr.timestamp, fan_seg->hdr.timestamp, seg->hdr.beg_ang, fan_seg->hdr.beg_ang);
		if (seg->hdr.timestamp != fan_seg->hdr.timestamp)
		{
			printf("drop old fan segments\n");
			while (seg) {
				(*last_fan_seg) = seg->next;
				delete seg;
				seg = (*last_fan_seg);
			}
			(*last_fan_seg) = fan_seg;
		}
		else {
			while (seg) {
				if (seg->hdr.ofset == fan_seg->hdr.ofset) {
					printf("drop duplicated segment\n");
					delete fan_seg;
					fan_seg = NULL;
					break;
				}
				if (seg->next == NULL) {
					seg->next = fan_seg;
					break;
				}
				seg = seg->next;
			}
		}
	}

	if (*last_fan_seg == NULL && fan_seg != NULL)
	{
		*last_fan_seg = fan_seg;
		//printf("bag1 data :%d  time:%ld\n", (*last_fan_seg)->hdr.N, (*last_fan_seg)->hdr.timestamp);
	}

	int N = GetFanPointCount((*last_fan_seg));

	if (N >= (*last_fan_seg)->hdr.whole_fan)
	{
		if (N == (*last_fan_seg)->hdr.whole_fan)
		{
			if (N > sizeof(dat.points) / sizeof(dat.points[0]))
			{
				printf("too many %d points in 1 fan\n", N);
			}
			else
			{
				PackFanData((*last_fan_seg), dat);
				got = true;
			}
		}

		// remove segments	
		FanSegment* seg = (*last_fan_seg);
		while (seg) {
			(*last_fan_seg) = seg->next;
			delete seg;
			seg = (*last_fan_seg);
		}
	}
	return got;
}


bool GetData0xCE(const RawDataHdr& hdr, unsigned char* pdat, int span, int with_chk, RawData& dat)
{
	// calc checksum
	unsigned short sum = hdr.angle + hdr.N, chk;
	for (int i = 0; i < hdr.N; i++)
	{
		dat.points[i].confidence = *pdat++;
		sum += dat.points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short vv = (v2 << 8) | v;
		dat.points[i].distance = vv / 1000.0;
		sum += vv;
		dat.points[i].angle = (hdr.angle + (double)span * i / hdr.N) * PI / 1800;
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum)
	{
		DEBUG_PR("chksum ce error");
		// consume = idx + HDR_SIZE + 3*hdr.N + 2;
		return false;
	}

	memcpy(&dat, &hdr, HDR_SIZE);
	dat.span = span;
	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	return true;
}

bool GetData0x9D(const RawDataHdr2& hdr, unsigned char* pdat, int with_chk, RawData& dat)
{
	unsigned short sum = hdr.angle + hdr.N + hdr.span, chk;

	for (int i = 0; i < hdr.N; i++)
	{
		dat.points[i].confidence = 1;// *pdat++;
		//sum += dat.points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;

		unsigned short vv = (v2 << 8) | v;

		sum += vv;
		dat.points[i].distance = vv / 1000.0;
		dat.points[i].angle = (hdr.angle + hdr.span * i / (double)hdr.N) * PI / 1800;
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum)
	{
		DEBUG_PR("chksum cf error");
		return 0;
	}

	memcpy(&dat, &hdr, sizeof(hdr));
	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	return true;

}


bool GetData0xCF(const RawDataHdr2& hdr, unsigned char* pdat, int with_chk, RawData& dat)
{
	unsigned short sum = hdr.angle + hdr.N + hdr.span, chk;

	for (int i = 0; i < hdr.N; i++)
	{
		dat.points[i].confidence = *pdat++;
		sum += dat.points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;

		unsigned short vv = (v2 << 8) | v;

		sum += vv;
		dat.points[i].distance = vv / 1000.0;
		double tmp = hdr.angle + hdr.span * i / (double)hdr.N;
		dat.points[i].angle = (tmp>=3600?tmp-3600:tmp) * PI / 1800;
		printf("qwer(%lf %f %d)\n", tmp, dat.points[i].angle, hdr.N);
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum)
	{
		DEBUG_PR("chksum cf error");
		return 0;
	}

	memcpy(&dat, &hdr, sizeof(hdr));
	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	return true;

}

bool GetData0xDF(const RawDataHdr3& hdr, unsigned char* pdat, int with_chk, RawData& dat)
{
	unsigned short sum = hdr.angle + hdr.N + hdr.span, chk;

	sum += hdr.fbase;
	sum += hdr.first;
	sum += hdr.last;
	sum += hdr.fend;

	double dan = (hdr.last - hdr.first) / double(hdr.N - 1);

	for (int i = 0; i < hdr.N; i++)
	{
		dat.points[i].confidence = *pdat++;
		sum += dat.points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short vv = (v2 << 8) | v;
		sum += vv;
		dat.points[i].distance = vv / 1000.0;
		dat.points[i].angle = (hdr.first + dan * i) * PI / 18000;
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum)
	{
		DEBUG_PR("chksum df error");
		return 0;
	}

	memcpy(&dat, &hdr, HDR2_SIZE);
	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	return true;

}

bool GetData0x99(const RawDataHdr99& hdr, unsigned char* pdat, int with_chk, RawData& dat)
{
	dat.code = hdr.code;
	dat.N = hdr.N;
	dat.angle = hdr.from * 3600 / hdr.total; // 0.1 degree
	dat.span = hdr.N * 3600 / hdr.total; // 0.1 degree
	//dat.fbase = ;
	//dat.first;
	//dat.last;
	//dat.fend;
	DecTimestamp(hdr.timestamp, dat.ts);

	pdat += HDR99_SIZE;

	uint8_t* dist = pdat;
	uint8_t* energy = pdat + 2 * hdr.N;

	for (int i = 0; i < hdr.N; i++)
	{
		uint16_t lo = *dist++;
		uint16_t hi = *dist++;
		dat.points[i].distance = ((hi << 8) | lo) / 1000.0;
		dat.points[i].angle = ((i + hdr.from) * 360.0 / hdr.total) * PI / 180;
		dat.points[i].confidence = energy[i];
	}
	return true;
}

//int getDataZone(LidarMsgHdr* data, char* result)
//{
//	if (data->flags >= 0x100)
//	{
//		//?????LMSG_ALARM???????
//		if (getbit(data->events, 12) == 1)
//		{
//			INFO_PR("ALARM LEVEL:OBSERVE  MSG TYPE:%d ZONE ACTIVE:%x\n", data->flags, data->zone_actived);
//		}
//		if (getbit(data->events, 13) == 1)
//		{
//			INFO_PR("ALARM LEVEL:WARM  MSG TYPE:%d ZONE ACTIVE:%x\n", data->flags, data->zone_actived);
//		}
//		if (getbit(data->events, 14) == 1)
//		{
//			INFO_PR("ALARM LEVEL:ALARM  MSG TYPE:%d ZONE ACTIVE:%x\n", data->flags, data->zone_actived);
//		}
//		if (getbit(data->events, 15) == 1)
//		{
//			DEBUG_PR("ALARM COVER   MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 16) == 1)
//		{
//			DEBUG_PR("ALARM NO DATA   MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 17) == 1)
//		{
//			DEBUG_PR("ALARM ZONE NO ACTIVE  MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 18) == 1)
//		{
//			DEBUG_PR("ALARM SYSTEM ERROR  MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 19) == 1)
//		{
//			DEBUG_PR("ALARM RUN EXCEPTION  MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 20) == 1)
//		{
//			DEBUG_PR("ALARM NETWORK ERROR  MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 21) == 1)
//		{
//			DEBUG_PR("ALARM UPDATING  MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 22) == 1)
//		{
//			DEBUG_PR("ALARM ZERO POS ERROR  MSG TYPE:%d\n", data->flags);
//		}
//	}
//	else if (data->flags % 2 == 1)
//	{
//		if (getbit(data->events, 0) == 1)
//		{
//			DEBUG_PR("LIDAR LOW POWER  MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 1) == 1)
//		{
//			DEBUG_PR("LIDAR  MOTOR STALL  MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 2) == 1)
//		{
//			DEBUG_PR("LIDAR RANGING MODULE TEMPERATURE HIGH  MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 3) == 1)
//		{
//			DEBUG_PR("LIDAR NETWORK ERROR  MSG TYPE:%d\n", data->flags);
//		}
//		if (getbit(data->events, 4) == 1)
//		{
//			DEBUG_PR("LIDAR RANGER MODULE NO OUTPUT  MSG TYPE:%d\n", data->flags);
//		}
//	}
//}


int ParseAPI::parse_data_x(int len, unsigned char* buf,
	int& span, int& is_mm, int& with_conf,
	RawData& dat, int& consume, int with_chk, LidarMsgHdr& zone, FanSegment** fan_segs)
{
	int pack_format = 0xce;
	int idx = 0;
	while (idx < len - 18)
	{
		if (memcmp(buf, "LMSG", 4) == 0)
		{
			LidarMsgHdr* hdr = (LidarMsgHdr*)(buf + idx);
			memcpy(&zone, hdr, sizeof(LidarMsgHdr));
			consume = idx + sizeof(LidarMsgHdr);
			return 2;
		}
		else if (buf[idx] == 0x4c && buf[idx + 1] == 0x48 && (unsigned char)buf[idx + 2] == 0xbe && (unsigned char)buf[idx + 3] == 0xb4)
		{
			consume = idx + len;
			return 3;
		}
		else if (buf[idx] == 'S' && buf[idx + 1] == 'T' && buf[idx + 6] == 'E' && buf[idx + 7] == 'D')
		{
			unsigned char flag = buf[idx + 2];
			span = 360;
			if (flag & 0x10) span = 180;
			if (flag & 0x20) span = 90;
			with_conf = flag & 2;
			is_mm = flag & 1;
			idx += 8;
		}

		else if (buf[idx + 1] == 0xfa && (buf[idx] == 0xdf || buf[idx] == 0xce || buf[idx] == 0xcf || buf[idx] == 0xc7 || buf[idx] == 0x9d || buf[idx] == 0x99))
		{
			pack_format = buf[idx];
		}
		else
		{
			idx++;
			continue;
		}

		RawDataHdr hdr;
		memcpy(&hdr, buf + idx, HDR_SIZE);
		if (buf[idx] == 0x99) {
			RawDataHdr99 hdr99;
			memcpy(&hdr99, buf + idx, HDR99_SIZE);
			if (hdr99.total == 0)
			{
				printf("bad num hdr99 \n");
				idx += 2;
				continue;
			}
			int hdr99_span = hdr99.N * 3600 / hdr99.total;
			if (hdr99_span % 90 != 0)
			{
				printf("bad angle %d \n", hdr99_span);
				idx += 2;
				continue;
			}
		}
		else if (buf[idx] != 0xc7) {
			if (hdr.angle % 90 != 0) {
				printf("bad angle %d \n", hdr.angle);
				idx += 2;
				continue;
			}
		}

		if (hdr.N > MAX_POINTS)
		{
			printf("points number %d seem not correct\n", hdr.N);
			idx += 2;
			continue;
		}

		bool got;
		if (buf[idx] == 0xce && idx + HDR_SIZE + hdr.N * 3 + 2 <= len)
		{
			got = GetData0xCE(hdr, buf + idx + HDR_SIZE,
				hdr.angle == 3420 ? span * 2 : span,
				with_chk, dat);
			consume = idx + HDR_SIZE + 3 * hdr.N + 2;
		}
		else if (buf[idx] == 0x9d && idx + hdr.N * 2 + 10 <= len)
		{
			RawDataHdr2 hdr2;
			memcpy(&hdr2, buf + idx, HDR2_SIZE);
			got = GetData0x9D(hdr2, buf + idx + HDR2_SIZE, with_chk, dat);
			consume = idx + HDR2_SIZE + 2 * hdr.N + 2;
		}
		else if (buf[idx] == 0xdf && idx + hdr.N * 3 + 18 <= len)
		{
			RawDataHdr3 hdr3;
			memcpy(&hdr3, buf + idx, HDR3_SIZE);
			got = GetData0xDF(hdr3, buf + idx + HDR3_SIZE, with_chk, dat);
			consume = idx + HDR3_SIZE + 3 * hdr.N + 2;
		}
		else if (buf[idx] == 0xcf && idx + HDR2_SIZE + hdr.N * 3 + 2 <= len)
		{
			RawDataHdr2 hdr2;
			memcpy(&hdr2, buf + idx, HDR2_SIZE);
			got = GetData0xCF(hdr2, buf + idx + HDR2_SIZE, with_chk, dat);
			consume = idx + HDR2_SIZE + 3 * hdr.N + 2;
		}
		else if (buf[idx] == 0xc7 && idx + HDR7_SIZE + hdr.N * 5 + 2 <= len)
		{
			RawDataHdr7 hdr7;
			memcpy(&hdr7, buf + idx, HDR7_SIZE);

			got = GetData0xC7(hdr7, buf + idx, with_chk, dat, fan_segs);

			consume = idx + HDR7_SIZE + 5 * hdr.N + 2;
		}
		else if (buf[idx] == 0x99 && idx + HDR99_SIZE + hdr.N * 3 + 2 <= len)
		{
			RawDataHdr99 hdr99;
			memcpy(&hdr99, buf + idx, HDR99_SIZE);

			got = GetData0x99(hdr99, buf + idx, with_chk, dat);

			consume = idx + HDR99_SIZE + 3 * hdr.N + 2;
		}

		else {
			// data packet not complete
			break;
		}
		return got;
	}

	if (idx > 1024) consume = idx / 2;
	return false;
}

bool ParseAPI::parse_data(int len, unsigned char* buf,
	int& span, int& is_mm, int& with_conf,
	RawData& dat, int& consume, int with_chk)
{
	int idx = 0;
	int pack_format = 0xce;
	while (idx < len - 180)
	{
		if (buf[idx] == 'S' && buf[idx + 1] == 'T' && buf[idx + 6] == 'E' && buf[idx + 7] == 'D')
		{
			unsigned char flag = buf[idx + 2];
			span = 360;
			if (flag & 0x10) span = 180;
			if (flag & 0x20) span = 90;
			with_conf = flag & 2;
			is_mm = flag & 1;
			idx += 8;
		}

		if (buf[idx] != 0xce || buf[idx + 1] != 0xfa)
		{
			idx++;
			continue;
		}
		pack_format = buf[idx];

		RawDataHdr hdr;
		memcpy(&hdr, buf + idx, HDR_SIZE);

		if ((hdr.angle % 360) != 0)
		{
			printf("bad angle %d\n", hdr.angle);
			idx += 2;
			continue;
		}

		if (hdr.N > 300 || hdr.N < 30)
		{
			printf("points number %d seem not correct\n", hdr.N);
			idx += 2;
			continue;
		}

		if (idx + HDR_SIZE + hdr.N * sizeof(short) + 2 > len)
		{
			// data packet not complete
			break;
		}

		// calc checksum
		unsigned short sum = hdr.angle + hdr.N, chk;
		unsigned char* pdat = buf + idx + HDR_SIZE;
		for (int i = 0; i < hdr.N; i++)
		{
			unsigned short v = *pdat++;
			unsigned short v2 = *pdat++;
			unsigned short val = (v2 << 8) | v;

			if (with_conf)
			{
				dat.points[i].confidence = val >> 13;
				dat.points[i].distance = val & 0x1fff;
				dat.points[i].distance /= (is_mm ? 1000.0 : 100.0);
			}
			else {
				dat.points[i].distance = is_mm ? val / 1000.0 : val / 100.0;
				dat.points[i].confidence = 0;
			}

			dat.points[i].angle = (hdr.angle + 360.0 * i / hdr.N) * PI / 1800;

			sum += val;
		}
		memcpy(&chk, buf + idx + HDR_SIZE + hdr.N * 2, 2);

		if (with_chk != 0 && chk != sum)
		{
			DEBUG_PR("chksum error");
			consume = idx + HDR_SIZE + 2 * hdr.N + 2;
			return 0;
		}

		memcpy(&dat, &hdr, HDR_SIZE);
		dat.span = 360;
		// memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);

		idx += HDR_SIZE + 2 * hdr.N + 2;
		consume = idx;
		return true;
	}

	if (idx > 1024) consume = idx / 2;
	return false;
}

void UserAPI::fan_data_process(const RawData& raw, const char* output_file, PointData& tmp)
{
	RawData* data = new RawData;
	memcpy(data, &raw, sizeof(RawData));
	uint32_t timestamp[2] = { 0 };
	timestamp[0] = data->ts[0];
	timestamp[1] = data->ts[1];


	//PointData tmp;
	tmp.N = data->N;
	memcpy(tmp.ts, data->ts, sizeof(uint32_t) * 2);
	memcpy(tmp.points, data->points, sizeof(DataPoint) * data->N);
	//??л??????
	//((void(*)(int, void *))msgptr)(1, &tmp);

	if (output_file != NULL)
	{
		FILE* fp = fopen(output_file, "w");
		if (fp)
		{
			for (int i = 0; i < data->N; i++)
			{
				fprintf(fp, "%.5f\t%.3f\t%d\n", data->points[i].angle, data->points[i].distance, data->points[i].confidence);
			}
			fclose(fp);
		}
	}
	delete data;
}

void UserAPI::whole_data_process(const RawData& raw, bool from_zero, int collect_angle, const char* output_file, PointData& tmp, std::vector<RawData*> &whole_datas)
{
	RawData* data = new RawData;
	memcpy(data, &raw, sizeof(RawData));
	whole_datas.push_back(data);
	if ((from_zero && (raw.angle + raw.span) % 3600 == 0) || (!from_zero && raw.angle + raw.span == 1800 + collect_angle * 10))
	{
	}
	else
	{
		return;
	}
	bool bfirst = true;
	uint32_t timestamp[2] = { 0 };
	int count = 0, n = 0, angles = 0;

	for (std::vector<RawData*>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
	{
		RawData* tmp = *it;
		// accumulate point's angle and counts
		angles += tmp->span;
		count += tmp->N;
		// record first point timestamp
		if (bfirst)
		{
			timestamp[0] = tmp->ts[0];
			timestamp[1] = tmp->ts[1];
			bfirst = false;
		}
		// record fan counts
		n++;
	}
	if (angles != 3600)
	{
		DEBUG_PR("angle sum %d, drop %d fans %d points\n", angles, n, count);
		for (std::vector<RawData*>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
		{
			RawData* tmp = *it;
			delete tmp;
		}
		whole_datas.clear();
	}

	DataPoint* points = new DataPoint[count]; // store whole circle points
	count = 0;
	for (std::vector<RawData*>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
	{
		RawData* tmp = *it;
		for (int i = 0; i < tmp->N; i++)
		{
			points[count++] = tmp->points[i];
		}
		delete tmp;
	}
	whole_datas.clear();
	tmp.N = count;
	memcpy(tmp.ts, timestamp, sizeof(uint32_t) * 2);
	memcpy(tmp.points, points, sizeof(DataPoint) * count);

	if (output_file != NULL)
	{
		FILE* fp = fopen(output_file, "a");
		if (fp)
		{
			for (int i = 0; i < count; i++)
			{
				fprintf(fp, "%s\t%d \t %d\t%.5f\t%.3f\t%d\n",
					tmp.ip,tmp.port,tmp.N,
					points[i].angle,
					points[i].distance, points[i].confidence);
			}
			fclose(fp);
		}
	}
	delete[] points;
}
bool judgepcIPAddrIsValid(const char* pcIPAddr)
{
	int iDots = 0; /* 字符.的个数 */
	int iSetions = 0; /* pcIPAddr 每一部分总和（0-255）*/

	if (NULL == pcIPAddr || *pcIPAddr == '.') { /*排除输入参数为NULL, 或者一个字符为'.'的字符串*/
		return false;
	}

	/* 循环取每个字符进行处理 */
	while (*pcIPAddr)
	{
		if (*pcIPAddr == '.')
		{
			iDots++;
			/* 检查 pcIPAddr 是否合法 */
			if (iSetions >= 0 && iSetions <= 255)
			{
				iSetions = 0;
				pcIPAddr++;
				continue;
			}
			else
			{
				return false;
			}
		}
		else if (*pcIPAddr >= '0' && *pcIPAddr <= '9') 	/*判断是不是数字*/
		{
			iSetions = iSetions * 10 + (*pcIPAddr - '0'); /*求每一段总和*/
		}
		else
		{
			return false;
		}
		pcIPAddr++;
	}

	/* 判断最后一段是否有值 如：1.1.1. */
	if ((*pcIPAddr == '\0') && (*(pcIPAddr - 1) == '.'))
	{
		return false;
	}

	/* 判断最后一段是否合法 */
	if (iSetions >= 0 && iSetions <= 255)
	{
		if (iDots == 3)
		{
			return true;
		}
	}

	return false;
}
bool BaseAPI::checkAndMerge(int type, char* ip, char* mask, char* gateway, int port, char* result)
{
	std::string s = ip;
	int a[4] = { 0 };
	for (int i = 0; i < 4; i++)
	{
		int tmp = s.find('.', 0);
		std::string str = s.substr(0, tmp);
		a[i] = atoi(str.c_str());
		s = s.substr(tmp + 1);
		if (a[0] < 10)
			return false;
	}
	if (!judgepcIPAddrIsValid(ip))
		return false;
	if (port <= 1000 || port > 65535)
		return false;
	if (type == 0)
	{
		sprintf(result, "%03d.%03d.%03d.%03d %05d", a[0], a[1], a[2], a[3], port);
		return true;
	}
	else
	{
		if (!judgepcIPAddrIsValid(mask) || !judgepcIPAddrIsValid(gateway))
			return false;
		int b[4] = { 0 };
		s = mask;
		for (int i = 0; i < 4; i++)
		{
			int tmp = s.find('.', 0);
			std::string str = s.substr(0, tmp);
			b[i] = atoi(str.c_str());
			s = s.substr(tmp + 1);
			if (b[0] == 0)
				return false;
		}
		int c[4] = { 0 };
		s = gateway;
		for (int i = 0; i < 4; i++)
		{
			int tmp = s.find('.', 0);
			std::string str = s.substr(0, tmp);
			c[i] = atoi(str.c_str());
			s = s.substr(tmp + 1);
			if (c[0] == 0)
				return false;
		}

		unsigned int str1 = 0;
		unsigned int str2 = 0;
		unsigned int str3 = 0;

		//字符串转整形
		//sscanf(ip, "%d.%d.%d.%d", &nTmpIP[0], &nTmpIP[1], &nTmpIP[2], &nTmpIP[3]);
		for (int i = 0; i < 4; i++)
		{
			str1 += (a[i] << (24 - (i * 8)) & 0xFFFFFFFF);
		}
		for (int i = 0; i < 4; i++)
		{
			str2 += (b[i] << (24 - (i * 8)) & 0xFFFFFFFF);
		}
		for (int i = 0; i < 4; i++)
		{
			str3 += (c[i] << (24 - (i * 8)) & 0xFFFFFFFF);
		}
		if ((str1 & str2) != (str2 & str3))
			return false;


		sprintf(result, "%03d.%03d.%03d.%03d %03d.%03d.%03d.%03d %03d.%03d.%03d.%03d %05d",
			a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3], c[0], c[1], c[2], c[3], port);
		return true;
	}
}