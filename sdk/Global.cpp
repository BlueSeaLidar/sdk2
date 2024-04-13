#include "Global.h"

static FanSegment_C7* GetFanSegment(const RawDataHdr7& hdr, uint8_t* pdat, bool with_chk);
static FanSegment_AA* GetFanSegment(const RawDataHdrAA& hdr, uint8_t* pdat, bool with_chk);
static int GetFanPointCount(FanSegment_C7* seg);
static int GetFanPointCount(FanSegment_AA* seg);
static void PackFanData(FanSegment_C7* seg, RawData& rdat);
static void PackFanData(FanSegment_AA* seg, RawData& rdat);



double getAngleWithViewpoint(float r1, float r2, double included_angle)
{
	return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
}
int int_cmper(const void* p1, const void* p2)
{
	int* d1 = (int*)p1;
	int* d2 = (int*)p2;
	return *d1 - *d2;
}

unsigned int stm32crc(unsigned int* ptr, unsigned int len)
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
std::string BaseAPI::stringfilter(char *str, int num)
{
    int index = 0;
    for(int i=0;i<num;i++)
    {
       if((str[i]>=45&&str[i]<=58)||(str[i]>=65&&str[i]<=90)||(str[i]>=97&&str[i]<=122))
       {
          index++;
       }
       else
       {
           std::string arr = str;
           arr=arr.substr(0,index);
           return   arr;
       }
    }
    return "";
}

int AlgorithmAPI_E100::ShadowsFilter(UserData* scan_in, const ShadowsFilterParam& param)
{
	double angle_increment = PI * 2 / scan_in->data.framedata.N;

	std::set<int> indices_to_delete;

	for (int i = 0; i < scan_in->data.framedata.N - param.window - 1; i++)
	{
		if (scan_in->data.framedata.data[i].distance < 0.002) continue;

		for (int y = 1; y < param.window + 1; y++)
		{
			int j = i + y;
			if (j < 0 || j >= (int)scan_in->data.framedata.N || (int)i == j)
				continue;
			if (scan_in->data.framedata.data[j].distance < 0.002)
				continue;
			if (fabs(scan_in->data.framedata.data[i].distance -
				scan_in->data.framedata.data[j].distance) < 0.2)
				continue;

			double rad = getAngleWithViewpoint(
				scan_in->data.framedata.data[i].distance,
				scan_in->data.framedata.data[j].distance,
				y * angle_increment);

			double angle = abs(rad * 180 / PI);
			if (angle < param.min_angle || angle > param.max_angle)
			{
				int from, to;
				if (scan_in->data.framedata.data[i].distance < scan_in->data.framedata.data[j].distance)
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
		scan_in->data.framedata.data[*it].distance = 0;
		nr++;
	}

	return nr;
}


int AlgorithmAPI_E100::MedianFilter(UserData* scan_in, const MedianFilterParam& param)
{
	int* dists = new int[scan_in->data.framedata.N];
	int* buf = new int[param.window * 2 + 1];

	for (unsigned int i = 0; i < scan_in->data.framedata.N; i++)
		dists[i] = scan_in->data.framedata.data[i].distance * 1000;

	for (int i = param.window; i < scan_in->data.framedata.N - param.window - 1; i++)
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
			scan_in->data.framedata.data[i].distance = (float)(buf[param.window] / 1000.0);
		}
	}

	delete[] dists;
	delete[] buf;

	return 0;
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



static bool GetData0xC7(const RawDataHdr7& hdr, uint8_t* pdat, bool with_chk, RawData& dat,char *result, FanSegment_C7** last_fan_seg)
{
	bool got = false;

	FanSegment_C7* fan_seg = GetFanSegment(hdr, pdat, with_chk);
	if (!fan_seg) {
		return NULL;
	}
	if (*last_fan_seg != NULL)
	{
		FanSegment_C7* seg = (*last_fan_seg);

		if (seg->hdr.timestamp != fan_seg->hdr.timestamp)
		{
            //printf("drop old fan segments\n");
            strcpy(result,"drop old fan segments");
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
                    strcpy(result,"drop duplicated segment");
                    //printf("drop duplicated segment\n");
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
	}

	int N = GetFanPointCount((*last_fan_seg));

	if (N >= (*last_fan_seg)->hdr.whole_fan)
	{
		if (N == (*last_fan_seg)->hdr.whole_fan)
		{
			if (N > sizeof(dat.points) / sizeof(dat.points[0]))
			{
                //printf("too many %d points in 1 fan\n", N);
                sprintf(result,"too many %d points in 1 fan",N);

			}
			else
			{
				PackFanData((*last_fan_seg), dat);
				got = true;
			}
		}

		// remove segments	
		FanSegment_C7* seg = (*last_fan_seg);
		while (seg) {
			(*last_fan_seg) = seg->next;
			delete seg;
			seg = (*last_fan_seg);
		}
	}
	return got;
}
bool GetData0xAA(const RawDataHdrAA& hdr, uint8_t* pdat, bool with_chk, RawData& dat, char* result, FanSegment_AA** last_fan_seg)
{
	bool got = false;

	FanSegment_AA* fan_seg = GetFanSegment(hdr, pdat, with_chk);
	if (!fan_seg) {
		return false;
	}
	if (*last_fan_seg != NULL)
	{
		FanSegment_AA* seg = (*last_fan_seg);

		if ((seg->hdr.second != fan_seg->hdr.second) && (seg->hdr.nano_sec != fan_seg->hdr.nano_sec))
		{
			//printf("drop old fan segments\n");
			strcpy(result, "drop old fan segments");
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
					strcpy(result, "drop duplicated segment");
					//printf("drop duplicated segment\n");
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
	}

	unsigned int N = GetFanPointCount((*last_fan_seg));

	if (N >= (*last_fan_seg)->hdr.whole_fan)
	{
		if (N == (*last_fan_seg)->hdr.whole_fan)
		{
			if (N > sizeof(dat.points) / sizeof(dat.points[0]))
			{
				//printf("too many %d points in 1 fan\n", N);
				sprintf(result, "too many %d points in 1 fan", N);

			}
			else
			{
				PackFanData((*last_fan_seg), dat);
				got = true;
			}
		}

		// remove segments
		FanSegment_AA* seg = (*last_fan_seg);
		while (seg) {
			(*last_fan_seg) = seg->next;
			delete seg;
			seg = (*last_fan_seg);
		}
	}
	return got;
}

bool GetData0xCE(const RawDataHdr& hdr, unsigned char* pdat, int span, int with_chk, RawData& dat,char*result)
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
        printf("chksum ce error");
        strcpy(result,"chksum  error");
		return false;
	}

	memcpy(&dat, &hdr, HDR_SIZE);
	dat.span = span;
	return true;
}

bool GetData0x9D(const RawDataHdr2& hdr, unsigned char* pdat, int with_chk, RawData& dat,char*result)
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
        printf("chksum cf error");
        strcpy(result,"chksum  error");
		return 0;
	}

	memcpy(&dat, &hdr, sizeof(hdr));
	return true;

}


bool GetData0xCF(const RawDataHdr2& hdr, unsigned char* pdat, int with_chk, RawData& dat,char*result)
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
		dat.points[i].angle = (hdr.angle + hdr.span * i / (double)hdr.N) * PI / 1800;
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum)
	{
        printf("chksum cf error");
        strcpy(result,"chksum  error");
		return 0;
	}

	memcpy(&dat, &hdr, sizeof(hdr));
	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	return true;

}

bool GetData0xDF(const RawDataHdr3& hdr, unsigned char* pdat, int with_chk, RawData& dat,char*result)
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
        //printf("chksum df error");
        strcpy(result,"chksum  error");
		return 0;
	}

	memcpy(&dat, &hdr, HDR2_SIZE);
	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	return true;

}

bool GetData0x99(const RawDataHdr99& hdr, unsigned char* pdat, int with_chk, RawData& dat,char*result)
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



int ParseAPI::parse_data_x(int len, unsigned char* buf,UartState *uartstate, RawData& dat, int& consume, int with_chk, char*result, CmdHeader *cmdheader,void** fan_segs)
{
	int pack_format = 0xce;
	int idx = 0;
    int span=360;
    while (idx < len - 12)
	{
        //qDebug()<<hex<<buf[idx]<<buf[idx+1]<<(unsigned char)buf[idx+2]<<(unsigned char)buf[idx+3]<<endl;
        //LMSG
        if (buf[idx] == 0x4c && buf[idx + 1] == 0x4d && (unsigned char)buf[idx + 2] == 0x53 && (unsigned char)buf[idx + 3] == 0x47)
		{
			LidarMsgHdr* hdr = (LidarMsgHdr*)(buf + idx);
            memcpy(result, hdr, sizeof(LidarMsgHdr));
			consume = idx + sizeof(LidarMsgHdr);
			return 2;
		}
        //EPRM
        else if (buf[idx] == 0x4c && buf[idx + 1] == 0x48 && (unsigned char)buf[idx + 2] == 0xac && (unsigned char)buf[idx + 3] == 0xb8)
        {

            memcpy(cmdheader, buf, sizeof(CmdHeader));
            EEpromV101*eepromv101 = (EEpromV101*)(buf + 8+idx);
            memcpy(result, eepromv101, sizeof(EEpromV101));
            consume = idx + sizeof(EEpromV101);
            return 3;
        }
        //时间同步返回的应答
        else if (buf[idx] == 0x4c && buf[idx + 1] == 0x48 && (unsigned char)buf[idx + 2] == 0xbe && (unsigned char)buf[idx + 3] == 0xb4)
		{
            consume = idx + 2;
            return 4;
		}
        //C_PACK
        else if (buf[idx] == 0x4c && buf[idx + 1] == 0x48 && (unsigned char)buf[idx + 2] == 0xbc && (unsigned char)buf[idx + 3] == 0xff)
        {
            consume = idx+2;
            memcpy(cmdheader, buf, sizeof(CmdHeader));
            return 5;
        }
        //S_PACK
        else if (buf[idx] == 0x4c && buf[idx + 1] == 0x48 && (unsigned char)buf[idx + 2] == 0xac && (unsigned char)buf[idx + 3] == 0xff)
        {
            consume = idx + 2;
            memcpy(cmdheader, buf, sizeof(CmdHeader));
            memcpy(result,buf+idx+8,2);
            return 6;
        }
        //readzone
        else if (buf[idx] == 0x4c && buf[idx + 1] == 0x48 && (unsigned char)buf[idx + 2] == 0xb8 && (unsigned char)buf[idx + 3] == 0xa5)
        {
            consume = idx+4;
            memcpy(cmdheader, buf, sizeof(CmdHeader));
            return 7;
        }
        //writezone
        else if (buf[idx] == 0x4c && buf[idx + 1] == 0x48 && (unsigned char)buf[idx + 2] == 0xac && (unsigned char)buf[idx + 3] == 0xa5)
        {
            consume = idx + 4;
            memcpy(cmdheader, buf, sizeof(CmdHeader));
            return 8;
        }

		if (buf[idx] == 'S' && buf[idx + 1] == 'T' && buf[idx + 6] == 'E' && buf[idx + 7] == 'D')
		{
			unsigned char flag = buf[idx + 2];
            uartstate->unit_mm=flag & 1;
            uartstate->with_conf = flag & 2;
            uartstate->with_smooth = flag & 4;
            uartstate->with_fitter = flag & 8;
            uartstate->span_9 = flag & 0x10;
            uartstate->span_18 = flag & 0x20;
            uartstate->span_other = flag & 0x40;
            uartstate->resampele = flag & 0x80;
            if(flag & 0x10)
                span = 180;
            if(flag & 0x20)
                span = 90;
            consume = idx + 8;
            return 9;
		}

		if (buf[idx + 1] == 0xfa && (buf[idx] == 0xdf || buf[idx] == 0xce || buf[idx] == 0xcf || buf[idx] == 0xc7 || buf[idx] == 0x9d || buf[idx] == 0x99 || buf[idx] == 0xaa))
		{
			// found;
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
		else if (buf[idx] != 0xc7&& buf[idx] != 0xaa) {
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
                with_chk, dat,result);
			consume = idx + HDR_SIZE + 3 * hdr.N + 2;
		}
		else if (buf[idx] == 0x9d && idx + hdr.N * 2 + 10 <= len)
		{
			RawDataHdr2 hdr2;
			memcpy(&hdr2, buf + idx, HDR2_SIZE);
            got = GetData0x9D(hdr2, buf + idx + HDR2_SIZE, with_chk, dat,result);
			consume = idx + HDR2_SIZE + 2 * hdr.N + 2;
		}
		else if (buf[idx] == 0xdf && idx + hdr.N * 3 + 18 <= len)
		{
			RawDataHdr3 hdr3;
			memcpy(&hdr3, buf + idx, HDR3_SIZE);
            got = GetData0xDF(hdr3, buf + idx + HDR3_SIZE, with_chk, dat,result);
			consume = idx + HDR3_SIZE + 3 * hdr.N + 2;
		}
		else if (buf[idx] == 0xcf && idx + HDR2_SIZE + hdr.N * 3 + 2 <= len)
		{
			RawDataHdr2 hdr2;
			memcpy(&hdr2, buf + idx, HDR2_SIZE);
            got = GetData0xCF(hdr2, buf + idx + HDR2_SIZE, with_chk, dat,result);
			consume = idx + HDR2_SIZE + 3 * hdr.N + 2;
		}
		else if (buf[idx] == 0xc7 && idx + HDR7_SIZE + hdr.N * 5 + 2 <= len)
		{
			RawDataHdr7 hdr7;
			memcpy(&hdr7, buf + idx, HDR7_SIZE);

            got = GetData0xC7(hdr7, buf + idx, with_chk, dat, result,(FanSegment_C7**)fan_segs);

			consume = idx + HDR7_SIZE + 5 * hdr.N + 2;
		}
		else if (buf[idx] == 0x99 && idx + HDR99_SIZE + hdr.N * 3 + 2 <= len)
		{
			RawDataHdr99 hdr99;
			memcpy(&hdr99, buf + idx, HDR99_SIZE);

            got = GetData0x99(hdr99, buf + idx, with_chk, dat,result);

			consume = idx + HDR99_SIZE + 3 * hdr.N + 2;
		}
		else if (buf[idx] == 0xaa && idx + HDRAA_SIZE + hdr.N * 3 + 2 <= len)
		{
			RawDataHdrAA hdraa;
			memcpy(&hdraa, buf + idx, HDRAA_SIZE);

			got = GetData0xAA(hdraa, buf + idx, with_chk, dat, result, (FanSegment_AA**)fan_segs);

			consume = idx + HDRAA_SIZE + 3 * hdr.N + 2;
		}
		else {
			// data packet not complete
			break;
		}
		return got;
	}

	if (idx > 1024) consume = idx / 2;

    return -1;
}

int ParseAPI::parse_data(int len, unsigned char* buf,UartState *uartstate,RawData& dat, int& consume, int with_chk)
{
	int idx = 0;
	int pack_format = 0xce;
    int span=360;
	while (idx < len - 180)
	{
		if (buf[idx] == 'S' && buf[idx + 1] == 'T' && buf[idx + 6] == 'E' && buf[idx + 7] == 'D')
		{
            unsigned char flag = buf[idx + 2];
            uartstate->unit_mm=flag & 1;
            uartstate->with_conf = flag & 2;
            uartstate->with_smooth = flag & 4;
            uartstate->with_fitter = flag & 8;
            uartstate->span_9 = flag & 0x10;
            uartstate->span_18 = flag & 0x20;
            uartstate->span_other = flag & 0x40;
            uartstate->resampele = flag & 0x80;
            if(flag & 0x10)
                span = 180;
            if(flag & 0x20)
                span = 90;
            consume = idx + 8;
            return 9;
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

            if (uartstate->with_conf)
			{
				dat.points[i].confidence = val >> 13;
				dat.points[i].distance = val & 0x1fff;
                dat.points[i].distance /= (uartstate->unit_mm ? 1000.0 : 100.0);
			}
			else {
                dat.points[i].distance = uartstate->unit_mm ? val / 1000.0 : val / 100.0;
				dat.points[i].confidence = 0;
			}

			dat.points[i].angle = (hdr.angle + 360.0 * i / hdr.N) * PI / 1800;

			sum += val;
		}
		memcpy(&chk, buf + idx + HDR_SIZE + hdr.N * 2, 2);

		if (with_chk != 0 && chk != sum)
		{
            printf("chksum error");
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

void UserAPI::fan_data_process(const RawData& raw, std::vector<RawData>& whole_datas)
{
	whole_datas.push_back(raw);
}

int UserAPI::whole_data_process(const RawData& raw,int collect_angle, std::vector<RawData> &whole_datas,std::string &error)
{
	char tmp[128]={0};
    whole_datas.push_back(raw);
    if ((raw.angle + raw.span)%3600 == collect_angle)
	{
        //int a=1;
	}
	else
	{
       // whole_datas.push_back(raw);
        return 0;
	}
    //printf("%d \n",whole_datas.size());
	int count = 0, n = 0, angles = 0;

    for (std::vector<RawData>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
	{
        RawData tmp = *it;
        angles += tmp.span;
        count += tmp.N;
		n++;
	}
    if (angles != 3600)
    {
        sprintf(tmp,"angle sum %d, drop %d fans %d points",angles, n, count);
        error = tmp;
        whole_datas.clear();
        return -1;
    }
    return 1;
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

int find(std::vector<RawData>a, int n, int x)
{
    int i;
    int min = abs(a.at(0).angle - x);
    int r = 0;

    for (i = 0; i < n; ++i)
    {
        if (abs(a.at(i).angle - x) < min)
        {
            min = abs(a.at(i).angle - x);
            r = i;
        }
    }

    return a[r].angle;
}
int UserAPI::autoGetFirstAngle(const RawData &raw, bool from_zero, std::vector<RawData> &raws,std::string &error)
{
    int angles = 0;
    char tmp_error[128]={0};
    int size = raws.size();
    if(size>=1)
    {
        RawData tmp = raws.at(size-1);
        RawData tmp2 = raws.at(0);
        if(raw.angle==tmp2.angle)
        {
            for (int i=0;i<size;i++)
            {
                tmp = raws.at(i);
                angles += tmp.span;
            }
            if (angles != 3600)
            {

                sprintf(tmp_error,"angle sum %d\n", angles);
                error = tmp_error;
                raws.clear();
                return -2;
            }
            else
            {
                int ret=-1;
                if(from_zero)
                     ret=find(raws,raws.size(),0);
                else
                     ret=find(raws,raws.size(),1800);

                raws.clear();
                return ret;
            }
        }
        if(raw.angle==(tmp.angle+tmp.span)%3600)
        {
            //说明是连续的扇区
            raws.push_back(raw);

        }

    }
    else
        raws.push_back(raw);

    return -1;

}


bool uart_talk(int fd, int n, const char* cmd,
	int nhdr, const char* hdr_str,
	int nfetch, char* fetch)
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
				if (strcmp(cmd, "LXVERH") == 0 || strcmp(cmd, "LUUIDH") == 0 || strcmp(cmd, "LTYPEH") == 0)
				{
					memcpy(fetch, buf + i + nhdr, nfetch);
					fetch[nfetch] = 0;
				}
				else
				{
					strcpy(fetch, "OK");
					fetch[3] = 0;
				}
			}
			return true;
		}
		else if (memcmp(buf + i, cmd, n) == 0)
		{
			if (nfetch > 0)
			{
				memcpy(fetch, buf + i + n + 1, 2);
				fetch[2] = 0;
			}
			return true;
		}
		else if (memcmp(buf + i, "unsupport", 9) == 0)
		{
			if (nfetch > 0)
			{
				strcpy(fetch, "unsupport");
				fetch[10] = 0;
			}
			return false;
		}
	}

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return false;
}

bool vpc_talk(int hcom, int mode, short sn, int len, const char* cmd, int nfetch, void* result)
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

	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);
	//pcrc[0] = BaseAPI::stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	int nr = write(hcom, buffer, len2);
	char buf[2048];
	int index = 10;
	//4C 48 BC FF   xx xx xx xx  result
	//读取之后的10*2048个长度，如果不存在即判定失败
	while (index--)
	{
		nr = read(hcom, buf, sizeof(buf));
		while (nr < (int)sizeof(buf))
		{
			int n = read(hcom, buf + nr, sizeof(buf) - nr);
			if (n > 0) nr += n;
		}

		for (int i = 0; i < (int)sizeof(buf) - nfetch; i++)
		{
			if (mode == C_PACK)
			{
				char* fetch = (char*)result;
				if (buf[i] == 0x4C && buf[i + 1] == 0x48 && buf[i + 2] == (signed char)0xBC && buf[i + 3] == (signed char)0xFF)
				{
					/*int packSN = ((unsigned int)buf[i + 5] << 8) | (unsigned int)buf[i + 4];
					if (packSN != sn)
						continue;*/

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
			else if (mode == S_PACK)
			{
				if ((buf[i + 2] == (signed char)0xAC && buf[i + 3] == (signed char)0xB8) || (buf[i + 2] == (signed char)0xAC && buf[i + 3] == (signed char)0xff))
				{
					//printf("%02x  %02x\n", buf[i + 2], buf[i + 3]);
					//随机码判定
					short packSN = ((unsigned char)buf[i + 5] << 8) | (unsigned char)buf[i + 4];
					if (packSN != sn)
						continue;

					memcpy(result, buf + i + 8, nfetch);
					return true;
				}
			}
		}
	}
	printf("read %d bytes, not found %s\n", nr, cmd);
	return false;
}

void send_cmd_vpc(int hCom, int mode, int sn, int len, const char* cmd)
{
	char buffer[2048] = { 0 };
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = mode;
	hdr->sn = sn;
	len = ((len + 3) >> 2) * 4;

	hdr->len = len;
	memcpy(buffer + sizeof(CmdHeader), cmd, len);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);

	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);
	//pcrc[0] =BaseAPI::stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	write(hCom, buffer, len2);
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

int SystemAPI::GetComList(std::vector<UARTARG>& list)
{
	std::vector<int> port_list;
	bool isOK = false;
	port_list.push_back(230400);
	port_list.push_back(256000);
	port_list.push_back(384000);
	port_list.push_back(460800);
	port_list.push_back(500000);
	port_list.push_back(768000);
	port_list.push_back(921600);
	port_list.push_back(1000000);
	port_list.push_back(1500000);
	std::vector<std::string> portNames = SystemAPI::GetComPort();
	if (portNames.size() <= 0)
		return 0;


	for (unsigned int i = 0; i < portNames.size(); i++)
	{
		UARTARG arg;
		for (unsigned int j = 0; j < port_list.size(); j++)
		{
			int com_speed = port_list.at(j);
			if (int ret = GetDevInfoByUART(portNames.at(i).c_str(), com_speed))
			{
				arg.port = com_speed;
				strcpy(arg.portName, portNames.at(i).c_str());
				list.push_back(arg);
				isOK = true;
				break;
			}
		}
		if (isOK)
		{
			isOK = false;
			continue;
		}

		int com_speed = 12345;
		if (int ret = GetDevInfoByVPC(portNames.at(i).c_str(), com_speed))
		{
			strcpy(arg.portName, portNames.at(i).c_str());
			arg.port = -1;
			list.push_back(arg);
		}
	}
	return 0;
}
int GetDevInfoByUART(const char* port_str, int speed)
{
	int zeroNum = 0;
	unsigned long  wf = 0, rf = 0;

	unsigned int check_size = 4096;
	int hPort = SystemAPI::open_serial_port(port_str, speed);
	if (hPort <= 0) {
		return false;
	}

	char cmd[] = "LUUIDH";
	wf = write(hPort, cmd, sizeof(cmd));
	int bOK = false;
	unsigned char* buf = new unsigned char[check_size];
	while (rf < check_size)
	{
		int  tmp = read(hPort, buf + rf, check_size - rf);
		if (tmp > 0)
		{
			rf += tmp;
			//zeroNum = 0;
		}
		else
		{
			zeroNum++;
			msleep(1);
		}

		if (zeroNum > 10)
		{
			//printf("read 0 byte max index break\n");
			break;
		}
	}
	/*if (zeroNum <= 10)
		printf("read max byte break\n");*/

	if (rf > 10)
	{
		for (unsigned int idx = 0; idx < rf - 10; idx++)
		{
			if (memcmp((char*)buf + idx, "PRODUCT SN:", 11) == 0)
			{
				bOK = 1;
				break;
			}
		}
	}
	SystemAPI::closefd(hPort, false);
	delete[]buf;
	return bOK;
}

int GetDevInfoByVPC(const char* port_str, int speed)
{
	int zeroNum = 0;
	unsigned long  rf = 0;

	unsigned int check_size = 10240;
	int hPort = SystemAPI::open_serial_port(port_str, speed);
	if (hPort <= 0) {
		//MessageBox(NULL, "open port failed", "warm", 0);
		return false;
	}
	char cmd[] = "LUUIDH";
	CommunicationAPI::send_cmd_vpc((int)hPort, 0x0043, rand(), sizeof(cmd), cmd);
	int bOK = false;
	unsigned char* buf = new unsigned char[check_size];
	//遍历返回的信息中是否含有MCU VERSION:
	while (rf < check_size)
	{
		int  tmp = read(hPort, buf + rf, check_size - rf);
		if (tmp > 0)
		{
			rf += tmp;
			zeroNum = 0;
		}
		else
		{
			zeroNum++;
			msleep(1);
		}

		if (zeroNum > 3)
			break;

	}
	if (rf > 10)
	{
		for (unsigned int idx = 0; idx < rf - 10; idx++)
		{
			if (memcmp((char*)buf + idx, "PRODUCT SN:", 11) == 0)
			{
				//MessageBox(NULL, "3", test, 0);
				bOK = 2;
			}
			else if (memcmp((char*)buf + idx, "LMSG", 4) == 0)
			{
				//MessageBox(NULL, "4", test, 0);
				bOK = 3;
			}
		}
	}
	SystemAPI::closefd(hPort, false);
	delete[]buf;
	return bOK;
}
int SystemAPI::open_serial_port(const char* name, int speed)
{
	return Open_serial_port(name, speed);
}

int SystemAPI::open_socket_port(int localhost)
{
#ifdef _WIN32
	WSADATA   wsda; //   Structure   to   store   info   
	WSAStartup(MAKEWORD(2, 2), &wsda);
#endif // _WIN32

	int fd_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd_udp < 0)
	{
		return -1;
	}
	// open UDP port
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(localhost);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int rt = ::bind(fd_udp, (struct sockaddr*)&addr, sizeof(addr));
	if (rt != 0)
	{
		printf("\033[1;31m----> bind port %d failed.\033[0m\n", localhost);
		return -2;
	}

	return fd_udp;
}
int SystemAPI::closefd(int __fd, bool isSocket)
{
#ifdef _WIN32
	if (!isSocket)
		CloseHandle((HANDLE)__fd);
	else
		closesocket(__fd);
	return 0;
#elif __linux
	UNUSED(isSocket);
	shutdown(__fd, SHUT_RDWR);
	return close(__fd);
#endif
}
std::vector<std::string> SystemAPI::GetComPort()
{
#ifdef _WIN32
	HKEY hKey;
	std::vector<std::string> comName;
	char portName[256], commName[256];
	//打开串口注册表对应的键值
	if (ERROR_SUCCESS == ::RegOpenKeyEx(HKEY_LOCAL_MACHINE, "Hardware\\DeviceMap\\SerialComm", NULL, KEY_READ, &hKey))
	{
		int i = 0;
		int mm = 0;
		DWORD  dwLong, dwSize;
		while (TRUE)
		{
			dwLong = dwSize = sizeof(portName);
			//枚举串口
			if (ERROR_NO_MORE_ITEMS == ::RegEnumValue(hKey, i, portName, &dwLong, NULL, NULL, (PUCHAR)commName, &dwSize))
			{
				break;
			}
			comName.push_back(commName);
			i++;
		}
		//关闭注册表
		RegCloseKey(hKey);
	}
	//返回串口号
	return comName;
#elif __linux

	std::vector<std::string> comName;
	std::string path = "/dev";
	DIR* dirp;
	struct dirent* dp;

	dirp = opendir(path.c_str());
	if (dirp == NULL) {
		printf("opendir %s failed\n", path.c_str());
		return comName;
	}

	while ((dp = readdir(dirp)) != NULL) {
		std::string curpath(path);
		if (path.at(path.length()-1) != '/') {
			curpath += std::string("/") += dp->d_name;
		}
		else {
			curpath += dp->d_name;
		}
		//判断是否为文件以及文件后缀名
		if (dp->d_type == DT_CHR) {

			if (curpath.find("ttyUSB") != std::string::npos || curpath.find("ttyACM") != std::string::npos)
			{
				//std::cout<<curpath<<std::endl;
				comName.push_back(curpath);
			}

		}
	}
	closedir(dirp);
	return comName;
#endif
}

#ifdef _WIN32
int read(int __fd, void* __buf, int __nbytes)
{
	DWORD nr;
	bool ret = ReadFile((HANDLE)__fd, __buf, __nbytes, &nr, NULL);
	if (ret == false)
		return -1;
	return nr;
}
int write(int __fd, const void* __buf, int __n)
{
	DWORD nr = 0;
	WriteFile((HANDLE)__fd, __buf, __n, &nr, NULL);
	return nr;
}

int Open_serial_port(const char* name, int port)
{
	char path[32];
	sprintf_s(path, 30, "\\\\.\\%s", name);
	// Open the serial port.
	HANDLE hPort = CreateFile(path,
		GENERIC_READ | GENERIC_WRITE,		  // Access (read-write) mode
		FILE_SHARE_READ | FILE_SHARE_WRITE, // Share mode
		NULL,								  // Pointer to the security attribute
		OPEN_EXISTING,					  // How to open the serial port
		0,								  // Port attributes
		NULL);							  // Handle to port with attribute

	if (hPort == NULL || hPort == INVALID_HANDLE_VALUE)
	{
		// MessageBox(0, "can not open port", name, MB_OK);
		return 0;
	}
	DCB PortDCB;
	// Initialize the DCBlength member.
	PortDCB.DCBlength = sizeof(DCB);
	// Get the default port setting information.
	GetCommState(hPort, &PortDCB);

	// Change the DCB structure settings.
	PortDCB.BaudRate = port; // 115200;              // Current baud
	PortDCB.fBinary = TRUE;			  // Binary mode; no EOF check
	PortDCB.fParity = TRUE;			  // Enable parity checking
	PortDCB.fOutxCtsFlow = FALSE;	  // No CTS output flow control
	PortDCB.fOutxDsrFlow = FALSE;	  // No DSR output flow control
	PortDCB.fDtrControl = DTR_CONTROL_ENABLE;
	// DTR flow control type
	PortDCB.fDsrSensitivity = FALSE;  // DSR sensitivity
	PortDCB.fTXContinueOnXoff = TRUE; // XOFF continues Tx
	PortDCB.fOutX = FALSE;			  // No XON/XOFF out flow control
	PortDCB.fInX = FALSE;			  // No XON/XOFF in flow control
	PortDCB.fErrorChar = FALSE;		  // Disable error replacement
	PortDCB.fNull = FALSE;			  // Disable null stripping
	PortDCB.fRtsControl = RTS_CONTROL_ENABLE;
	// RTS flow control
	PortDCB.fAbortOnError = FALSE; // Do not abort reads/writes on
	// error
	PortDCB.ByteSize = 8;		   // Number of bits/byte, 4-8
	PortDCB.Parity = NOPARITY;	   // 0-4=no,odd,even,mark,space
	PortDCB.StopBits = ONESTOPBIT; // 0,1,2 = 1, 1.5, 2

	// Configure the port according to the specifications of the DCB
	// structure.
	if (!SetCommState(hPort, &PortDCB))
	{
		// MessageBox(0, "Unable to configure the serial port", "error", MB_OK);
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
	CommTimeouts.WriteTotalTimeoutMultiplier = 0; // 10;
	CommTimeouts.WriteTotalTimeoutConstant = 0;	  // 1000;

	// Set the timeout parameters for all read and write operations
	// on the port.
	if (!SetCommTimeouts(hPort, &CommTimeouts))
	{
		// Could not set the timeout parameters.
		// MessageBox(0, "Unable to set the timeout parameters", "error", MB_OK);
		CloseHandle(hPort);
		return NULL;
	}

	return (int)hPort;
}
#elif __linux
int Open_serial_port(const char* port, int baud_rate)
{
	int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0)
	{
		printf("\033[1;31m----> Open %s error\033[0m\n", port);
		return -1;
	}

	int ret;
	struct termios attrs;
	tcflush(fd, TCIOFLUSH);

	/* get current attrs */
	ret = tcgetattr(fd, &attrs);
	if (ret < 0)
	{
		printf("get attrs failed");
		return -2;
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
		printf("tcsetattr err");
		return -3;
	}

	if (change_baud(fd, baud_rate))
	{
		close(fd);
		printf("fail to set baudrate %d", baud_rate);
		return -4;
	}

	return fd;
}
#endif // _WIN32


void CommunicationAPI::send_cmd_vpc(int hCom, int mode, int sn, int len, const char* cmd)
{
	char buffer[2048] = { 0 };
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = mode;
	hdr->sn = sn;
	len = ((len + 3) >> 2) * 4;

	hdr->len = len;
	memcpy(buffer + sizeof(CmdHeader), cmd, len);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);

	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);
	//pcrc[0] =BaseAPI::stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	write(hCom, buffer, len2);
}

bool CommunicationAPI::uart_talk(int fd, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch)
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
				if (strcmp(cmd, "LXVERH") == 0 || strcmp(cmd, "LUUIDH") == 0 || strcmp(cmd, "LTYPEH") == 0)
				{
					memcpy(fetch, buf + i + nhdr, nfetch);
					fetch[nfetch] = 0;
				}
				else if(strstr(cmd, "LSRPM")!=NULL)
				{
					if(buf[i + nhdr+1]=='O'&&buf[i + nhdr+2]=='K')
					{
						strcpy(fetch, "OK");
						fetch[3] = 0;
					}
					else if(buf[i + nhdr+1]=='e'&&buf[i + nhdr+2]=='r')
					{
						strcpy(fetch, "NG");
						fetch[3] = 0;
					}
				}
				else
				{
					strcpy(fetch, "OK");
					fetch[3] = 0;
				}
			}
			return true;
		}
		else if (memcmp(buf + i, cmd, n) == 0)
		{
			if (nfetch > 0)
			{
				memcpy(fetch, buf + i + n + 1, 2);
				if(buf[ i + n + 1]=='E'&&buf[ i + n + 2]=='R') 
				{
					fetch[0]='N';
					fetch[1]='G';
				}
				fetch[2] = 0;
			}
			return true;
		}
		else if (memcmp(buf + i, "unsupport", 9) == 0)
		{
			if (nfetch > 0)
			{
				strcpy(fetch, "unsupport"); 
				fetch[10] = 0;
			}
			return true;
		}
	}

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return false;
}

bool CommunicationAPI::vpc_talk(int hcom, int mode, short sn, int len, const char* cmd, int nfetch, void* result)
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

	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);
	//pcrc[0] = BaseAPI::stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	int nr = write(hcom, buffer, len2);
	char buf[2048];
	int index = 10;
	//4C 48 BC FF   xx xx xx xx  result
	//读取之后的10*2048个长度，如果不存在即判定失败
	while (index--)
	{
		nr = read(hcom, buf, sizeof(buf));
		while (nr < (int)sizeof(buf))
		{
			int n = read(hcom, buf + nr, sizeof(buf) - nr);
			if (n > 0) nr += n;
		}

		for (int i = 0; i < (int)sizeof(buf) - nfetch; i++)
		{
			if (mode == C_PACK)
			{
				char* fetch = (char*)result;
				if (buf[i] == 0x4C && buf[i + 1] == 0x48 && buf[i + 2] == (signed char)0xBC && buf[i + 3] == (signed char)0xFF)
				{
					/*int packSN = ((unsigned int)buf[i + 5] << 8) | (unsigned int)buf[i + 4];
					if (packSN != sn)
						continue;*/

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
			else if (mode == S_PACK)
			{
				if ((buf[i + 2] == (signed char)0xAC && buf[i + 3] == (signed char)0xB8) || (buf[i + 2] == (signed char)0xAC && buf[i + 3] == (signed char)0xff))
				{
					//printf("%02x  %02x\n", buf[i + 2], buf[i + 3]);
					//随机码判定
					short packSN = ((unsigned char)buf[i + 5] << 8) | (unsigned char)buf[i + 4];
					if (packSN != sn)
						continue;

					memcpy(result, buf + i + 8, nfetch);
					return true;
				}
			}
		}
	}
	printf("read %d bytes, not found %s\n", nr, cmd);
	return false;
}

void  CommunicationAPI::send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf)
{
	char buffer[2048];
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd;
	hdr->sn = sn;

	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), snd_buf, len);

	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	sockaddr_in to;
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = inet_addr(dev_ip);
	to.sin_port = htons(dev_port);

	int len2 = len + sizeof(CmdHeader) + 4;

	sendto(fd_udp, buffer, len2, 0, (struct sockaddr*)&to, sizeof(struct sockaddr));


}
bool CommunicationAPI::udp_talk_GS_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result)
{
	unsigned short sn = rand();
	CommunicationAPI::send_cmd_udp(fd_udp, ip, port, 0x4753, sn, n, cmd);

	int nr = 0;
	for (int i = 0; i < 1000; i++)
	{
		fd_set fds;
		FD_ZERO(&fds);

		FD_SET(fd_udp, &fds);

		struct timeval to = { 1,0 };
		int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);

		if (ret <= 0)
		{
			return false;
		}

		// read UDP data
		if (FD_ISSET(fd_udp, &fds))
		{
			nr++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			char buf[1024] = { 0 };
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader* hdr = (CmdHeader*)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;

				memcpy(result, buf + 8, sizeof(EEpromV101));
				EEpromV101 t;
				memcpy(&t, result, sizeof(EEpromV101));
				return true;
			}
		}
	}

	printf("read %d packets, not response\n", nr);
	return false;
}
//配置信息设置
bool CommunicationAPI::udp_talk_S_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result)
{
	unsigned short sn = rand();
	CommunicationAPI::send_cmd_udp(fd_udp, ip, port, 0x0053, sn, n, cmd);

	int nr = 0;
	for (int i = 0; i < 1000; i++)
	{
		fd_set fds;
		FD_ZERO(&fds);

		FD_SET(fd_udp, &fds);

		struct timeval to = { 3, 0 };
		int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);
		if (ret <= 0)
		{
			return false;
		}
		// read UDP data
		if (FD_ISSET(fd_udp, &fds))
		{
			nr++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);
			char buf[1024] = { 0 };
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader* hdr = (CmdHeader*)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;
				memcpy(result, buf + 8, 2);
				return true;
			}
		}
	}

	printf("read %d packets, not response\n", nr);
	return false;
}

bool CommunicationAPI::udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port,
	int n, const char* cmd,
	int nhdr, const char* hdr_str,
	int nfetch, char* fetch)
{
	printf("send command : \'%s\' \n", cmd);

	unsigned short sn = rand();
	CommunicationAPI::send_cmd_udp(fd_udp, lidar_ip, lidar_port, 0x0043, sn, n, cmd);

	time_t t0 = time(NULL);
	int ntry = 0;
	while (time(NULL) < t0 + 3 && ntry < 1000)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd_udp, &fds);

		struct timeval to = { 1, 0 };
		int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);

		if (ret < 0)
		{
			printf("select error\n");
			return false;
		}
		if (ret == 0)
		{
			continue;
		}

		// read UDP data
		if (FD_ISSET(fd_udp, &fds))
		{
			ntry++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);
			char buf[1024] = { 0 };
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader* hdr = (CmdHeader*)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;

				char* payload = buf + sizeof(CmdHeader);
				for (int i = 0; i < nr - nhdr - 1; i++)
				{
					if (memcmp(payload + i, hdr_str, nhdr) == 0)
					{
						if (nfetch > 0)
						{
							memset(fetch, 0, nfetch);
							for (int j = 0; j < nfetch && i + nhdr + j < nr; j++)
								fetch[j] = payload[i + nhdr + j];
						}
						return true;
					}
				}
			}
		}
	}
	printf("read %d packets, not response\n", ntry);
	return false;
}


static FanSegment_C7* GetFanSegment(const RawDataHdr7& hdr, uint8_t* pdat, bool with_chk)
{
	UNUSED(with_chk);
	FanSegment_C7* fan_seg = new FanSegment_C7;
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
		printf("checksum error\n");
		delete fan_seg;
		return NULL;
	}

	return fan_seg;
}
static FanSegment_AA* GetFanSegment(const RawDataHdrAA& hdr, uint8_t* pdat, bool with_chk)
{
	UNUSED(with_chk);
	FanSegment_AA* fan_seg = new FanSegment_AA;
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
		for (int i = 1; i < (HDRAA_SIZE) / 2; i++)
			sum += pchk[i];
	}


	uint8_t* pDist = pdat + HDRAA_SIZE;
	uint8_t* pAngle = pdat + HDRAA_SIZE + 2 * hdr.N;
	uint8_t* energy = pdat + HDRAA_SIZE + 4 * hdr.N;

	for (int i = 0; i < hdr.N; i++, pDist += 2, pAngle += 2)
	{
		fan_seg->dist[i] = ((uint16_t)(pDist[1]) << 8) | pDist[0];
		fan_seg->angle[i] = ((uint16_t)(pAngle[1]) << 8) | pAngle[0];
		fan_seg->energy[i] = energy[i];

		sum += fan_seg->dist[i];
		sum += fan_seg->angle[i];
		sum += energy[i];
	}

	uint8_t* pchk = pdat + HDRAA_SIZE + 5 * hdr.N;
	uint16_t chksum = ((uint16_t)(pchk[1]) << 8) | pchk[0];
	if (chksum != sum) {
		printf("checksum error\n");
		delete fan_seg;
		return NULL;
	}

	return fan_seg;
}
static int GetFanPointCount(FanSegment_C7* seg)
{
	int n = 0;

	while (seg) { n += seg->hdr.N;  seg = seg->next; }

	return n;
}
static int GetFanPointCount(FanSegment_AA* seg)
{
	int n = 0;

	while (seg) { n += seg->hdr.N;  seg = seg->next; }

	return n;
}
static void PackFanData(FanSegment_C7* seg, RawData& rdat)
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
	dat->flags = seg->hdr.flags;

	DecTimestamp(seg->hdr.timestamp, dat->ts);

	int count = 0;
	while (seg)
	{
		double s = PI / 180000.0;
		for (int i = 0; i < seg->hdr.N; i++, count++)
		{
			dat->points[count].confidence = seg->energy[i];
			dat->points[count].distance = seg->dist[i] / 1000.0;
			dat->points[count].angle = (seg->angle[i] + seg->hdr.beg_ang) * s;
		}

		seg = seg->next;
	}
	//return dat;
}
static void PackFanData(FanSegment_AA* seg, RawData& rdat)
{
	RawData* dat = &rdat;

	dat->code = 0xfaaa;
	dat->N = seg->hdr.whole_fan;
	dat->angle = seg->hdr.beg_ang / 100; // 0.1 degree
	dat->span = (seg->hdr.end_ang - seg->hdr.beg_ang) / 100; // 0.1 degree
	dat->fbase = 0;
	dat->first = 0;
	dat->last = 0;
	dat->fend = 0;
	dat->flags = seg->hdr.flags;

	dat->ts[0] = seg->hdr.second;
	dat->ts[1] = seg->hdr.nano_sec / 1000;

	int count = 0;
	while (seg)
	{
		double s = PI / 180000.0;
		for (int i = 0; i < seg->hdr.N; i++, count++)
		{
			dat->points[count].confidence = seg->energy[i];
			dat->points[count].distance = seg->dist[i] / 1000.0;
			dat->points[count].angle = (seg->angle[i] + seg->hdr.beg_ang) * s;
		}

		seg = seg->next;
	}
	//return dat;
}
