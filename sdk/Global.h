#pragma once
#include"data.h"
#include <set>
#include <vector>
#include<math.h>


namespace BaseAPI {
    unsigned int stm32crc(unsigned int* ptr, unsigned int len);
}

namespace ParseAPI {
    int parse_data_x(int len, unsigned char* buf,
        int& span, int& is_mm, int& with_conf,
        RawData& dat, int& consume, int with_chk, LidarMsgHdr& zone, FanSegment** fan_segs);
    bool parse_data(int len, unsigned char* buf,
        int& span, int& is_mm, int& with_conf,
        RawData& dat, int& consume, int with_chk);
}

namespace UserAPI {
    void fan_data_process(const RawData& raw, const char* output_file, PointData& tmp);
    void whole_data_process(const RawData& raw, bool from_zero, int collect_angle, const char* output_file, PointData& data, std::vector<RawData*>& last_whole_datas);
}

namespace AlgorithmAPI_E100 {

    int ShadowsFilter(PointData*, const ShadowsFilterParam&);
    int MedianFilter(PointData*, const MedianFilterParam&);


}

#ifdef _WIN32
    void gettimeofday(timeval* tv, void*);
#endif