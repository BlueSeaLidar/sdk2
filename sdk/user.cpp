/**

 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved

 * File name:      user.cpp

 * Author:  	   *
     
 * Version:        1.0  
   
 * Date:		   2022.3.25

 * Description:   
				CN:打印雷达回传信息的分析数据，主要提供用户集成所需的雷达数据
				EN：Print analysis data of radar return information, mainly to provide radar data required for user integration

 */

#include <vector>
#include "data.h"
#include <string>

using namespace std;
vector<RawData*> whole_datas;

void fan_data_process(const RawData &raw, const char *output_file, PointData &tmp)
{
	RawData *data = new RawData;
	memcpy(data, &raw, sizeof(RawData));
	uint32_t timestamp[2] = {0};
	timestamp[0] = data->ts[0];
	timestamp[1] = data->ts[1];


	//PointData tmp;
	tmp.N = data->N;
	memcpy(tmp.ts, data->ts, sizeof(uint32_t) * 2);
	memcpy(tmp.points, data->points, sizeof(DataPoint)* data->N);
	//执行回调函数
	//((void(*)(int, void *))msgptr)(1, &tmp);

	if (output_file != NULL)
	{
		FILE *fp = fopen(output_file, "w");
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

void whole_data_process(const RawData &raw, bool from_zero, int collect_angle, const char *output_file, PointData& tmp)
{
	RawData *data = new RawData;
	memcpy(data, &raw, sizeof(RawData));
	whole_datas.push_back(data);
	if ((from_zero && (raw.angle + raw.span) % 3600 == 0)|| (!from_zero && raw.angle + raw.span == 1800+collect_angle * 10))
	{
	}
	else
	{
		return;
	}
	bool bfirst = true;
	uint32_t timestamp[2] = {0};
	int count = 0, n = 0, angles = 0;

	for (std::vector<RawData *>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
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
	//printf("3N:%d\n", whole_datas.size());
	// not whole circle data, so clear it
	if (angles != 3600)
	{
		DEBUG_PR("angle sum %d, drop %d fans %d points\n", angles, n, count);
		for (std::vector<RawData *>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
		{
			RawData* tmp = *it;
			delete tmp;
		}
		whole_datas.clear();
	}

	DataPoint *points = new DataPoint[count]; // store whole circle points
	count = 0;
	for (std::vector<RawData *>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
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
		FILE *fp = fopen(output_file, "w");
		if (fp)
		{
			for (int i = 0; i < count; i++)
			{
				fprintf(fp, "%.5f\t%.3f\t%d\n",
						points[i].angle,
						points[i].distance, points[i].confidence);
			}
			fclose(fp);
		}
	}
	delete[] points;
}

