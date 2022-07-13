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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "data.h"
#include <string>
#include <signal.h>
using namespace std;

// from_zero true时从[0，pi），false时[-pi,pi)
bool from_zero = false;
int factor = from_zero ? 2 : 1;
#ifndef FILL_FAN_ZERO
vector<RawData*> datas;


// 获取360°完整扇区的点
void data_process(const RawData &raw, const char* output_file, PointData& tmp)
{
	RawData *data = new RawData;
	memcpy(data, &raw, sizeof(RawData));
	datas.push_back(data);

	if (raw.angle + raw.span != factor * 1800)
	{
		return;
	}

	bool bfirst = true;
	uint32_t timestamp[2] = {0};
	int count = 0, n = 0, angles = 0;
	for (vector<RawData *>::iterator it = datas.begin(); it != datas.end(); ++it)
	{
		data = *it;
		angles += data->span;
		count += data->N;
		if (bfirst)
		{
			timestamp[0] = data->ts[0];
			timestamp[1] = data->ts[1];
			bfirst = false;
		}
		n++;
	}

	if (angles != 3600)
	{
		DEBUG_PR("angle sum %d, drop %d fans %d points\n", angles, n, count);
		for (vector<RawData *>::iterator it = datas.begin(); it != datas.end(); ++it)
		{
			data = *it;
			delete data;
		}
		datas.clear();
	}

	DataPoint *points = new DataPoint[count];
	count = 0;

	for (vector<RawData *>::iterator it = datas.begin(); it != datas.end(); ++it)
	{
		data = *it;
		for (int i = 0; i < data->N; i++)
		{
			points[count++] = data->points[i];
		}
		delete data;
	}
	datas.clear();

	tmp.N = count;
	memcpy(tmp.ts, timestamp, sizeof(timestamp));
	memcpy(tmp.points, points, sizeof(DataPoint)* count);
	//printf("%d\n", tmp.N);
	delete[] points;
}

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

vector<RawData *> whole_datas;
void whole_data_process(const RawData &raw, bool from_zero, const char *output_file, PointData& tmp)
{
	RawData *data = new RawData;
	memcpy(data, &raw, sizeof(RawData));
	whole_datas.push_back(data);

	int factor = from_zero ? 2 : 1;
	if (raw.angle + raw.span != factor * 1800)
	{
		return;
	}
	bool bfirst = true;
	uint32_t timestamp[2] = {0};
	int count = 0, n = 0, angles = 0;

	for (std::vector<RawData *>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
	{
		data = *it;
		// accumulate point's angle and counts
		angles += data->span;
		count += data->N;
		// record first point timestamp
		if (bfirst)
		{
			timestamp[0] = data->ts[0];
			timestamp[1] = data->ts[1];
			bfirst = false;
		}
		// record fan counts
		n++;
	}

	// not whole circle data, so clear it
	if (angles != 3600)
	{
		DEBUG_PR("angle sum %d, drop %d fans %d points\n", angles, n, count);
		for (std::vector<RawData *>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
		{
			data = *it;
			delete data;
		}
		whole_datas.clear();
	}

	DataPoint *points = new DataPoint[count]; // store whole circle points
	count = 0;
	for (std::vector<RawData *>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
	{
		data = *it;
		for (int i = 0; i < data->N; i++)
		{
			points[count++] = data->points[i];
		}
		delete data;
	}
	whole_datas.clear();
	//INFO_PR("single span data points %d  time:%d.%d\n", count, timestamp[0], timestamp[1]);

	//如果打开了打印开关，则返回雷达信息
	
	//PointData tmp;
	tmp.N = count;
	memcpy(tmp.ts, timestamp, sizeof(uint32_t) * 2);
	memcpy(tmp.points, points, sizeof(DataPoint) * count);
	//执行回调函数
	//((void(*)(int, void *))msgptr)(1, &tmp);
	// printf("\r%d.%d : Data frame head = %x, 360 degrees contains %d spans", timestamp[0], timestamp[1], pack_format, n);
	if (output_file != NULL)
	{
		FILE *fp = fopen(output_file, "w");
		if (fp)
		{
			for (int i = 0; i < count; i++)
			{
				fprintf(fp, "%.5f\t%.3f\t%d\n",
						points[i].angle > factor * PI ? points[i].angle - 2 * PI : points[i].angle,
						points[i].distance, points[i].confidence);
			}
			fclose(fp);
		}
	}
	delete[] points;
}

#else

#define MAX_FANS 40
RawData *datas[MAX_FANS] = {NULL};

double calc_res()
{
	double sum = 0;
	int cnt = 0;
	for (int i = 0; i < MAX_FANS; i++)
	{
		if (datas[i] != NULL)
		{
			sum += datas[i]->span;
			cnt += datas[i]->N;
		}
	}
	return sum / cnt;
}

int fill_zero(int from, int to, double res, DataPoint *points)
{
	printf("fill hole %d ~ %d : %f\n", from, to, res);
	int N = (to - from) / res;
	for (int j = 0; j < N; j++)
	{
		points[j].angle = (from + j * res) * PI / 1800;
		points[j].distance = 0;
		points[j].confidence = 0;
	}
	return N;
}

int data_publish(int from, int to, double res, DataPoint *points)
{
	int M = from / 90, N = to / 90, count = 0;

	for (int i = M; i < N; i++)
	{
		RawData *data = datas[i];
		if (data == NULL)
			continue;

		if (from < data->angle)
		{
			count += fill_zero(from, data->angle, res, points + count);
		}

		for (int j = 0; j < data->N; j++)
		{
			points[count++] = data->points[j];
		}
		from = data->angle + data->span;
	}

	if (from < to)
	{
		count += fill_zero(from, to, res, points + count);
	}

	return count;
}

void data_publish(DataPoint *points)
{
	double res = calc_res();

	int count = data_publish(1800, 3600, res, points);

	count += data_publish(0, 1800, res, points + count);

	for (int i = 0; i < MAX_FANS; i++)
	{
		delete datas[i];
		datas[i] = NULL;
	}

	data_process(count, points);
}

DataPoint g_points[10000];
// 每次获得一个扇区（9°/ 36°)数据
void data_process(const RawData &raw)
{
	// if ((rand() % 100) == 50) {
	// printf("角度 %d, 数据点数 %d + %d\n", raw.angle/10, raw.N, raw.span);
	// return ;
	// }

	RawData *data = new RawData;
	memcpy(data, &raw, sizeof(RawData));

	int idx = raw.angle / 90;

	if (datas[idx] != NULL)
	{
		if (data->angle >= 1800)
		{
			data_publish(g_points);
		}
		else
		{
			delete datas[idx];
			datas[idx] = NULL;
		}
	}

	datas[idx] = data;
	if (raw.angle + raw.span == 1800)
	{
		data_publish(g_points);
	}
}

int getProductSN(char *data)
{
	if (strlen(g_uuid) > 0)
	{
		strcpy(data, g_uuid);
		return 0;
	}

	return -1;
}

#endif
