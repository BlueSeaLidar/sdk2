#include "data.h"
#include <set>

#define M_PI 3.14159265358979323846

inline double getAngleWithViewpoint(float r1, float r2, float included_angle)
{
	return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
}

int ShadowsFilter(PointData* scan_in, const ShadowsFilterParam& param)
{
	double angle_increment = M_PI * 2 / scan_in->N;

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

			double angle = abs(rad * 180 / M_PI);
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
		scan_in->points[*it].distance = std::numeric_limits<float>::quiet_NaN();
		nr++;
	}

	return nr;
}

int int_cmper(const void* p1, const void* p2)
{
	int* d1 = (int*)p1;
	int* d2 = (int*)p2;

	return *d1 - *d2;
}

int MedianFilter(PointData* scan_in, const MedianFilterParam& param)
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
