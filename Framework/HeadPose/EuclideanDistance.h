#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Kinect.h>
#include "Global_def.h"

class EuclideanDistance
{
public:
	EuclideanDistance();
	~EuclideanDistance();

	//float computeSegments(const vector<Location>) const;
	float compute(const Location &ji, const Location &je) const;
};

