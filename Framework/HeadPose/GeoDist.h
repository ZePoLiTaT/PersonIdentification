#pragma once

#include "vtkDijkstraGraphGeodesicPath.h"
#include "vtkFastMarchingGeodesicDistance.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Kinect.h>
#include <vtkRenderWindow.h>

class GeoDist
{
public:
	GeoDist();
	~GeoDist();
	
	void compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const CameraSpacePoint &ji, const CameraSpacePoint &je) const;
};

