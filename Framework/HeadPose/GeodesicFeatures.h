#pragma once

#include "vtkDijkstraGraphGeodesicPath.h"
//#include "vtkFastMarchingGeodesicDistance.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Kinect.h>
#include "vtkRenderWindow.h"
#include "vtkSmartPointer.h"
#include "vtkPolyData.h"
#include "Global_def.h"
#include <vector>

class GeodesicFeatures
{
public:
	GeodesicFeatures();
	~GeodesicFeatures();

	void processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	float compute(const Location &ji, const Location &je) const;

	void extract(std::vector<JointLoc> const &body) const;

private:
	vtkSmartPointer<vtkPolyData> vtk_polygons;
};

