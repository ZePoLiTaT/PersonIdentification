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
#include"DijkstraGeodesicPath.h"

#include <iostream>
#include <vector>

using namespace std;

class GeodesicFeatures
{
public:
	GeodesicFeatures();
	~GeodesicFeatures();

	vector<float> extract(std::vector<JointLoc> const &body) const;

	void processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	float compute(const Location &ji, const Location &je, unsigned short geoFeatureId) const;

	void plot(const Location &ji, const Location &je, vtkIdType iniId, vtkIdType endId, vtkSmartPointer<DijkstraGraphGeodesicPath> dijkstra) const;
	

private:
	vtkSmartPointer<vtkPolyData> vtk_polygons;
	static bool showRender[5];
};

