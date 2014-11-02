#include "GeodesicFeatures.h"

#include <pcl/io/pcd_io.h>
#include <Kinect.h>


#include "vtkTimerLog.h"
#include "vtkPolyDataMapper.h"
#include "vtkTimerLog.h"
#include "vtkCamera.h"
#include "vtkProperty.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkPolyDataNormals.h"
#include "vtkPolyDataCollection.h"
#include "vtkObjectFactory.h"
#include "vtkIdList.h"
#include "vtkXMLPolyDataReader.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkNew.h"
#include "vtkPointData.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/point_types.h>
//vtk stuff
#include "pcl/io/vtk_io.h"
#include "pcl/io/vtk_lib_io.h"
#include "pcl/ros/conversions.h"

#include <pcl/surface/processing.h>
#include "pcl/surface/vtk_smoothing/vtk.h"
#include "pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h"
#include "pcl/surface/vtk_smoothing/vtk_utils.h"
#include "vtkSmoothPolyDataFilter.h"
#include "vtkDoubleArray.h"


GeodesicFeatures::GeodesicFeatures()
{
}


GeodesicFeatures::~GeodesicFeatures()
{
}

void GeodesicFeatures::processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	vtkNew<vtkTimerLog> timer;

	// Organized Fast Mesh Algorithm
	pcl::OrganizedFastMesh<pcl::PointXYZRGB> ofm;
	ofm.setTrianglePixelSize(1);
	ofm.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);

	// Triangulate using FastMesh
	ofm.setInputCloud(cloud);

	//reconstruct mesh
	pcl::PolygonMesh triangles;

	cout << endl << "Computing Mesh ..." << endl;
	timer->StartTimer();

	ofm.reconstruct(triangles);

	timer->StopTimer();
	cout << "Mesh took " << timer->GetElapsedTime() << " s." << endl;

	/** VTK **/
	cout << endl << "Converting to VTK ..." << endl;
	timer->StartTimer();

	pcl::VTKUtils::convertToVTK(triangles, this->vtk_polygons);

	

	timer->StopTimer();
	cout << "Conversion took " << timer->GetElapsedTime() << " s." << endl;
}

float GeodesicFeatures::compute(const Location &ji, const Location &je) const
{
	float geodist = 0;

	vtkIdType iniId = this->vtk_polygons->FindPoint(ji.x, ji.y, ji.z);
	vtkIdType endId = this->vtk_polygons->FindPoint(je.x, je.y, je.z);

	//cout << "   i(" << ji.x << ji.y << ji.z << ")  ->  IDi(" << iniId << ")   ";
	//cout << "   f(" << je.x << je.y << je.z << ")  ->  IDf(" << endId << ")   ";

	vtkSmartPointer<vtkDijkstraGraphGeodesicPath> dijkstra =
		vtkSmartPointer<vtkDijkstraGraphGeodesicPath>::New();
	dijkstra->SetInputData(this->vtk_polygons );
	dijkstra->SetStartVertex(iniId);
	dijkstra->SetEndVertex(endId);

	try{
		//cout << "Before dijkstra update" << endl;
		dijkstra->Update();
		//cout << "After dijkstra update" << endl;

		vtkNew <vtkDoubleArray> weights;
		dijkstra->GetCumulativeWeights(weights.GetPointer());
		geodist = weights->GetValue(endId);

	}catch (const std::exception& e) 
	{
		std::cout << "exception caught: " << e.what() << '\n';
	}

	return geodist;
}

void GeodesicFeatures::extract(std::vector<JointLoc> const &body) const
{
	float geo[5];
	unsigned short idx=0;

	cout << endl << "--- GEODESIC DISTANCES:";

	// Left to right shoulder
	if (body.at(JointType::JointType_ShoulderLeft).tracked && body.at(JointType::JointType_ShoulderRight).tracked)
	{
		geo[0] = this->compute(
			body.at(JointType::JointType_ShoulderLeft).Loc3D,
			body.at(JointType::JointType_ShoulderRight).Loc3D);

		cout << endl << "	1.Left -> Right shoulder:   " << geo[0];
	}
	else
	{
		cout << endl << "	1.Left -> Right shoulder (NOT TRACKED CORRECTLY)   ";
	}


	// Left to right hip
	if (body.at(JointType::JointType_HipLeft).tracked && body.at(JointType::JointType_HipRight).tracked)
	{
		geo[1] = this->compute(
			body.at(JointType::JointType_HipLeft).Loc3D,
			body.at(JointType::JointType_HipRight).Loc3D);

		cout << endl << "	2.Left -> Right hip:   " << geo[1];
	}
	else
	{
		cout << endl << "		2.Left -> Right hip (NOT TRACKED CORRECTLY)   ";
	}

	// Middle torso -> Left shoulder:
	if (body.at(JointType::JointType_SpineMid).tracked && body.at(JointType::JointType_ShoulderLeft).tracked)
	{
		geo[2] = this->compute(
			body.at(JointType::JointType_SpineMid).Loc3D,
			body.at(JointType::JointType_ShoulderLeft).Loc3D);

		cout << endl << "	3.Middle torso -> Left shoulder:   " << geo[2];
	}
	else
	{
		cout << endl << "		3.Middle torso -> Left shoulder (NOT TRACKED CORRECTLY)   ";
	}

	// Middle torso -> Left hip
	if (body.at(JointType::JointType_SpineMid).tracked && body.at(JointType::JointType_HipLeft).tracked)
	{
		geo[3] = this->compute(
			body.at(JointType::JointType_SpineMid).Loc3D,
			body.at(JointType::JointType_HipLeft).Loc3D);

		cout << endl << "	4.Middle torso -> Left hip:   " << geo[3];
	}
	else
	{
		cout << endl << "		4.Middle torso -> Left hip (NOT TRACKED CORRECTLY)   ";
	}

	// Middle torso -> Right hip
	if (body.at(JointType::JointType_SpineMid).tracked && body.at(JointType::JointType_HipRight).tracked)
	{
		geo[4] = this->compute(
			body.at(JointType::JointType_SpineMid).Loc3D,
			body.at(JointType::JointType_HipRight).Loc3D);

		cout << endl << "	4.Middle torso -> Right hip:   " << geo[4];
	}
	else
	{
		cout << endl << "		5.Middle torso -> Right hip (NOT TRACKED CORRECTLY)   ";
	}
}
