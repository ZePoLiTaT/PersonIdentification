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

#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h>


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

	//cout << endl << "Computing Mesh ..." << endl;
	timer->StartTimer();

	ofm.reconstruct(triangles);

	timer->StopTimer();
	cout << "Mesh took " << timer->GetElapsedTime() << " s." << endl;

	/** VTK **/
	//cout << endl << "Converting to VTK ..." << endl;
	timer->StartTimer();

	pcl::VTKUtils::convertToVTK(triangles, this->vtk_polygons);

	

	timer->StopTimer();
	//cout << "Conversion took " << timer->GetElapsedTime() << " s." << endl;
}

float GeodesicFeatures::compute(const Location &ji, const Location &je, unsigned short geoFeatureId) const
{
	float geodist = 0;

	vtkIdType iniId = this->vtk_polygons->FindPoint(ji.x, ji.y, ji.z);
	vtkIdType endId = this->vtk_polygons->FindPoint(je.x, je.y, je.z);

	//cout << "   i(" << ji.x << ji.y << ji.z << ")  ->  IDi(" << iniId << ")   ";
	//cout << "   f(" << je.x << je.y << je.z << ")  ->  IDf(" << endId << ")   ";



	vtkSmartPointer<DijkstraGraphGeodesicPath> dijkstra =
		vtkSmartPointer<DijkstraGraphGeodesicPath>::New();
	dijkstra->SetInputData(this->vtk_polygons );
	dijkstra->SetStartVertex(iniId);
	dijkstra->SetEndVertex(endId);

	
	
	//cout << "Before dijkstra update" << endl;
	dijkstra->Update();

	//cout << "After dijkstra update" << endl;
	if (dijkstra->checkPathSuccess())
	{
		vtkNew <vtkDoubleArray> weights;
		dijkstra->GetCumulativeWeights(weights.GetPointer());
		geodist = weights->GetValue(endId);

		if (!GeodesicFeatures::showRender[geoFeatureId])
		{
			GeodesicFeatures::showRender[geoFeatureId] = true;
			//plot(ji, je, iniId, endId, dijkstra);
		}
		
			
	}
		
	else
	{
		return -1;
	}



	return geodist;
}

vector<float> GeodesicFeatures::extract(std::vector<JointLoc> const &body) const
{
	float geodist;
	unsigned short idx=0;
	vector <float> geo;

	//cout << endl << "--- GEODESIC DISTANCES:";

	// Left to right shoulder
	if (body.at(JointType::JointType_ShoulderLeft).tracked && body.at(JointType::JointType_ShoulderRight).tracked)
	{
		geodist = this->compute(
			body.at(JointType::JointType_ShoulderLeft).Loc3D,
			body.at(JointType::JointType_ShoulderRight).Loc3D,
			0);

		geo.push_back(geodist);
		//cout << endl << "	1.Left -> Right shoulder:   " << geodist;
	}
	else
	{
		geo.push_back(-1);
		//cout << endl << "	1.Left -> Right shoulder (NOT TRACKED CORRECTLY)   ";
	}


	// Left to right hip
	if (body.at(JointType::JointType_HipLeft).tracked && body.at(JointType::JointType_HipRight).tracked)
	{
		geodist = this->compute(
			body.at(JointType::JointType_HipLeft).Loc3D,
			body.at(JointType::JointType_HipRight).Loc3D,
			1);

		geo.push_back(geodist);
		//cout << endl << "	2.Left -> Right hip:   " << geodist;
	}
	else
	{
		geo.push_back(-1);
		//cout << endl << "		2.Left -> Right hip (NOT TRACKED CORRECTLY)   ";
	}

	Location spinemid_loc = body.at(JointType::JointType_SpineMid).Loc3D;
	spinemid_loc.z = spinemid_loc.z - 0.2;

	// Middle torso -> Left shoulder:
	if (body.at(JointType::JointType_SpineMid).tracked && body.at(JointType::JointType_ShoulderLeft).tracked)
	{
		geodist = this->compute(
			spinemid_loc,
			//body.at(JointType::JointType_SpineMid).Loc3D,
			body.at(JointType::JointType_ShoulderLeft).Loc3D,
			2);

		geo.push_back(geodist);
		//cout << endl << "	3.Middle torso -> Left shoulder:   " << geodist;
	}
	else
	{
		geo.push_back(-1);
		//cout << endl << "		3.Middle torso -> Left shoulder (NOT TRACKED CORRECTLY)   ";
	}

	// Middle torso -> Left hip
	if (body.at(JointType::JointType_SpineMid).tracked && body.at(JointType::JointType_HipLeft).tracked)
	{
		geodist = this->compute(
			spinemid_loc,
			//body.at(JointType::JointType_SpineMid).Loc3D,
			body.at(JointType::JointType_HipLeft).Loc3D,
			3);

		geo.push_back(geodist);
		//cout << endl << "	4.Middle torso -> Left hip:   " << geodist;
	}
	else
	{
		geo.push_back(-1);
		//cout << endl << "		4.Middle torso -> Left hip (NOT TRACKED CORRECTLY)   ";
	}

	// Middle torso -> Right hip
	if (body.at(JointType::JointType_SpineMid).tracked && body.at(JointType::JointType_HipRight).tracked)
	{
		geodist = this->compute(
			spinemid_loc,
			//body.at(JointType::JointType_SpineMid).Loc3D,
			body.at(JointType::JointType_HipRight).Loc3D,
			4);

		geo.push_back(geodist);
		//cout << endl << "	4.Middle torso -> Right hip:   " << geodist;
	}
	else
	{
		geo.push_back(-1);
		//cout << endl << "		5.Middle torso -> Right hip (NOT TRACKED CORRECTLY)   ";
	}

	return geo;
}

void GeodesicFeatures::plot(const Location &ji, const Location &je, vtkIdType iniId, vtkIdType endId, vtkSmartPointer<DijkstraGraphGeodesicPath> dijkstra) const
{
	//geodist = 0;

	// Create the geometry of a point (the coordinate)
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();
	const float p[3] = { ji.x, ji.y, ji.z };
	const float p2[3] = { je.x, je.y, je.z };

	// Create the topology of the point (a vertex)
	vtkSmartPointer<vtkCellArray> vertices =
		vtkSmartPointer<vtkCellArray>::New();
	vtkIdType pid[2];
	pid[0] = points->InsertNextPoint(p);
	pid[1] = points->InsertNextPoint(p2);
	vertices->InsertNextCell(2, pid);

	// Create a polydata object
	vtkSmartPointer<vtkPolyData> point =
		vtkSmartPointer<vtkPolyData>::New();

	// Set the points and vertices we created as the geometry and topology of the polydata
	point->SetPoints(points);
	point->SetVerts(vertices);

	// Visualize
	vtkSmartPointer<vtkPolyDataMapper> mapper_punto =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper_punto->SetInputData(point);

	vtkSmartPointer<vtkActor> actor_punto =
		vtkSmartPointer<vtkActor>::New();
	actor_punto->SetMapper(mapper_punto);
	actor_punto->GetProperty()->SetPointSize(5);
	actor_punto->GetProperty()->SetColor(0, 1, 0); // Red



	////////////////////////////////////////////////////////////////////////////////

	double vp[3];
	double vp2[3];
	vtk_polygons->GetPoint(iniId, vp);
	vtk_polygons->GetPoint(endId, vp2);
	//cout << "punto inicial: " << vp[0] << " " << vp[1] << " " << vp[2] << endl;
	//cout << "punto final: " << vp2[0] << " " << vp2[1] << " " << vp2[2] << endl;

	//Create the geometry of a point (the coordinate)
	vtkSmartPointer<vtkPoints> vtkpoints =
		vtkSmartPointer<vtkPoints>::New();
	//const float vp[3] = { ji.x, ji.y, ji.z };
	//const float vp2[3] = { je.x, je.y, je.z };

	// Create the topology of the point (a vertex)
	vtkSmartPointer<vtkCellArray> vtkvertices =
		vtkSmartPointer<vtkCellArray>::New();
	vtkIdType vtkpid[2];
	vtkpid[0] = vtkpoints->InsertNextPoint(vp);
	vtkpid[1] = vtkpoints->InsertNextPoint(vp2);
	vtkvertices->InsertNextCell(2, vtkpid);

	// Create a polydata object
	vtkSmartPointer<vtkPolyData> vtkpoint =
		vtkSmartPointer<vtkPolyData>::New();

	// Set the points and vertices we created as the geometry and topology of the polydata
	vtkpoint->SetPoints(vtkpoints);
	vtkpoint->SetVerts(vtkvertices);

	// Visualize
	vtkSmartPointer<vtkPolyDataMapper> mapper_puntovtk =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper_puntovtk->SetInputData(vtkpoint);

	vtkSmartPointer<vtkActor> actor_puntovtk =
		vtkSmartPointer<vtkActor>::New();
	actor_puntovtk->SetMapper(mapper_puntovtk);
	actor_puntovtk->GetProperty()->SetPointSize(10);
	actor_puntovtk->GetProperty()->SetColor(1, 0.5, 0); // blue

	////////////////////////////////////////////////////////////////////////////////


	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> pathMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	pathMapper->SetInputConnection(dijkstra->GetOutputPort());

	vtkSmartPointer<vtkActor> pathActor =
		vtkSmartPointer<vtkActor>::New();
	pathActor->SetMapper(pathMapper);
	pathActor->GetProperty()->SetColor(1, 0, 0); // Red
	pathActor->GetProperty()->SetLineWidth(4);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(vtk_polygons);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	// Create a renderer, render window, and interactor
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	// Add the actor to the scene
	renderer->AddActor(actor);
	renderer->AddActor(pathActor);
	renderer->AddActor(actor_punto);
	// Add the actor to the scene
	renderer->AddActor(actor_puntovtk);
	renderer->SetBackground(.3, .6, .3); // Background color green

	// Render and interact
	renderWindow->Render();
	renderWindowInteractor->Start();
}

bool GeodesicFeatures::showRender[5] = { false, false, false, false, false };

