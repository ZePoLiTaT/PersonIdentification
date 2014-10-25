#include "GeoDist.h"

#include <pcl/io/pcd_io.h>
#include <Kinect.h>

#include "vtkSmartPointer.h"
#include "vtkTimerLog.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkTimerLog.h"
#include "vtkCamera.h"
#include "vtkProperty.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkPolyDataNormals.h"
#include "vtkRendererCollection.h"
#include "vtkPolyDataCollection.h"
#include "vtkObjectFactory.h"
#include "vtkIdList.h"
#include "vtkXMLPolyDataReader.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkNew.h"
#include "vtkPointData.h"
#include "vtkContourWidget.h"
#include "vtkOrientedGlyphContourRepresentation.h"
#include "vtkPolygonalSurfacePointPlacer.h"
#include "vtkPolygonalSurfaceContourLineInterpolator2.h"

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


GeoDist::GeoDist()
{
}


GeoDist::~GeoDist()
{
}



void GeoDist::compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const CameraSpacePoint &ji, const CameraSpacePoint &je) const
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
	vtkSmartPointer<vtkPolyData> vtk_polygons;
	cout << endl << "Converting to VTK ..." << endl;
	timer->StartTimer();
	
	pcl::VTKUtils::convertToVTK(triangles, vtk_polygons);
	
	vtkIdType iniId = vtk_polygons->FindPoint(ji.X, ji.Y, ji.Z);
	vtkIdType endId = vtk_polygons->FindPoint(je.X, je.Y, je.Z);

	timer->StopTimer();
	cout << "Conversion took " << timer->GetElapsedTime() << " s." << endl;

	vtkNew<vtkPolyDataNormals> normals;

	const int geodesicMethod = 0;
	const int interpolationOrder = 0;
	const double distanceOffset = 0;

	// We need to ensure that the dataset has normals if a distance offset was
	// specified.
	if (fabs(distanceOffset) > 1e-6)
	{
		//normals->SetInputConnection(reader->GetOutputPort());
		normals->SetInputData(vtk_polygons);
		normals->SplittingOff();

		// vtkPolygonalSurfacePointPlacer needs cell normals
		// vtkPolygonalSurfaceContourLineInterpolator needs vertex normals
		normals->ComputeCellNormalsOn();
		normals->ComputePointNormalsOn();
		normals->Update();
	}

	vtkSmartPointer<vtkDijkstraGraphGeodesicPath> dijkstra =
		vtkSmartPointer<vtkDijkstraGraphGeodesicPath>::New();
	dijkstra->SetInputData(vtk_polygons);
	dijkstra->SetStartVertex(iniId);
	dijkstra->SetEndVertex(endId);

	try{
		cout << "Before dijkstra update" << endl;
		dijkstra->Update();

		cout << "Before get weights" << endl;
		vtkNew <vtkDoubleArray> weights;
		dijkstra->GetCumulativeWeights(weights.GetPointer());
		//cout << "* * *Geodesic distance value: " << endl;
		cout << "number of weights: " << weights->GetSize() << endl;
		cout << "endVertId: " << endId << endl;
		cout << "distance = " << weights->GetValue(endId) << endl;
	}catch (const std::exception& e) 
	{
		std::cout << "exception caught: " << e.what() << '\n';
	}
	

	

	//return NULL; // dijkstra;
}