#ifndef RenderWindowUISingleInheritance_H
#define RenderWindowUISingleInheritance_H
 
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <QMainWindow>
#include<QString>
#include<QDir>
#include "IMU.h"
#include <Kinect_Thread.h>
#include<PipelineGovernor.h>
#include <concurrent_queue.h>
#define USER_MATRIX
#include <vtkArrowSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkMath.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <time.h>
// Forward Qt class declarations
class Ui_MainWindow;
 
class RenderWindowUISingleInheritance : public QMainWindow
{
	Q_OBJECT
public:
 
	// Constructor/Destructor
	RenderWindowUISingleInheritance(); 
	~RenderWindowUISingleInheritance();
signals:
	void cont(void);
	void colourDisplayParameterChange(int);
	void hpDisplayParameterChange(int);
public slots:
	virtual void dataReceived(QuaternionValue data,EulerAnglesStruct angles);
	virtual void frameReceived();
	virtual void slotExit();
	virtual void record();
	virtual void pause();
	virtual void new_person();
	virtual void colOPTChange(int);
	virtual void hpOPTChange(int);
	virtual void broadcastOPTChange(int);
private:
	ofstream fout;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; //(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //(new pcl::PointCloud<pcl::PointXYZ>);
  // Designer form
	IMU_THREAD *imuThread;
	Ui_MainWindow *ui;
	PipelineUtilities::PipelineGovernor *govt;
	concurrency::unbounded_buffer<shared_ptr<Kinect_Data>> *frames;
	concurrency::overwrite_buffer<tuple<QuaternionValue,EulerAnglesStruct>> *sensor_Data;
	//shared_ptrKinect_Data *dat;
	Kinect_Thread *kinectThread;
	QDir new_dir;
	QDir currentDir;
	int personNumber;
	long frameNumber;
	QString dirName;
	QString current_Dir;
	QString pcd_filename;
	QString data_filename;
	bool record_data;
	pcl::PCDWriter writer;
	std::ofstream fileWriter;
	std::ofstream listFileWriter;
	Colour_Display_Properties m_Colour_Properties;
	HeadPose_Display_Properties m_HP_Properties;
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter;
	vtkImageData* vtkRGBimage;
	//long frameNumber;
	bool broadcast;
	int geodist;
};
 
#endif
