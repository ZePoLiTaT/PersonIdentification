#ifndef KINECT_THREAD
#define KINECT_THREAD
#define NOMINMAX
#pragma region HEADERS
#include <qobject.h>
#include <qthread.h>
#include <Kinect.h>
#include <iostream>
#include <QMutex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <vector>
#ifndef Q_MOC_RUN
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/search/organized.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "MultilayerPerceptron.h"
#include "cJSON.h"
#include "IDX.hpp"
#include <cmath>
#endif
#include <ppl.h>
#include <concurrent_queue.h>
#include <agents.h>

#include <Global_def.h>
#include <iostream>
#include <PipelineGovernor.h>
#pragma endregion
using namespace std;
using namespace cv;
using namespace pcl;
using namespace visualization;
using namespace concurrency;
using namespace OMLT;

class Kinect_Thread:public QThread
{
	Q_OBJECT
	

public:
	//Kinect_Thread(PipelineUtilities::PipelineGovernor &governor, ITarget<Kinect_Data> &target);
	Kinect_Thread(PipelineUtilities::PipelineGovernor &governor, ITarget<shared_ptr<Kinect_Data>> &target);
	~Kinect_Thread(void);
	void run();
signals:
	void Kinect_Frame_Available();
public slots:
		virtual void endThread(void);
		virtual void changeColor(int);
		virtual void changeHP(int);
		virtual void hpComputeState(int);
		virtual void changeColorScheme(int);
		virtual void changeKappa(int);
		virtual void record();
private:
	bool								record_data;
	QMutex								lockGuard;
	concurrency::ITarget<shared_ptr<Kinect_Data>>   &m_KinectData;
	PipelineUtilities::PipelineGovernor &m_Governor;
	//Kinect_Data							m_CurrentFrame;
	static const int					cDepthWidth		= 512;
	static const int					cDepthHeight	= 424;
	static const int					cColorWidth		= 1920;
	static const int					cColorHeight	= 1080;
	static const int					nLength			= 217088;
	double								p1;
	double								p2;
	double								p3;
	double								p4;
	HRESULT								hr;
	CameraSpacePoint*					realPoints;
	ICoordinateMapper*					m_pCoordinateMapper;
	ColorSpacePoint*					m_pColorCoordinates;
	Colour_Display_Properties			m_Colour_Properties;
	HeadPose_Display_Properties			m_HP_Properties;
	vector<HeadLoc> heads;
	PointCloud<pcl::PointXYZRGB>::Ptr	m_Kinect2DepthCloud;
	//BYTE storage for RGBD
	RGBQUAD*							m_pDepthRGBX;
	RGBQUAD*							m_pOutputRGBX; 
	RGBQUAD*							m_pBackgroundRGBX; 
	RGBQUAD*							m_pColorRGBX;
	//OpenCV Mat to store depthmap
	Mat									m_ColorImage;
	Mat									m_DepthImage;
	Mat									m_BodyImage;
	Mat									test;
	BYTE*								depthImageBuffer;
	// Current Kinect
	IKinectSensor*						m_pKinectSensor;

	// Depth reader
	IDepthFrameReader*					m_pDepthFrameReader;
	IColorFrameReader*					m_pColorFrameReader;
	IBodyFrameReader*					m_pBodyFrameReader;
	IBodyIndexFrameReader*				m_pBodyIndexFrameReader;

	IMultiSourceFrameReader*			m_pMultiSourceFrameReader;
	bool								m_continue;
	bool								m_ComputeHeadPose;
	Model								model;
	int									input_count;
	int									output_count;
	int									colourScheme;
	float								kappa;

	HRESULT								InitializeDefaultSensor();





	void								ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth, USHORT nMinDepth, USHORT nMaxDepth);



	void								Update();



	void								ProcessFrame(INT64 nTime, 
														UINT16* pDepthBuffer, int nDepthHeight, int nDepthWidth, 
														RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
														BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight,
														int BodyCount,IBody** ppBodies, Vector4 floorPlane);
	void								ProcessFrameAMP(INT64 nTime, 
														UINT16* pDepthBuffer, int nDepthHeight, int nDepthWidth, 
														RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
														BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight,
														int BodyCount,IBody** ppBodies);
	HRESULT								InitializeMLP(string jsonFile);
	float								d2r(float d);
	
	float								r2d(float r);
	Mat									equalizeIntensity(const Mat& inputImage);
	
	//HRESULT								ComputeColourScheme(Mat &dst,UINT16* pDepthBuffer,BYTE* pBodyIndexBuffer,)

};
#endif

