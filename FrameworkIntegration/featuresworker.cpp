#include "featuresworker.h"
#include <QtCore>


#include <boost/make_shared.hpp>
#include "GeodesicFeatures.h"
#include "SkeletonFeatures.h"

unsigned int FeaturesWorker::framesProcessed = 0;

FeaturesWorker::FeaturesWorker(concurrency::unbounded_buffer<shared_ptr<Kinect_Data>> *target)
{
	frames = target;
	fout.open("C:/local/data/test2.csv", ios::out);
}

FeaturesWorker::~FeaturesWorker()
{

	fout.close();
	qDebug() << "Ending Features Extraction";
}

void savefeatures(ofstream &fout, vector<float> skfeat, vector<float> gdfeat)
{
	vector <float>::iterator featIterator;

	for (featIterator = skfeat.begin(); featIterator != skfeat.end(); featIterator++)
	{
		fout << *featIterator << ", ";
	}

	for (featIterator = gdfeat.begin(); featIterator != gdfeat.end(); featIterator++)
	{
		fout << *featIterator << ", ";
	}
	fout << endl;
}

void receiveFrame(shared_ptr<Kinect_Data> &m_CurrentFrame, shared_ptr<Kinect_Data> &m_lastFrame)
{	
	// Create a copy of the point cloud

	//TODO: Delete this
	cout << "	===Copy...." << "SRCSIZE [" << m_CurrentFrame->new_cloud->size() << " , " << m_CurrentFrame->new_cloud->points[0].a << "]" << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr geo_cloud = m_CurrentFrame->new_cloud->makeShared();

	//TODO: Remove me
	geo_cloud->points[0].a = 100;
	m_CurrentFrame->new_cloud->points[0].a = 150;
	cout << "	Copy...." << "SRCSIZE [" << m_CurrentFrame->new_cloud->size() << " , " << m_CurrentFrame->new_cloud->points[0].a << "]"
						  << "COPYSIZE=" << geo_cloud->points.size() << " , " << geo_cloud->points[0].a << "]" << endl;

	// Create a new Kinect Data from the Current Frame
	m_lastFrame->num_heads = m_CurrentFrame->num_heads;
	m_lastFrame->floorPlane = m_CurrentFrame->floorPlane;
	m_lastFrame->locations = m_CurrentFrame->locations;
	m_lastFrame->bodies = m_CurrentFrame->bodies;
	m_lastFrame->new_cloud = geo_cloud;
}

void FeaturesWorker::process()
{
	cout << ">>>>> PROCESSING: " << QThread::currentThreadId()<<endl;
	{
		QMutexLocker locker(&m_mutex);

		// Receive data from the override buffer
		shared_ptr<Kinect_Data> m_CurrentFrame;
		m_CurrentFrame = receive(*frames);

		// Copy the data for processing (because the buffer is constantly being override)
		//shared_ptr<Kinect_Data> featureFrame(new Kinect_Data());
		//receiveFrame(m_CurrentFrame, featureFrame);

		// Extract the features of the frame
		vector<float> feat_sk;
		vector<float> feat_gd;
		extractFeatures(m_CurrentFrame, feat_sk, feat_gd);
		

		if (m_record)
		{
			cout << ">>>>> ---> RECORDING: " << QThread::currentThreadId()<<endl;
			savefeatures(fout, feat_sk, feat_gd);
		}
		else
		{
			cout << ">>>>> ----> NOT RECORDING: " << QThread::currentThreadId()<<endl;
		}
	}

	////emit featuresAvailable();
}

void FeaturesWorker::SwitchRecording()
{
	cout << ">>>>>  Thread::SwitchRecording called from main thread: " << QThread::currentThreadId()<<endl;
	QMutexLocker locker(&m_mutex);
	m_record = !m_record;
}

void FeaturesWorker::extractFeatures(shared_ptr<Kinect_Data> &dat, vector<float> &feat_sk, vector<float> &feat_gd)
{
		cout << "	NumHeads: " << (dat->num_heads);
		if (dat->num_heads > 0 && dat->new_cloud)
		{
			FeaturesWorker::framesProcessed++;

			SkeletonFeatures ske;
			GeodesicFeatures geo;

			geo.processCloud(dat->new_cloud);
			
			for (int i = 0; i < dat->num_heads; i++)
			{
				cout << endl << "============== PERSON [" << i << "] =============";
	
				//Eigen::Vector3f trans = Eigen::Vector3f(dat->bodies[i].at(JointType::JointType_SpineMid).Loc3D.x, 
				//										dat->bodies[i].at(JointType::JointType_SpineMid).Loc3D.y, 
				//										dat->bodies[i].at(JointType::JointType_SpineMid).Loc3D.z - 0.2);
	
				//Eigen::Quaternionf quat = Eigen::Quaternionf();
				//getQuat(0, 0, 0, quat);
				//quat.normalize();
				//viewer->addCube(trans, quat, 0.05, 0.05, 0.05, "x");
				
				feat_sk = ske.extract(dat->bodies[i],dat->floorPlane);
				for (int i = 0; i < feat_sk.size(); i++)
				{
				cout << "sd(" << (i + 1) << ") = " << feat_sk.at(i)<<endl;
				}

				feat_gd = geo.extract(dat->bodies[i]);
				for (int i = 0; i < feat_gd.size(); i++)
				{
					cout << "gd(" << (i + 1) << ") = " << feat_gd.at(i) << endl;
				}

				
			}
			
		}
}
