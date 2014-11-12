#include "featuresworker.h"
#include <QtCore>

#include <boost/make_shared.hpp>
#include "GeodesicFeatures.h"
#include "SkeletonFeatures.h"

unsigned int FeaturesWorker::framesProcessed = 0;

FeaturesWorker::FeaturesWorker(concurrency::overwrite_buffer<shared_ptr<Kinect_Data>> *target)
{
	frames = target;
}

FeaturesWorker::~FeaturesWorker()
{
	qDebug() << "Ending Features Extraction";
}



void FeaturesWorker::process()
{
	cout << ">>>>> PROCESSING: " << QThread::currentThreadId()<<endl;
	{
		QMutexLocker locker(&m_mutex);

		shared_ptr<Kinect_Data> dat;
		dat = receive(*frames);

		// Extract the features of the frame
		extractFeatures(dat);
		

		if (m_record)
		{
			cout << ">>>>> ---> RECORDING: " << QThread::currentThreadId()<<endl;
			//savefeatures(fout, feat_sk, feat_gd);
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

void FeaturesWorker::extractFeatures(shared_ptr<Kinect_Data> &dat)
{
		cout << "	NumHeads: " << (dat->num_heads);
		if (dat->num_heads > 0 && dat->new_cloud)
		{
			FeaturesWorker::framesProcessed++;

			cout << "	===Copy...." << "SRCSIZE [" << dat->new_cloud->size() << " , " << dat->new_cloud->points[0].a << "]"<<endl;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr geo_cloud = dat->new_cloud->makeShared();

			geo_cloud->points[0].a = 100;
			dat->new_cloud->points[0].a = 150;
			cout << "	Copy...." << "SRCSIZE [" << dat->new_cloud->size() << " , " << dat->new_cloud->points[0].a << "]"
								  << "COPYSIZE=" << geo_cloud->points.size() << " , " << geo_cloud->points[0].a << "]" << endl;

			SkeletonFeatures ske;

			GeodesicFeatures geo;
			//geo.processCloud(dat->new_cloud);
			geo.processCloud(geo_cloud);
			
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
				
				vector<float> feat_sk = ske.extract(dat->bodies[i],dat->floorPlane);
				for (int i = 0; i < feat_sk.size(); i++)
				{
				cout << "sd(" << (i + 1) << ") = " << feat_sk.at(i)<<endl;
				}

				vector<float> feat_gd = geo.extract(dat->bodies[i]);
				for (int i = 0; i < feat_gd.size(); i++)
				{
					cout << "gd(" << (i + 1) << ") = " << feat_gd.at(i) << endl;
				}

				
			}
			
		}
}
