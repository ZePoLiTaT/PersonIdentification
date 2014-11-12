#ifndef FEATURESWORKER_H
#define FEATURESWORKER_H

#include <QObject>
#include <QtCore>
#include <QThread>
#include <iostream>
#include <fstream>
#include <QMutex>

#include <ppl.h>
#include <concurrent_queue.h>
#include <agents.h>

#include <Kinect.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Global_def.h"

using namespace concurrency;
using namespace std;

class FeaturesWorker : public QObject
{
	Q_OBJECT

public:
	FeaturesWorker(concurrency::unbounded_buffer<shared_ptr<Kinect_Data>> *target);
	~FeaturesWorker();

	void extractFeatures(shared_ptr<Kinect_Data> &dat, vector<float> &feat_sk, vector<float> &feat_gd);

	public slots:
	void process();
	void SwitchRecording();

	//signals:
	//	void featuresAvailable();
private:
	QMutex m_mutex;
	bool m_record = false;
	static unsigned int framesProcessed;

	concurrency::unbounded_buffer<shared_ptr<Kinect_Data>> *frames;
	ofstream fout;
};

#endif // FEATURESWORKER_H
