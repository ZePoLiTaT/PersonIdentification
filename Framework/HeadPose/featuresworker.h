#ifndef FEATURESWORKER_H
#define FEATURESWORKER_H

#include <QObject>
#include <QtCore>
#include <QThread>
#include <iostream>
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
	FeaturesWorker(concurrency::overwrite_buffer<shared_ptr<Kinect_Data>> *target);
	~FeaturesWorker();

	void extractFeatures(shared_ptr<Kinect_Data> &dat);

	public slots:
	void process();
	void SwitchRecording();

	//signals:
	//	void featuresAvailable();
private:
	QMutex m_mutex;
	bool m_record = false;
	static unsigned int framesProcessed;

	concurrency::overwrite_buffer<shared_ptr<Kinect_Data>> *frames;
};

#endif // FEATURESWORKER_H
