#include "featuresworker.h"
#include <QtCore>
#include <iostream>



FeaturesWorker::FeaturesWorker(QObject *parent)
	: QObject(parent)
{

}

FeaturesWorker::FeaturesWorker()
{

}

FeaturesWorker::~FeaturesWorker()
{
	std::cout << "Ending Features Extraction";
}

void FeaturesWorker::process()
{
	while (true)
	{
		if (isRecording)
		{
			std::cout << "Starting Skeleton Features Extraction" << QThread::currentThreadId();
		}
	}
	
	emit finished();
}

