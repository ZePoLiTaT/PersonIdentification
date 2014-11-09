#ifndef FEATURESWORKER_H
#define FEATURESWORKER_H

#include <QObject>

class FeaturesWorker : public QObject
{
	Q_OBJECT

public:
	FeaturesWorker(QObject *parent);
	FeaturesWorker();
	~FeaturesWorker();



public slots:
	void process();
	void SwitchRecording() { isRecording = !isRecording; }
signals:
	void finished();
private:
	bool isRecording = false;
};

#endif // FEATURESWORKER_H
