
#ifndef _IMU_THREAD_H
#define _IMU_THREAD_H
#include<iostream>
#include<qobject.h>
#include<qthread.h>
#include <XbimuReceiver.h>
#include <Serial.h>
 #include <QMetaType>
#include<Quaternion.h>
#include <ppl.h>
#include <concurrent_queue.h>
#include <agents.h>
struct QuaternionValue
{
	float Qw;
	float Qx;
	float Qy;
	float Qz;
};
Q_DECLARE_METATYPE(QuaternionValue);
class IMU_THREAD:public QThread
{
	Q_OBJECT
public:
	 IMU_THREAD(concurrency::ITarget<std::tuple<QuaternionValue,EulerAnglesStruct>> &target);
	~IMU_THREAD(void);
	void run();
	concurrency::ITarget<std::tuple<QuaternionValue,EulerAnglesStruct>>   &m_Sensor_Data;
signals:
	void Sensor_Data_Available(QuaternionValue data,EulerAnglesStruct angles);
public slots:
	void endThreadExec(void);
	void getNext(void);
private:

	Quaternion *quaternion;
	QuaternionValue *data;
	unsigned char *a;
	XbimuReceiver dataProcessor;
	CSerial sensor;
	bool m_abort,m_continue;
	float Qw,Qx,Qy,Qz;
};
#endif