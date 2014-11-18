
#ifndef KINECT_AUDIO_THREAD
#define KINECT_AUDIO_THREAD
#define NOMINMAX
#pragma region HEADERS
#include <qobject.h>
#include <QThread>
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
#include <QMutex>
#endif
#include <ppl.h>
#include <concurrent_queue.h>
#include <agents.h>

#include <Global_def.h>
#include <iostream>
#include <PipelineGovernor.h>
#include "stdafx.h"
#include <shlobj.h>
#include <wchar.h>
#include <devicetopology.h>
#include <Functiondiscoverykeys_devpkey.h>

#include "WASAPICapture.h"
#pragma endregion
using namespace std;
using namespace cv;
using namespace pcl;
using namespace visualization;
using namespace concurrency;
using namespace OMLT;

class Kinect_Audio_Thread :public QThread
{
    Q_OBJECT

public:
	Kinect_Audio_Thread(PipelineUtilities::PipelineGovernor &governor, ITarget<shared_ptr<Kinect_Data>> &target);
	~Kinect_Audio_Thread(void);
	void run();

	void startAudio();

signals:
    void Kinect_AudioRecording_Done();
public slots:
    virtual void stopRecording();
	virtual void startRecording();

private:

    HRESULT	hr;
    HANDLE waveFile;
    CWASAPICapture *capturer;
    IMMDevice *device;
    char stopRecordSignal;
	bool startRecordSignal=false;
	wchar_t ch;
	QMutex	lockGuard;
    HRESULT GetKinectAudioDevice(IMMDevice **ppDevice);
    HRESULT WriteWaveHeader(HANDLE waveFile, const WAVEFORMATEX *pWaveFormat, DWORD dataSize);
    HRESULT GetWaveFileName(_Out_writes_(waveFileNameSize) wchar_t *waveFileName, UINT waveFileNameSize);
    HRESULT CaptureAudio(CWASAPICapture *capturer, HANDLE waveFile, const wchar_t *waveFileName);
};
#endif