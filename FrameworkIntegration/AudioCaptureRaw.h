#include "stdafx.h"
#include <shlobj.h>
#include <wchar.h>
#include <devicetopology.h>
#include <Functiondiscoverykeys_devpkey.h>

#include "WASAPICapture.h"

class AudioCaptureRaw
{
public:
	int startRecording();

private:
	HRESULT GetKinectAudioDevice(IMMDevice **ppDevice);
	HRESULT WriteWaveHeader(HANDLE waveFile, const WAVEFORMATEX *pWaveFormat, DWORD dataSize);
	HRESULT GetWaveFileName(_Out_writes_(waveFileNameSize) wchar_t *waveFileName, UINT waveFileNameSize);
	HRESULT CaptureAudio(CWASAPICapture *capturer, HANDLE waveFile, const wchar_t *waveFileName);
};