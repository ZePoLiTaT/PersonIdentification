
//------------------------------------------------------------------------------
// <copyright file="AudioCaptureRaw.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "AudioCaptureRaw.h"

#include <iostream>

// Number of milliseconds of acceptable lag between live sound being produced and recording operation.
const int TargetLatency = 20;

int AudioCaptureRaw::startRecording()
{
	wchar_t waveFileName[MAX_PATH];
	IMMDevice *device = NULL;
	HANDLE waveFile = INVALID_HANDLE_VALUE;
	CWASAPICapture *capturer = NULL;

	std::cout<<"Raw Kinect Audio Data Capture Using WASAPI"<<std::endl;
	

	//  A GUI application should use COINIT_APARTMENTTHREADED instead of COINIT_MULTITHREADED.
	HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);
	if (SUCCEEDED(hr))
	{
		std::cout << "multithread succeeded" << std::endl;
		// Create the first connected Kinect sensor found.
		if (SUCCEEDED(hr))
		{
			//  Find the audio device corresponding to the kinect sensor.
			hr = GetKinectAudioDevice(&device);
			if (SUCCEEDED(hr))
			{
				std::cout << "GetKinectAudioDevice succeeded" << std::endl;
				// Create the wave file that will contain audio data
				hr = GetWaveFileName(waveFileName, _countof(waveFileName));
				if (SUCCEEDED(hr))
				{
					waveFile = CreateFile(waveFileName,
						GENERIC_WRITE,
						FILE_SHARE_READ,
						NULL,
						CREATE_ALWAYS,
						FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
						NULL);

					if (INVALID_HANDLE_VALUE != waveFile)
					{
						//  Instantiate a capturer
						capturer = new (std::nothrow) CWASAPICapture(device);
						if ((NULL != capturer) && capturer->Initialize(TargetLatency))
						{
							hr = CaptureAudio(capturer, waveFile, waveFileName);
							if (FAILED(hr))
							{
								std::cout << "Unable to capture audio data." << std::endl;
							}
						}
						else
						{
							std::cout << "Unable to initialize capturer."<<std::endl;
							hr = E_FAIL;
						}
					}
					else
					{
						std::cout << "Unable to create output WAV file %S.\nAnother application might be using this file." << std::endl;//, waveFileName);
						hr = E_FAIL;
					}
				}
				else
				{
					std::cout << "Unable to construct output WAV file path.\n"<<std::endl;
				}
			}
			else
			{
				std::cout << "No matching audio device found!" << std::endl;
			}
		}
		else
		{
			std::cout << "No ready Kinect found!\n"<<std::endl;
		}
	}

	std::cout << "Press any key to continue.\n"<<std::endl;

	wchar_t ch = _getwch();
	UNREFERENCED_PARAMETER(ch);

	if (INVALID_HANDLE_VALUE != waveFile)
	{
		CloseHandle(waveFile);
	}

	delete capturer;
	SafeRelease(device);
	CoUninitialize();
	return SUCCEEDED(hr) ? EXIT_SUCCESS : EXIT_FAILURE;
}

/// <summary>
/// Get an audio device that corresponds to the Kinect sensor, if such a device exists.
/// </summary>
/// <param name="ppDevice">
/// [out] Pointer to hold matching audio device found.
/// </param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT AudioCaptureRaw::GetKinectAudioDevice(IMMDevice **ppDevice)
{
	IMMDeviceEnumerator *pDeviceEnumerator = NULL;
	IMMDeviceCollection *pDeviceCollection = NULL;
	HRESULT hr = S_OK;

	*ppDevice = NULL;

	hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pDeviceEnumerator));
	if (SUCCEEDED(hr))
	{
		hr = pDeviceEnumerator->EnumAudioEndpoints(eCapture, DEVICE_STATE_ACTIVE, &pDeviceCollection);
		if (SUCCEEDED(hr))
		{
			UINT deviceCount;
			hr = pDeviceCollection->GetCount(&deviceCount);
			if (SUCCEEDED(hr))
			{
				// Iterate through all active audio capture devices looking for one that matches
				// the specified Kinect sensor.
				for (UINT i = 0; i < deviceCount; ++i)
				{
					IMMDevice *pDevice = NULL;
					bool deviceFound = false;
					hr = pDeviceCollection->Item(i, &pDevice);

					{ // Identify by friendly name
						IPropertyStore* pPropertyStore = NULL;
						PROPVARIANT varName;
						int sensorIndex = 0;

						hr = pDevice->OpenPropertyStore(STGM_READ, &pPropertyStore);
						PropVariantInit(&varName);
						hr = pPropertyStore->GetValue(PKEY_Device_FriendlyName, &varName);

						if (0 == lstrcmpW(varName.pwszVal, L"Microphone Array (Xbox NUI Sensor)") ||
							1 == swscanf_s(varName.pwszVal, L"Microphone Array (%d- Xbox NUI Sensor)", &sensorIndex))
						{
							*ppDevice = pDevice;
							deviceFound = true;
						}

						PropVariantClear(&varName);
						//SafeRelease(pPropertyStore);

						if (true == deviceFound)
						{
							break;
						}
					}

					SafeRelease(pDevice);
				}
			}

			SafeRelease(pDeviceCollection);
		}

		SafeRelease(pDeviceEnumerator);
	}

	if (SUCCEEDED(hr) && (NULL == *ppDevice))
	{
		// If nothing went wrong but we haven't found a device, return failure
		hr = E_FAIL;
	}

	return hr;
}

//
//  A wave file consists of:
//
//  RIFF header:    8 bytes consisting of the signature "RIFF" followed by a 4 byte file length.
//  WAVE header:    4 bytes consisting of the signature "WAVE".
//  fmt header:     4 bytes consisting of the signature "fmt " followed by a WAVEFORMATEX 
//  WAVEFORMAT:     <n> bytes containing a waveformat structure.
//  DATA header:    8 bytes consisting of the signature "data" followed by a 4 byte file length.
//  wave data:      <m> bytes containing wave data.
//

//  Header for a WAV file - we define a structure describing the first few fields in the header for convenience.
struct WAVEHEADER
{
	DWORD   dwRiff;                     // "RIFF"
	DWORD   dwSize;                     // Size
	DWORD   dwWave;                     // "WAVE"
	DWORD   dwFmt;                      // "fmt "
	DWORD   dwFmtSize;                  // Wave Format Size
};

//  Static RIFF header, we'll append the format to it.
const BYTE WaveHeaderTemplate[] =
{
	'R', 'I', 'F', 'F', 0x00, 0x00, 0x00, 0x00, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 0x00, 0x00, 0x00, 0x00
};

//  Static wave DATA tag.
const BYTE WaveData[] = { 'd', 'a', 't', 'a' };

/// <summary>
/// Write the WAV file header contents. 
/// </summary>
/// <param name="waveFile">
/// [in] Handle to file where header will be written.
/// </param>
/// <param name="pWaveFormat">
/// [in] Format of file to write.
/// </param>
/// <param name="dataSize">
/// Number of bytes of data in file's data section.
/// </param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT AudioCaptureRaw::WriteWaveHeader(HANDLE waveFile, const WAVEFORMATEX *pWaveFormat, DWORD dataSize)
{
	std::cout << "saving wav header" << std::endl;
	DWORD waveHeaderSize = sizeof(WAVEHEADER) + sizeof(WAVEFORMATEX) + pWaveFormat->cbSize + sizeof(WaveData) + sizeof(DWORD);
	WAVEHEADER waveHeader;
	DWORD bytesWritten;

	// Update the sizes in the header
	memcpy_s(&waveHeader, sizeof(waveHeader), WaveHeaderTemplate, sizeof(WaveHeaderTemplate));
	waveHeader.dwSize = waveHeaderSize + dataSize - (2 * sizeof(DWORD));
	waveHeader.dwFmtSize = sizeof(WAVEFORMATEX) + pWaveFormat->cbSize;

	// Write the file header
	if (!WriteFile(waveFile, &waveHeader, sizeof(waveHeader), &bytesWritten, NULL))
	{
		return E_FAIL;
	}

	// Write the format
	if (!WriteFile(waveFile, pWaveFormat, sizeof(WAVEFORMATEX) + pWaveFormat->cbSize, &bytesWritten, NULL))
	{
		return E_FAIL;
	}

	// Write the data header
	if (!WriteFile(waveFile, WaveData, sizeof(WaveData), &bytesWritten, NULL))
	{
		return E_FAIL;
	}

	if (!WriteFile(waveFile, &dataSize, sizeof(dataSize), &bytesWritten, NULL))
	{
		return E_FAIL;
	}

	return S_OK;
}

/// <summary>
/// Get the name of the file where WAVE data will be stored.
/// </summary>
/// <param name="waveFileName">
/// [out] String buffer that will receive wave file name.
/// </param>
/// <param name="waveFileNameSize">
/// [in] Number of characters in waveFileName string buffer.
/// </param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT AudioCaptureRaw::GetWaveFileName(_Out_writes_(waveFileNameSize) wchar_t *waveFileName, UINT waveFileNameSize)
{
	wchar_t *knownPath = NULL;
	HRESULT hr = SHGetKnownFolderPath(FOLDERID_Music, 0, NULL, &knownPath);

	if (SUCCEEDED(hr))
	{
		std::cout << "get file name succeeded" << std::endl;
		// Get the time
		wchar_t timeString[MAX_PATH];
		GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", timeString, _countof(timeString));

		// File name will be KinectAudio-HH-MM-SS.wav
		StringCchPrintfW(waveFileName, waveFileNameSize, L"%s\\KinectAudio-%s.wav", knownPath, timeString);
		
	}

	CoTaskMemFree(knownPath);
	return hr;
}

/// <summary>
/// Capture raw audio from Kinect USB audio device and write it out to a WAVE file.
/// </summary>
/// <param name="capturer">
/// [in] Object used to capture raw audio data from Kinect USB audio device.
/// </param>
/// <param name="waveFile">
/// [in] Handle to file where audio data will be written.
/// </param>
/// <param name="waveFileName">
/// [in] Name of file where audio data will be written.
/// </param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT AudioCaptureRaw::CaptureAudio(CWASAPICapture *capturer, HANDLE waveFile, const wchar_t *waveFileName)
{
	HRESULT hr = S_OK;
	wchar_t ch;

	// Write a placeholder wave file header. Actual size of data section will be fixed up later.
	hr = WriteWaveHeader(waveFile, capturer->GetOutputFormat(), 0);
	if (SUCCEEDED(hr))
	{
		if (capturer->Start(waveFile))
		{
			int counter = 0;
		//	printf_s("Capturing audio data to file %S\nPress 's' to stop capturing.\n", waveFileName);

			do
			{
				ch = _getwch();
				counter++;
				if (counter = 10)
					ch = 'S';
			} while (L'S' != towupper(ch));

			//printf_s("\n");
			std::cout << "capture audio done" << std::endl;
			capturer->Stop();

			// Fix up the wave file header to reflect the right amount of captured data.
			SetFilePointer(waveFile, 0, NULL, FILE_BEGIN);
			hr = WriteWaveHeader(waveFile, capturer->GetOutputFormat(), capturer->BytesCaptured());
		}
		else
		{
			hr = E_FAIL;
		}
	}

	return hr;
}
