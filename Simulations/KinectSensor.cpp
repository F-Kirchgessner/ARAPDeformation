#include "stdafx.h"
#include "KinectSensor.h"

#include <strsafe.h>
#include "resource.h"
#include <fstream>

KinectSensor::KinectSensor()
{
	std::cout << "kinect" << std::endl;
	HRESULT hr = createFirstConnected();
}


KinectSensor::~KinectSensor()
{
	if (m_pNuiSensor)
	{
		m_pNuiSensor->NuiShutdown();
	}

	if (m_hNextSkeletonEvent && (m_hNextSkeletonEvent != INVALID_HANDLE_VALUE))
	{
		CloseHandle(m_hNextSkeletonEvent);
	}
}

void KinectSensor::ResetKinect1()
{
	if (m_pNuiSensor)
	{
		m_pNuiSensor->NuiShutdown();
	}
	if (m_hNextSkeletonEvent && (m_hNextSkeletonEvent != INVALID_HANDLE_VALUE))
	{
		CloseHandle(m_hNextSkeletonEvent);
	}
	HRESULT hr = createFirstConnected();
}

void KinectSensor::Update()
{
	while (1) {
		if (NULL == m_pNuiSensor)
		{
			return;
		}

		// Wait for 0ms, just quickly test if it is time to process a skeleton
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonEvent, 0))
		{
			ProcessSkeleton();
		}
	}
}

NUI_SKELETON_FRAME KinectSensor::GetSkeletonframe()
{
	NUI_SKELETON_FRAME skeletonFrame = { 0 };

	HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
	if (FAILED(hr))
	{
		return skeletonFrame;
	}

	m_pNuiSensor->NuiTransformSmooth(&skeletonFrame, NULL);

	return(skeletonFrame);

}

void KinectSensor::ProcessSkeleton() {

	NUI_SKELETON_FRAME skeletonFrame = { 0 };

	HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
	if (FAILED(hr))
	{
		return;
	}

	// smooth out the skeleton data
	m_pNuiSensor->NuiTransformSmooth(&skeletonFrame, NULL);


	for (int i = 0; i < NUI_SKELETON_COUNT; i++) //Six times, because the Kinect has space to track six people
	{
		NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;
		//std::cout << "ProcessSkeleton" << std::endl;

		if (NUI_SKELETON_TRACKED == trackingState) {
			std::cout << "Right Hand: "; //Print "Right hand:"
			std::cout << skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x << std::endl;
			std::cout << skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y << std::endl;
			std::cout << skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z << std::endl;
			//See more on this line below
		}

	}
}

HRESULT KinectSensor::createFirstConnected()
{
	INuiSensor * pNuiSensor;

	int iSensorCount = 0;
	HRESULT hr = NuiGetSensorCount(&iSensorCount);
	if (FAILED(hr))
	{
		return hr;
	}

	// Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			m_pNuiSensor = pNuiSensor;
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}

	if (NULL != m_pNuiSensor)
	{
		// Initialize the Kinect and specify that we'll be using skeleton
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON);
		if (SUCCEEDED(hr))
		{
			// Create an event that will be signaled when skeleton data is available
			m_hNextSkeletonEvent = CreateEventW(NULL, TRUE, FALSE, NULL);

			// Open a skeleton stream to receive skeleton data
			//hr = m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);
			hr = m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, 0);
			//std::cout << "m_pNuiSensor: " << hr << std::endl;
		}
	}

	if (NULL == m_pNuiSensor || FAILED(hr))
	{
		//SetStatusMessage(L"No ready Kinect found!");
		std::cout << "Failed hr: " << hr << std::endl;
		std::cout << "Failed m_pNuiSensor: " << m_pNuiSensor << std::endl;
		return E_FAIL;
	}

	return hr;
}

void KinectSensor::SetStatusMessage(WCHAR * szMessage)
{
	//SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, (LPARAM)szMessage);
}