#pragma once

#include <iostream>
#include <Windows.h>
#include <NuiApi.h>
#include <NuiSkeleton.h>

class KinectSensor
{
public:
	KinectSensor();
	~KinectSensor();

	void Update();
	void ProcessSkeleton();

	//to get the skeleton frame
	NUI_SKELETON_FRAME GetSkeletonframe();

	HRESULT createFirstConnected();
	

private:
	INuiSensor * m_pNuiSensor;

	HANDLE                  m_pSkeletonStreamHandle;
	HANDLE                  m_hNextSkeletonEvent;

	void                    SetStatusMessage(WCHAR* szMessage);
};

