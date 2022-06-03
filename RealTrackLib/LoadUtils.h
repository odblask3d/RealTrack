//--------------------------------------------------
// A utility for the purpose of loading
//
// @author: Wild Boar
//
// @date: 2022-05-31
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include <NVLib/LoadUtils.h>
#include <NVLib/FileUtils.h>
#include <NVLib/Model/DepthFrame.h>

#include "Calibration.h"

namespace NVL_App
{
	class LoadUtils
	{
	public:
		static NVLib::DepthFrame * LoadFrame(const string& folder, int index);
		static Calibration * LoadCalibration(const string& path);
	};
}