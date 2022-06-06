//--------------------------------------------------
// Add the logic to save the output to disk
//
// @author: Wild Boar
//
// @date: 2022-06-06
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <NVLib/Model/DepthFrame.h>
#include <NVLib/FileUtils.h>

#include <opencv2/opencv.hpp>
using namespace cv;

namespace NVL_App
{
	class SaveUtils
	{
	public:
		static void SaveFrame(const string& folder, NVLib::DepthFrame * frame, int index);
		static void SavePose(const string& folder, Mat pose, int index);
	};
}
