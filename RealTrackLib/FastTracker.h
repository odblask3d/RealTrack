//--------------------------------------------------
// Defines a tracker that uses "FAST" features for pose estimation
//
// @author: Wild Boar
//
// @date: 2022-06-04
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include <NVLib/Model/DepthFrame.h>
#include "Calibration.h"
#include "FastDetector.h"

namespace NVL_App
{
	class FastTracker
	{
	private:
		Calibration * _calibration;
		NVLib::DepthFrame * _frame;
		vector<KeyPoint> _keypoints;
	public:
		FastTracker(Calibration * calibration, NVLib::DepthFrame * firstFrame) {}

		Mat GetPose(NVLib::DepthFrame * frame, Vec2d& error, bool freePrevious);

		inline NVLib::DepthFrame *& GetFrame() { return _frame; }
		inline vector<KeyPoint>& GetKeypoints() { return _keypoints; }
	};
}
