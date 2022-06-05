//--------------------------------------------------
// Defines a basic engine for a vanilla C++ project.
//
// @author: Wild Boar
//
// @date: 2022-06-03
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <NVLib/Logger.h>
#include <NVLib/DisplayUtils.h>

#include <RealTrackLib/ArgUtils.h>
#include <RealTrackLib/LoadUtils.h>
#include <RealTrackLib/FastTracker.h>
#include <RealTrackLib/PoseImage.h>
#include <RealTrackLib/PhotoMatcher.h>

namespace NVL_App
{
	class Engine
	{
	private:
		NVLib::Parameters * _parameters;
		NVLib::Logger* _logger;

		string _folder;
		int _imageCount;
		Calibration * _calibration;

	public:
		Engine(NVLib::Logger* logger, NVLib::Parameters * parameters);
		~Engine();

		void Run();
	private:
		void ConvertPoints(vector<Point2f>& floatPoints, vector<Point>& outputs);
		Mat CreateCounter(Mat& depthMap);
	};
}
