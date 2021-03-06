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
#include <RealTrackLib/MapMerger.h>
#include <RealTrackLib/SaveUtils.h>
#include <RealTrackLib/Trajectory.h>

namespace NVL_App
{
	class Engine
	{
	private:
		NVLib::Parameters * _parameters;
		NVLib::Logger* _logger;

		string _inputFolder;
		string _outputFolder;
		int _imageCount;
		Calibration * _calibration;

	public:
		Engine(NVLib::Logger* logger, NVLib::Parameters * parameters);
		~Engine();

		void Run();
	};
}
