//--------------------------------------------------
// Keeps track of the position of the camera
//
// @author: Wild Boar
//
// @date: 2022-06-06
//--------------------------------------------------

#pragma once

#include <fstream>
#include <iostream>
using namespace std;

#include <NVLib/PoseUtils.h>

#include <opencv2/opencv.hpp>
using namespace cv;

namespace NVL_App
{
	class Trajectory
	{
		private:
			Mat _currentPose;
			vector<Point3d> _trajectory;
		public:
			Trajectory();

			void AddPose(Mat& pose);
			void Save(const string& path);

			inline Mat& GetCurrentPose() { return _currentPose; }
			inline vector<Point3d>& GetTrajectory() { return _trajectory; }
	};
}
