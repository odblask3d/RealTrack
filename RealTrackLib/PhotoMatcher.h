//--------------------------------------------------
// A photometric refiner (with a good initialization)
//
// @author: Wild Boar
//
// @date: 2022-06-05
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <NVLib/PoseUtils.h>

#include <opencv2/opencv.hpp>
using namespace cv;

#include <minpack.h>
#include "PoseImage.h"

namespace NVL_App
{
	class PhotoMatcher
	{
	private:
		Mat _testImage;
		PoseImage * _poseImage;
		inline static PhotoMatcher * _staticLink = nullptr;
	public:
		PhotoMatcher(PoseImage * photoImage);

		Mat Refine(Mat& initialPose, Mat& matchImage);
	private:
		void GetErrors(double * inputs, double * errors);
		static void Callback(int * m, int * n, double * x, double * fvec, int * iflag);

		void Pose2Vector(Mat& pose, double * parameters);
		void Vector2Pose(double * parameters, Mat& pose);
	};
}
