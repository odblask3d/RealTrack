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

#include <opencv2/opencv.hpp>
using namespace cv;

#include <minpack.h>
#include "PoseImage.h"

namespace NVL_App
{
	class PhotoMatcher
	{
	private:
		PoseImage * _poseImage;
		inline static PhotoMatcher * _link = nullptr;
	public:
		PhotoMatcher(PoseImage * photoImage);

		Mat Refine(Mat& initialPose, Mat& matchImage);
	private:
		void GetError(double * inputs, double * errors);
		static void Callback(int * m, int * n, double * x, double * fvec, int * iflag);
	};
}
