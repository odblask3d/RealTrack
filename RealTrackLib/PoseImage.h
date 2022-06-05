//--------------------------------------------------
// An image that we can get the relevant pose scores
//
// @author: Wild Boar
//
// @date: 2022-04-20
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <NVLib/Math3D.h>
#include <NVLib/Model/DepthFrame.h>
#include <NVLib/CloudUtils.h>

#include <opencv2/opencv.hpp>
using namespace cv;

namespace NVL_App
{
	class PoseImage
	{
	private:
		Mat _camera;
		Mat _cloud;
		int _pixelCount;
	public:
		PoseImage(Mat& camera, NVLib::DepthFrame * frame);

		Mat GetImage(Mat& pose);
		Mat GetDepth(Mat& pose);

		double GetScore(Mat& pose, Mat& matchImage, vector<double>& errors);

		inline Mat& GetCloud() { return _cloud; }
		inline int GetPixelCount() { return _pixelCount; }
	};
}
