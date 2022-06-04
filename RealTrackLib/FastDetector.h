//--------------------------------------------------
// Defines a detector that uses FAST
//
// @author: Wild Boar
//
// @date: 2022-06-03
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include <NVLib/FeatureUtils.h>
#include <NVLib/Model/StereoFrame.h>

#include "FeatureMatch.h"

namespace NVL_App
{
	class FastDetector
	{
	private:
		int _blockSize;
		NVLib::StereoFrame * _frame;
	public:
		FastDetector(int blockSize) : _blockSize(blockSize) { _frame = nullptr; }
		~FastDetector() { if (_frame != nullptr) delete _frame; }

		void Extract(Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors); 
		void Match(vector<KeyPoint>& kp_1, vector<KeyPoint>& kp_2, Mat& descriptor_1, Mat& descriptor_2, vector<FeatureMatch *>& output);

		void SetFrame(Mat& image1, Mat& image2);
	private:
		int GetIndex(const Point2d& point, int blockSize, int width);
		void FindMatches(vector<Point2f>& pointSet1, vector<Point2f>& pointSet2, vector<FeatureMatch *>& matches);
		void FilterOnError(vector<uchar>& status, vector<float>& errors, vector<FeatureMatch *>& matches);
		void EpipolarFilter(vector<Point2f>& pointSet1, vector<Point2f>& pointSet2, vector<FeatureMatch *>& output);
		Mat BuildPointDescriptor(vector<Point2f>& points);
	};
}
