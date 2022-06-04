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
		FastDetector * _detector;
	public:
		FastTracker(Calibration * calibration, NVLib::DepthFrame * firstFrame);
		~FastTracker();

		Mat GetPose(NVLib::DepthFrame * frame, Vec2d& error, bool freePrevious);

		inline NVLib::DepthFrame *& GetFrame() { return _frame; }
		inline vector<KeyPoint>& GetKeypoints() { return _keypoints; }
	private:
		Mat FindPoseProcess(vector<KeyPoint>& keypoints_2, vector<FeatureMatch *>& matches, Vec2d& error);
		void GetScenePoints(Mat& camera, Mat& depth, vector<FeatureMatch *>& matches, vector<KeyPoint>& keypoints, vector<Point3f>& out);
		void GetImagePoints(vector<KeyPoint>& keypoints, vector<Point2f>& out);
		void FilterBadDepth(vector<Point3f>& scenePoints, vector<Point2f>& imagePoints);
		Mat EstimatePose(Mat& camera, vector<Point3f>& scenePoints, vector<Point2f>& imagePoints);	
		void EstimateError(Mat& camera, Mat& pose, vector<Point3f>& scenePoints, vector<Point2f>& imagePoints);
	};
}
