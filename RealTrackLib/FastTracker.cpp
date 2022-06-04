//--------------------------------------------------
// Implementation of class FastTracker
//
// @author: Wild Boar
//
// @date: 2022-06-04
//--------------------------------------------------

#include "FastTracker.h"
using namespace NVL_App;

//--------------------------------------------------
// Constructors
//--------------------------------------------------

/**
 * @brief Main Constructor
 * @param calibration The main calibration parameters
 * @param firstFrame The first frame within the series
 */
FastTracker::FastTracker(Calibration * calibration, NVLib::DepthFrame * firstFrame) : _calibration(calibration), _frame(firstFrame)
{
	_detector = new FastDetector(5); _detector->Extract(firstFrame->GetColor(), _keypoints);
}

/**
 * @brief Main Terminator
 */
FastTracker::~FastTracker() 
{
	delete _detector;
}

//--------------------------------------------------
// Estimate Pose
//--------------------------------------------------

/**
 * @brief Add the logic to estimate the pose from the next frame within the sequence
 * @param frame The frame that we are getting the pose from
 * @param error The output reprojection error
 * @param freePrevious Indicates whether the memory for the previous frame can be freed or not
 * @return Mat Returns a Mat
 */
Mat FastTracker::GetPose(NVLib::DepthFrame * frame, Vec2d& error, bool freePrevious)
{
	// Extract the features that we need
	auto keypoints = vector<KeyPoint>(); _detector->Extract(frame->GetColor(), keypoints);

	// Find corresponding features
	_detector->SetFrame(_frame->GetColor(), frame->GetColor());
	auto matches = vector<FeatureMatch *>(); _detector->Match(_keypoints, keypoints, matches);

	// Estimate the pose
	Mat pose = FindPoseProcess(keypoints, matches, error);

	// Free Values
	for (auto match : matches) delete match;

	// TODO: Perform Previous Updating

	// Return the pose
	return pose;
}

/**
 * @brief Encapsulate the entire process of finding the pose
 * @param keypoints_2 The key points detected from the second image
 * @param matches The matches that were found between images
 * @param error The reprojection error due to matching
 * @return The resultant pose matrix
 */
Mat FastTracker::FindPoseProcess(vector<KeyPoint>& keypoints_2, vector<FeatureMatch *>& matches, Vec2d& error) 
{
	throw runtime_error("Not Implemented");
}

/**
 * @brief Extract the scene points from the system
 * @param camera The given camera matrix
 * @param depth The given depth map
 * @param matches The matches that were found
 * @param keypoints The list of associated key points
 * @param out The output scene points
 */
void FastTracker::GetScenePoints(Mat& camera, Mat& depth, vector<FeatureMatch *>& matches, vector<KeyPoint>& keypoints, vector<Point3f>& out) 
{
	throw runtime_error("Not Implemented");
}

/**
 * @brief Extract the associated image points
 * @param keypoints The key points that we are extracting from
 * @param out The list of output image points
 */
void FastTracker::GetImagePoints(vector<KeyPoint>& keypoints, vector<Point2f>& out) 
{
	throw runtime_error("Not Implemented");
}

/**
 * @brief Remove all the points that dont have a proper depth value associated
 * @param scenePoints The given scene points
 * @param imagePoints The given image points
 */
void FastTracker::FilterBadDepth(vector<Point3f>& scenePoints, vector<Point2f>& imagePoints) 
{
	throw runtime_error("Not Implemented");
}

/**
 * @brief Perform the pose estimation logic
 * @param camera The given camera matrix
 * @param scenePoints The list of scene points
 * @param imagePoints The list of image points
 * @return Mat The pose that was estimated
 */
Mat FastTracker::EstimatePose(Mat& camera, vector<Point3f>& scenePoints, vector<Point2f>& imagePoints) 
{
	throw runtime_error("Not Implemented");
}

/**
 * @brief Determine the reprojection error associated with the pose
 * @param camera The given camera matrix
 * @param pose The estimated pose
 * @param scenePoints The list of scene points
 * @param imagePoints The list of image points
 */
void FastTracker::EstimateError(Mat& camera, Mat& pose, vector<Point3f>& scenePoints, vector<Point2f>& imagePoints) 
{
	throw runtime_error("Not Implemented");
}