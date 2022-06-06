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
 * @param keypoints The keypoints associated with the new frame
 * @param error The output reprojection error
 * @return Mat Returns a Mat
 */
Mat FastTracker::GetPose(NVLib::DepthFrame * frame, vector<KeyPoint>& keypoints, Vec2d& error)
{
	// Extract the features that we need
	_detector->Extract(frame->GetColor(), keypoints);

	// Find corresponding features
	_detector->SetFrame(_frame->GetColor(), frame->GetColor());
	auto matches = vector<FeatureMatch *>(); _detector->Match(_keypoints, keypoints, matches);

	// DEBUG: Show the correspondences
	//auto stereoFrame = NVLib::StereoFrame(_frame->GetColor(), frame->GetColor());
	//ShowMatchingPoints(stereoFrame, matches, _keypoints, keypoints);

	// Estimate the pose
	Mat pose = FindPoseProcess(keypoints, matches, error);

	// Free Values
	for (auto match : matches) delete match;

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
	// Extract the camera matrix
	Mat camera = _calibration->GetMatrix();

	// Retrieve the scene points
	auto scenePoints = vector<Point3f>(); scenePoints.clear();
	GetScenePoints(_calibration, _frame->GetDepth(), matches, _keypoints, scenePoints);

	// Retrieve the image points
	auto imagePoints = vector<Point2f>(); imagePoints.clear();
	GetImagePoints(keypoints_2, matches, imagePoints);

	// Filter the points so that they all have valid depth values
	FilterBadDepth(scenePoints, imagePoints);

	// Determine the pose value
	Mat pose = EstimatePose(camera, scenePoints, imagePoints);

	// Determine the reprojection value
	EstimateError(camera, pose, scenePoints, imagePoints, error);

	// Return the pose result
	return pose;
}

/**
 * @brief Extract the scene points from the system
 * @param calibration The calibration details
 * @param depth The given depth map
 * @param matches The matches that were found
 * @param keypoints The list of associated key points
 * @param out The output scene points
 */
void FastTracker::GetScenePoints(Calibration * calibration, Mat& depth, vector<FeatureMatch *>& matches, vector<KeyPoint>& keypoints, vector<Point3f>& out) 
{
	for (auto match : matches) 
	{
		// Retrieve image points from the system
		auto point = keypoints[match->GetFirstId()].pt;	

		// Get the depth from the system
		auto Z = ExtractDepth(depth, point);

		// Handle the error case
		if (Z <= 0) { out.push_back(Point3f()); continue; }

		// Convert to a 3D point
		auto X = (point.x - calibration->GetCenter().x) * (Z / calibration->GetFocals()[0]);
		auto Y = (point.y - calibration->GetCenter().y) * (Z / calibration->GetFocals()[1]);
		
		// Add the 3D point to the collection
		out.push_back(Point3f(X, Y, Z)); 
	}
}

/**
 * @brief Extract the associated image points
 * @param keypoints The key points that we are extracting from
 * @param matches The matches we are using in our system
 * @param out The list of output image points
 */
void FastTracker::GetImagePoints(vector<KeyPoint>& keypoints, vector<FeatureMatch *>& matches, vector<Point2f>& out) 
{
	for (auto match : matches) 
	{
		auto point = keypoints[match->GetSecondId()];
		out.push_back(point.pt);
	}
}

/**
 * @brief Remove all the points that dont have a proper depth value associated
 * @param scenePoints The given scene points
 * @param imagePoints The given image points
 */
void FastTracker::FilterBadDepth(vector<Point3f>& scenePoints, vector<Point2f>& imagePoints) 
{
	// Make sure that the incoming points are "kosher" 
	assert(scenePoints.size() == imagePoints.size());

	// Create a new container to hold the "validated" points
	auto ascenePoints = vector<Point3f>(); auto aimagePoints = vector<Point2f>();

	// Loop thru and validate the given points
	for (auto i = 0; i < scenePoints.size(); i++) 
	{
		auto scenePoint = scenePoints[i]; auto imagePoint = imagePoints[i];

		if (scenePoint.z > 300 && scenePoint.z < 2000) 
		{
			ascenePoints.push_back(scenePoint); aimagePoints.push_back(imagePoint);
		}
	}

	// Clear the incoming points
	scenePoints.clear(); imagePoints.clear();

	// Copy the new values into the array
	for (auto i = 0; i < ascenePoints.size(); i++) 
	{
		scenePoints.push_back(ascenePoints[i]);
		imagePoints.push_back(aimagePoints[i]);
	}
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
	// Convert the scene points and image points to doubles
	auto dscene = vector<Point3d>(); auto dimage = vector<Point2d>();
	for (auto i = 0; i < scenePoints.size(); i++) 
	{
		dscene.push_back(Point3d(scenePoints[i].x, scenePoints[i].y, scenePoints[i].z)); 
		dimage.push_back(Point2d(imagePoints[i].x, imagePoints[i].y));
	}

	// Perform the pose estimation
	Mat nodistortion = Mat_<double>::zeros(4,1);
	Vec3d rvec, tvec; solvePnPRansac(dscene, dimage, camera, nodistortion, rvec, tvec, false, 1e4, 10, 0.9, noArray(), SOLVEPNP_DLS);

	// Return the result
	return NVLib::PoseUtils::Vectors2Pose(rvec, tvec);
}

/**
 * @brief Determine the reprojection error associated with the pose
 * @param camera The given camera matrix
 * @param pose The estimated pose
 * @param scenePoints The list of scene points
 * @param imagePoints The list of image points
 * @param error The error point that we are getting
 */
void FastTracker::EstimateError(Mat& camera, Mat& pose, vector<Point3f>& scenePoints, vector<Point2f>& imagePoints, Vec2d& error) 
{
	// Convert the scene points and image points to doubles
	auto dscene = vector<Point3d>(); auto dimage = vector<Point2d>();
	for (auto i = 0; i < scenePoints.size(); i++) 
	{
		dscene.push_back(Point3d(scenePoints[i].x, scenePoints[i].y, scenePoints[i].z)); 
		dimage.push_back(Point2d(imagePoints[i].x, imagePoints[i].y));
	}

	// Convert the pose matrix to some vectors
	auto rvec = Vec3d(); auto tvec = Vec3d(); NVLib::PoseUtils::Pose2Vectors(pose, rvec, tvec);

	// Project 3D points to get "estimated points"
	Mat nodistortion = Mat_<double>::zeros(4,1);
	auto estimated = vector<Point2d>(); projectPoints(dscene, rvec, tvec, camera, nodistortion, estimated);

	// Calculate the errors
	auto errors = vector<double>();
    for (auto i = 0; i < estimated.size(); i++) 
    {
        auto xDiff = dimage[i].x - estimated[i].x;
        auto yDiff = dimage[i].y - estimated[i].y;
        auto length = sqrt(xDiff * xDiff + yDiff * yDiff);
		errors.push_back(length);
    }

	// Extract the error "summaries"
	auto mean = Scalar(); auto stddev = Scalar();
	cv::meanStdDev(errors, mean, stddev);
	error[0] = mean[0]; error[1] = stddev[0];
}

//--------------------------------------------------
// UpdateNextFrame
//--------------------------------------------------

/**
 * @brief Add the logic to advance to the next frame
 * @param frame The new frame that we are adding
 * @param keypoints The key points that we are adding
 * @param free Indicates whether we want to delete the value
 */
void FastTracker::UpdateNextFrame(NVLib::DepthFrame * frame, vector<KeyPoint>& keypoints, bool free) 
{
	// Perform Previous Updating
	if (free) delete _frame; _frame = frame; 
	_keypoints.clear(); 
	for (auto point : keypoints) _keypoints.push_back(point);
}

//--------------------------------------------------
// Utilities
//--------------------------------------------------

/**
 * @brief Add the logic to extract depth from a given system
 * @param depth The depth value that we are extracting
 * @param location The location of the depth value that we are extracting
 * @return float The depth value that we have gotten from the file
 */
float FastTracker::ExtractDepth(Mat& depth, const Point2f& location) 
{
	auto x = (int)round(location.x); auto y = (int)round(location.y);
	if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) return 0;
	auto data = (float *) depth.data; auto index = x + y * depth.cols;
	return data[index];
}

/**
 * @brief Show the set of corresponding points wrt the system
 * @param frame The stereo frame
 * @param matches The points that matched
 * @param keypoints_1 All the feature points for the first image
 * @param keypoints_2 All the feature points for the second image
 */
void FastTracker::ShowMatchingPoints(NVLib::StereoFrame& frame, vector<FeatureMatch *>& matches, vector<KeyPoint>& keypoints_1, vector<KeyPoint>& keypoints_2) 
{
	auto displayMatches = vector<NVLib::FeatureMatch>();
	for (auto& match : matches) 
	{	
		auto id_1 = match->GetFirstId(); auto id_2 = match->GetSecondId();
		auto m = NVLib::FeatureMatch(keypoints_1[id_1].pt, keypoints_2[id_2].pt);
		displayMatches.push_back(m);
	}

    NVLib::DisplayUtils::ShowStereoFeatures("Features", &frame, 1000, displayMatches, Vec3b(0, 0, 255));
    waitKey();
}