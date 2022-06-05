//--------------------------------------------------
// Implementation of class PoseImage
//
// @author: Wild Boar
//
// @date: 2022-04-20
//--------------------------------------------------

#include "PoseImage.h"
using namespace NVL_App;

//--------------------------------------------------
// Constructors and Terminators
//--------------------------------------------------

/**
 * @brief Custom Constructor
 * @param camera The camera matrix associated with the system
 * @param frame The given depth frame
 */
PoseImage::PoseImage(Mat &camera, NVLib::DepthFrame *frame) : _camera(camera)
{
	Mat depth;
	frame->GetDepth().convertTo(depth, CV_64F);
	_cloud = NVLib::CloudUtils::BuildColorCloud(camera, frame->GetColor(), depth);
	_pixelCount = depth.rows * depth.cols;
}

//--------------------------------------------------
// GetImage
//--------------------------------------------------

/**
 * @brief Get Image
 * @param pose The pose of the image we are getting
 * @return Mat Returns a Mat
 */
Mat PoseImage::GetImage(Mat &pose)
{
	return NVLib::CloudUtils::RenderImage(_cloud, _camera, pose);
}

//--------------------------------------------------
// GetDepth
//--------------------------------------------------

/**
 * @brief Merge the depth the map
 * @param pose The pose that we are finding
 * @return Mat The given depth map
 */
Mat PoseImage::GetDepth(Mat &pose)
{
	Mat tcloud = NVLib::CloudUtils::TransformCloud(_cloud, pose);
	auto cloudData = (double *)tcloud.data;

	Mat result = Mat_<float>::zeros(_cloud.size());
	auto output = (float *)result.data;

	for (auto row = 0; row < result.rows; row++)
	{
		for (auto column = 0; column < result.cols; column++)
		{
			// Get the index
			auto index = column + row * _cloud.cols;

			// Get 3D image
			auto X = cloudData[index * 6 + 0];
			auto Y = cloudData[index * 6 + 1];
			auto Z = cloudData[index * 6 + 2];
			if (Z < 300 || Z > 2500) continue;

			// Get the image location
			auto imagePoint = NVLib::Math3D::Project(_camera, Point3d(X, Y, Z));

			// Round Location
			auto u = (int)round(imagePoint.x); auto v = (int)round(imagePoint.y);
			if (u < 0 || v < 0 || u >= result.cols || v >= result.rows) continue;
			auto imageIndex = u + v * result.cols;

			// Perform Updates
			output[imageIndex] = Z;
		}
	}

	// Add a median blur to get rid of some of the holes
	medianBlur(result, result, 5);

	// Return the result
	return result;
}

//--------------------------------------------------
// WarpCounter
//--------------------------------------------------

/**
 * @brief Add the logic to warp the counter to the new "pose"
 * @param pose The pose that we are warping the counter to
 * @param counter The counter we are warping
 * @return Mat The resultant new counter location
 */
Mat PoseImage::WarpCounter(Mat& pose, Mat& counter) 
{
	// Warp the point cloud wrt the pose
	Mat tcloud = NVLib::CloudUtils::TransformCloud(_cloud, pose);
	auto cloudData = (double *)tcloud.data;

	// Create handles to the remap locations
	Mat result = Mat_<uchar>::zeros(counter.size());

	// Populate the remap tables
	for (auto row = 0; row < counter.rows; row++)
	{
		for (auto column = 0; column < counter.cols; column++)
		{
			// Get the index
			auto index = column + row * _cloud.cols;

			// Get 3D image
			auto X = cloudData[index * 6 + 0];
			auto Y = cloudData[index * 6 + 1];
			auto Z = cloudData[index * 6 + 2];
			if (Z < 300 || Z > 2500) continue;

			// Get the image location
			auto imagePoint = NVLib::Math3D::Project(_camera, Point3d(X, Y, Z));

			// Round Location
			auto u = (int)round(imagePoint.x); auto v = (int)round(imagePoint.y);
			if (u < 0 || v < 0 || u >= result.cols || v >= result.rows) continue;
			auto imageIndex = u + v * result.cols;

			// Perform Updates
			result.data[imageIndex] = counter.data[index];
		}
	}

	// Add a median blur to get rid of some of the holes
	medianBlur(result, result, 5);

	// Return the result
	return result;
}

//--------------------------------------------------
// GetScore
//--------------------------------------------------

/**
 * @brief Get the score of the image with a match image
 * @param pose The pose of the image we are getting
 * @param match The image we are matching with
 * @param errors The errors that we are dealing with
 * @return Mat Returns a Mat
 */
double PoseImage::GetScore(Mat &pose, Mat &matchImage, vector<double> &errors)
{
	Mat tcloud = NVLib::CloudUtils::TransformCloud(_cloud, pose);
	Mat imagePoints = NVLib::CloudUtils::ProjectImagePoints(_camera, tcloud);
	auto parts = vector<Mat>();
	split(imagePoints, parts);
	Mat mapx;
	parts[0].convertTo(mapx, CV_32FC1);
	Mat mapy;
	parts[1].convertTo(mapy, CV_32FC1);
	Mat samples;
	remap(matchImage, samples, mapx, mapy, INTER_LINEAR);

	auto cinput = (double *)_cloud.data;
	auto pinput = (double *)imagePoints.data;
	errors.clear();
	for (auto row = 0; row < matchImage.rows; row++)
	{
		for (auto column = 0; column < matchImage.cols; column++)
		{
			// Get the index
			auto index = column + row * matchImage.cols;

			// Figure out if we are within range
			auto x = pinput[index * 2 + 0];
			auto y = pinput[index * 2 + 1];
			if (x < 0 || x >= matchImage.cols || y < 0 || y >= matchImage.rows)
			{
				// errors.push_back(0);
				continue;
			}

			// Get the match color
			auto c11 = (int)samples.data[index * 3 + 0];
			auto c12 = (int)samples.data[index * 3 + 1];
			auto c13 = (int)samples.data[index * 3 + 2];

			// Get the cloud color
			auto c21 = (int)cinput[index * 6 + 3];
			auto c22 = (int)cinput[index * 6 + 4];
			auto c23 = (int)cinput[index * 6 + 5];

			// Calculate the score
			auto d1 = c11 - c21;
			auto d2 = c12 - c22;
			auto d3 = c13 - c23;
			auto score = sqrt(d1 * d1 + d2 * d2 + d3 * d3);
			errors.push_back(score);
		}
	}

	// Determine the average score
	auto mean = Scalar();
	auto stddev = Scalar();
	meanStdDev(errors, mean, stddev);

	// Return the result
	return mean[0];
}