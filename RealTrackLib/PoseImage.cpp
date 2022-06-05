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
PoseImage::PoseImage(Mat& camera, NVLib::DepthFrame * frame) : _camera(camera)
{
	Mat depth; frame->GetDepth().convertTo(depth, CV_64F);
	_cloud = NVLib::CloudUtils::BuildColorCloud(camera, frame->GetColor(), depth);
}

//--------------------------------------------------
// Transform
//--------------------------------------------------

/**
 * @brief Get Image
 * @param pose The pose of the image we are getting
 * @return Mat Returns a Mat
 */
Mat PoseImage::GetImage(Mat& pose)
{
	return NVLib::CloudUtils::RenderImage(_cloud, _camera, pose);
}

/**
 * @brief Get the score of the image with a match image
 * @param pose The pose of the image we are getting
 * @param match The image we are matching with
 * @return Mat Returns a Mat
 */
double PoseImage::GetScore(Mat& pose, Mat& matchImage)
{
	Mat tcloud = NVLib::CloudUtils::TransformCloud(_cloud, pose);
	Mat imagePoints = NVLib::CloudUtils::ProjectImagePoints(_camera, tcloud);
	auto parts = vector<Mat>(); split(imagePoints, parts);
	Mat mapx; parts[0].convertTo(mapx, CV_32FC1);
	Mat mapy; parts[1].convertTo(mapy, CV_32FC1);
	Mat samples; remap(matchImage, samples, mapx, mapy, INTER_CUBIC);

	auto cinput = (double*)_cloud.data;
	auto counter = 0; auto pinput = (double *) imagePoints.data; auto totalScore = 0.0;
	for (auto row = 0; row < matchImage.rows; row++)
	{
		for (auto column = 0; column < matchImage.cols; column++)
		{
			// Get the index
			auto index = column + row * matchImage.cols;
			
			// Figure out if we are within range
			auto x = pinput[index * 2 + 0]; auto y = pinput[index * 2 + 1];
			if (x < 0 || x >= matchImage.cols || y < 0 || y >= matchImage.rows) continue;

			// Get the match color
			auto c11 = (int)samples.data[index * 3 + 0];
			auto c12 = (int)samples.data[index * 3 + 1];
			auto c13 = (int)samples.data[index * 3 + 2];

			// Get the cloud color
			auto c21 = (int)cinput[index * 6 + 3];
			auto c22 = (int)cinput[index * 6 + 4];
			auto c23 = (int)cinput[index * 6 + 5];

			// Calculate the score
			auto d1 = c11 - c21; auto d2 = c12 - c22; auto d3 = c13 - c23;
			auto score = sqrt(d1 * d1 + d2 * d2 + d3 * d3);

			totalScore += score; counter++;
		}
	}

	// Determine the average score
	auto score = (totalScore / counter);

	// Return the result
	return score;
}