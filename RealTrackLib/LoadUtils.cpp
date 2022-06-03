//--------------------------------------------------
// Implementation of class LoadUtils
//
// @author: Wild Boar
//
// @date: 2022-05-31
//--------------------------------------------------

#include "LoadUtils.h"
using namespace NVL_App;

//--------------------------------------------------
// Load Depth Frames
//--------------------------------------------------

/**
 * @brief Load a collection of depth frame from the system
 * @param folder The folder to load the frames from
 * @param frameCount The frameCount of frames that we want to load
 * @return Returns the depth frame that we have loaded
 */
NVLib::DepthFrame * LoadUtils::LoadFrame(const string& folder, int index)
{
	auto colorFile = stringstream(); colorFile << "color_" << setw(4) << setfill('0') << index << ".png";
	auto depthFile = stringstream(); depthFile << "depth_" << setw(4) << setfill('0') << index << ".tiff";
	auto colorPath = NVLib::FileUtils::PathCombine(folder, colorFile.str());
	auto depthPath = NVLib::FileUtils::PathCombine(folder, depthFile.str());
	return NVLib::LoadUtils::LoadDepthFrame(colorPath, depthPath);
}

//--------------------------------------------------
// Load Calibration
//--------------------------------------------------

/**
 * @brief Load calibration details from disk
 * @param path The path where the calibration file is located
 * @return Calibration * Returns a Calibration *
 */
Calibration * LoadUtils::LoadCalibration(const string& path)
{
	auto reader = FileStorage(path, FileStorage::FORMAT_XML | FileStorage::READ);
	if (!reader.isOpened()) throw runtime_error("Unable to open file: " + path);

	Mat camera; reader["camera"] >> camera; if (camera.empty()) throw runtime_error("Unable to read camera matrix from the calibration file");
	auto input = (double *) camera.data;

	auto focals = Vec2d(input[0], input[4]); auto center = Point2d(input[2], input[5]);

	reader.release();

	return new Calibration(focals, center);
}