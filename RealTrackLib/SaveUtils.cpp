//--------------------------------------------------
// Implementation of class SaveUtils
//
// @author: Wild Boar
//
// @date: 2022-06-06
//--------------------------------------------------

#include "SaveUtils.h"
using namespace NVL_App;


//--------------------------------------------------
// SaveDepth
//--------------------------------------------------

/**
 * @brief Save the frame to disk
 * @param folder The folder that we are writing to
 * @param depth The depth map that we are writing to disk
 * @param index The index of the depth we are saving
 */
void SaveUtils::SaveFrame(const string& folder, NVLib::DepthFrame * frame, int index)
{
	auto colorFile = stringstream(); colorFile << "color_"  << setw(4) << setfill('0') << index << ".png";
	auto depthFile = stringstream();  depthFile << "depth_" << setw(4) << setfill('0') << index << ".tiff";
	auto colorPath = NVLib::FileUtils::PathCombine(folder, colorFile.str());
	auto depthPath = NVLib::FileUtils::PathCombine(folder, depthFile.str());
	imwrite(depthPath, frame->GetDepth()); imwrite(colorPath, frame->GetColor());
}

//--------------------------------------------------
// SavePose
//--------------------------------------------------

/**
 * @brief Save the pose to disk
 * @param folder the folder that we are writing to
 * @param pose The pose that we are writing to disk
 * @param index The index of the pose that we are saving
 */
void SaveUtils::SavePose(const string& folder, Mat pose, int index)
{
	auto fileName = stringstream();  fileName << "pose_" << setw(4) << setfill('0') << index << ".xml";
	auto path = NVLib::FileUtils::PathCombine(folder, fileName.str());
	auto writer = FileStorage(path, FileStorage::FORMAT_XML | FileStorage::WRITE);
	writer << "pose" << pose;
	writer.release();
}
