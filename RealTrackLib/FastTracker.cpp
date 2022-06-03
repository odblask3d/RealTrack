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
	throw runtime_error("Not implemented");
}
