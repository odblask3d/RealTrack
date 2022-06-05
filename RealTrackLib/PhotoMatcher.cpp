//--------------------------------------------------
// Implementation of class PhotoMatcher
//
// @author: Wild Boar
//
// @date: 2022-06-05
//--------------------------------------------------

#include "PhotoMatcher.h"
using namespace NVL_App;

//--------------------------------------------------
// Constructors and Terminators
//--------------------------------------------------

/**
 * @brief Custom Constructor
 * @param poseImage The photo image that we are tracking against
 */
PhotoMatcher::PhotoMatcher(PoseImage * poseImage) : _poseImage(poseImage)
{
	throw runtime_error("Not implemented");
}

//--------------------------------------------------
// Refinement
//--------------------------------------------------

/**
 * @brief Refine a pose estimation
 * @param initialPose The initial pose guess
 * @param matchImage The image that we are matching against
 * @return Mat Returns a Mat
 */
Mat PhotoMatcher::Refine(Mat& initialPose, Mat& matchImage)
{
	throw runtime_error("Not implemented");
}

//--------------------------------------------------
// Error Handlers
//--------------------------------------------------

/**
 * @brief Defines the logic to find the error
 * @param inputs The input parameters that we are processing from
 * @param errors The list of output errors associated with the inputs
 */
void PhotoMatcher::GetError(double * inputs, double * errors)
{
	throw runtime_error("Not implemented");
}

/**
 * @brief The callback for cminpack
 * @param m The number of parameters
 * @param n The number of equations
 * @param x The parameter values
 * @param fvec The errors associated with the parameters
 * @param iflag The flag values that we are using
 */
void PhotoMatcher::Callback(int * m, int * n, double * x, double * fvec, int * iflag)
{
	throw runtime_error("Not implemented");
}
