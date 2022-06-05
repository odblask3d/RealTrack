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
	_staticLink = this;
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
	// Set the given start image
	_testImage = matchImage;

	// Setup descent parameters
	int j, m, n, info, lwa, iwa[_poseImage->GetPixelCount()], one = 1;

	m = 6; n = 6; lwa = (m * n) + (5 * n) + m;

  	double x[n], fvec[m], wa[lwa]; 
	  
	// Initialize the variables  
	Pose2Vector(initialPose, x);

	// Generate tol
	auto tol = sqrt(dpmpar_(&one));

	// Perform the levenberg marquardt descent
  	lmdif1_(&Callback, &m, &n, x, fvec, &tol, &info, iwa, wa, &lwa);

	// Final update of the result
	Mat pose; 
	Vector2Pose(x, pose);
	return pose;
}

//--------------------------------------------------
// Error Handlers
//--------------------------------------------------

/**
 * @brief Defines the logic to find the error
 * @param inputs The input parameters that we are processing from
 * @param errors The list of output errors associated with the inputs
 */
void PhotoMatcher::GetErrors(double * inputs, double * errors)
{
	// Retrieve the pose
	Mat pose; Vector2Pose(inputs, pose);

	// Attempt to match the warp image
	auto errorVector = vector<double>(); auto score = _poseImage->GetScore(pose, _testImage, errorVector);

	// Copy the errors back
	for (auto i = 0; i < 6; i++) errors[i] = score;

	// Show the current score
	cout << "Error Score: " << score << endl;
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
	_staticLink->GetErrors(x, fvec);
}

//--------------------------------------------------
// Conversion
//--------------------------------------------------

/**
 * @brief Convert a pose into a vector of parameters
 * @param pose The pose that we are converting
 * @param parameters The parameters that we have converted to
 */
void PhotoMatcher::Pose2Vector(Mat& pose, double * parameters) 
{
	auto rvec = Vec3d(); auto tvec = Vec3d();
	NVLib::PoseUtils::Pose2Vectors(pose, rvec, tvec);

	parameters[0] = rvec[0]; parameters[1] = rvec[1]; parameters[2] = rvec[2];
	parameters[3] = tvec[0]; parameters[4] = tvec[1]; parameters[5] = tvec[2]; 
}

/**
 * @brief Convert a vector to a given pose
 * @param parameters The parameters that we have converted
 * @param pose The pose that we have obtained
 */
void PhotoMatcher::Vector2Pose(double * parameters, Mat& pose) 
{
	auto rvec = Vec3d(parameters[0], parameters[1], parameters[2]);
	auto tvec = Vec3d(parameters[3], parameters[4], parameters[5]);
	pose = NVLib::PoseUtils::Vectors2Pose(rvec, tvec);
}