//--------------------------------------------------
// Implementation code for the Engine
//
// @author: Wild Boar
//
// @date: 2022-06-03
//--------------------------------------------------

#include "Engine.h"
using namespace NVL_App;

//--------------------------------------------------
// Constructor and Terminator
//--------------------------------------------------

/**
 * Main Constructor
 * @param logger The logger that we are using for the system
 * @param parameters The input parameters
 */
Engine::Engine(NVLib::Logger* logger, NVLib::Parameters* parameters) : _logger(logger), _parameters(parameters)
{
    // Get the image folder
    _folder = ArgUtils::GetString(parameters, "input_folder");

    // Retrieve the image count
    _imageCount = ArgUtils::GetInteger(parameters, "image_count");

    // Load Calibration
    auto calibrationPath = NVLib::FileUtils::PathCombine(_folder, "calibration.xml");
    _calibration = LoadUtils::LoadCalibration(calibrationPath);
}

/**
 * Main Terminator 
 */
Engine::~Engine() 
{
    delete _parameters; delete _calibration;
}

//--------------------------------------------------
// Execution Entry Point
//--------------------------------------------------

/**
 * Entry point function
 */
void Engine::Run()
{
    auto index = 10;

    _logger->Log(1, "Creating a fast tracker");
    auto firstFrame = LoadUtils::LoadFrame(_folder, index);
    auto tracker = FastTracker(_calibration, firstFrame);

    _logger->Log(1, "Estimating the pose of frame 1");
    auto frame = LoadUtils::LoadFrame(_folder, index + 2); auto error = Vec2d();
    Mat pose = tracker.GetPose(frame, error, false);

    cout << endl << "----------------- Results: POSE extraction" << endl;
    cout << pose << endl;
    cout << "Reprojection Error: " << error[0] << " ± " << error[1] << endl;
    cout << "----------------- End POSE extraction" << endl << endl;

    _logger->Log(1, "Generating a pose image");
    Mat camera = _calibration->GetMatrix(); 
    auto poseImage = PoseImage(camera, firstFrame);
    auto errors = vector<double>(); auto initialError = poseImage.GetScore(pose, frame->GetColor(), errors);
    _logger->Log(1, "Initial Error: %f", initialError);

    _logger->Log(1, "Launching the refinement of the pose");
    auto refiner = PhotoMatcher(&poseImage);
    Mat pose2 = refiner.Refine(pose, frame->GetColor());

    _logger->Log(1, "Performing counters test");
    Mat counter_1 = CreateCounter(firstFrame->GetDepth());
    Mat counter_2 = poseImage.WarpCounter(pose2, counter_1);
    Mat depthmap = poseImage.GetDepth(pose2);
    imshow("Counter 1", counter_1); imshow("Counter 2", counter_2); 
    NVLib::DisplayUtils::ShowFloatMap("Depth", depthmap, 1000);
    waitKey();


    _logger->Log(1, "Free working variables");
    delete firstFrame; delete frame;
}

//--------------------------------------------------
// Utility Methods
//--------------------------------------------------

/**
 * @brief Add the functionality to convert given points
 * @param floatPoints The list of points that we are converting
 * @param outputs The output points that we are converting
 */
void Engine::ConvertPoints(vector<Point2f>& floatPoints, vector<Point>& outputs) 
{
    for (auto& point : floatPoints) 
    {
        auto x = (int) round(point.x);
        auto y = (int) round(point.y);
        outputs.push_back(Point(x,y));
    }
}

/**
 * @brief Add the counter
 * @param depthMap The depth map that we are dealing with
 * @return The result
 */
Mat Engine::CreateCounter(Mat& depthMap) 
{
    Mat result = Mat_<uchar>(depthMap.size());

    auto depthData = (float *) depthMap.data;

    for (auto row = 0; row < depthMap.rows; row++) 
    {
        for (auto column = 0; column < depthMap.cols; column++) 
        {
            auto index = column + row * depthMap.cols;
            auto Z = depthData[index];

            if (Z < 300 || Z > 2500) result.data[index] = 0;
            else result.data[index] = 100;
        }
    }

    return result;
}
