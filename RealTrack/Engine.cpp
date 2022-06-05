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
    _logger->Log(1, "Creating a fast tracker");
    auto firstFrame = LoadUtils::LoadFrame(_folder, 0);
    auto tracker = FastTracker(_calibration, firstFrame);

    _logger->Log(1, "Estimating the pose of frame 1");
    auto frame = LoadUtils::LoadFrame(_folder, 1); auto error = Vec2d();
    auto pose = tracker.GetPose(frame, error, false);

    cout << "----------------- Results: POSE extraction" << endl;
    cout << pose << endl << endl;
    cout << "Reprojection Error: " << error[0] << " ± " << error[1] << endl;
    cout << "----------------- End POSE extraction" << endl;

    _logger->Log(1, "Showing the transform image");
    Mat camera = _calibration->GetMatrix(); 
    auto poseImage = PoseImage(camera, firstFrame);
    Mat warpImage = poseImage.GetImage(pose); 
    NVLib::DisplayUtils::ShowToggleImages("Toggle", warpImage, frame->GetColor(), 1000);

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
