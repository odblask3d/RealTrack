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

    _logger->Log(1, "Create a merge map");
    Mat counter_1 = Mat_<int>(frame->GetColor().size()); counter_1.setTo(1);
    Mat counter_2 = poseImage.WarpCounter(pose2, counter_1);
    Mat depthmap_1 = poseImage.GetDepth(pose2);
    Mat depthmap_2 = frame->GetDepth().clone();
    Mat depthmap_3 = MapMerger::Merge(depthmap_1, depthmap_2, counter_2);
    Mat displayCounter = counter_2 * 100;

    _logger->Log(1, "Displaying Merging Results");
    NVLib::DisplayUtils::ShowFloatMap("Depth 1", depthmap_1, 1000);
    NVLib::DisplayUtils::ShowFloatMap("Depth 2", depthmap_2, 1000);
    NVLib::DisplayUtils::ShowFloatMap("Depth 3", depthmap_3, 1000);
    NVLib::DisplayUtils::ShowImage("Counter", displayCounter, 1000);
    imwrite("Merge.tiff", depthmap_3);
    waitKey();

    _logger->Log(1, "Free working variables");
    delete firstFrame; delete frame;
}
