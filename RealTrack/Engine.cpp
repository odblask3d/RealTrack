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
    // Get golder
    _inputFolder = ArgUtils::GetString(parameters, "input_folder");
    _outputFolder = ArgUtils::GetString(parameters, "output_folder");

    // Retrieve the image count
    _imageCount = ArgUtils::GetInteger(parameters, "image_count");

    // Load Calibration
    auto calibrationPath = NVLib::FileUtils::PathCombine(_inputFolder, "calibration.xml");
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
    _logger->Log(1, "Loading the first frame");
    auto firstFrame = LoadUtils::LoadFrame(_inputFolder, 0);
    auto tracker = FastTracker(_calibration, firstFrame);
    Mat camera = _calibration->GetMatrix();
    Mat counter = Mat_<int>(firstFrame->GetColor().size()); counter.setTo(1);

    _logger->Log(1, "Saving the first frame details to disk");
    Mat initialPose = Mat_<double>::eye(4,4); SaveUtils::SavePose(_outputFolder, initialPose, 0);
    SaveUtils::SaveFrame(_outputFolder, firstFrame, 0);

    auto index = 1;

    for (auto i = 1; i < _imageCount; i++) 
    {
        _logger->Log(1, "Processing frame: %i", i);

        auto frame = LoadUtils::LoadFrame(_inputFolder, i); auto error = Vec2d();
        auto keypoints = vector<KeyPoint>(); Mat pose = tracker.GetPose(frame, keypoints, error);

        cout << endl << "----------------- Results: POSE extraction" << endl;
        cout << pose << endl;
        cout << "Reprojection Error: " << error[0] << " ± " << error[1] << endl;
        cout << "----------------- End POSE extraction" << endl << endl;

        if (error[0] > 3) 
        {
            _logger->Log(1, "Tracking Failed");
            delete frame;
            continue;
        }

        _logger->Log(1, "Creating a pose image");
        auto poseImage = PoseImage(camera, tracker.GetFrame());

        _logger->Log(1, "Refining pose");
        auto refiner = PhotoMatcher(&poseImage);
        pose = refiner.Refine(pose, frame->GetColor());

        _logger->Log(1, "Setting the new frame");
        counter = poseImage.WarpCounter(pose, counter);
        Mat previousDepth = poseImage.GetDepth(pose);
        frame->GetDepth() = MapMerger::Merge(previousDepth, frame->GetDepth(), counter);
        tracker.UpdateNextFrame(frame, keypoints, true);

        _logger->Log(1, "Save the frame to disk");
        SaveUtils::SavePose(_outputFolder, pose, index);
        SaveUtils::SaveFrame(_outputFolder, frame, index);
        index++;

        NVLib::DisplayUtils::ShowFloatMap("Depth", frame->GetDepth(), 1000);
        auto key = waitKey(30);
        if (key == 27) break;
    }
}
