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

}
