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
    _inputFolder = ArgUtils::GetString(parameters, "input_folder");

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
    auto imageCount = ArgUtils::GetInteger(_parameters, "image_count");
    for (auto i = 0; i < imageCount; i++) 
    {
        auto frame = LoadUtils::LoadFrame(_inputFolder, i);

        NVLib::DisplayUtils::ShowDepthFrame("Frame", frame, 1000);
        auto key = waitKey(30);

        delete frame;
    
        if (key == 27) break;
    }

}
