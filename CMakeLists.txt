#--------------------------------------------------------
# Top-Level: RealTrack
#
# @author: Wild Boar
#
# @Date Created: 2022-06-03
#--------------------------------------------------------

# Create the main application
cmake_minimum_required(VERSION 3.0.0)
project(RealTrack VERSION 0.1.0)

# Enable Testing
enable_testing()

# Set the correct version of C++
set(CMAKE_CXX_STANDARD 17)

# Setup base directory
set(LIBRARY_BASE $ENV{HOME}/Libraries)

# Setup the includes for the libraries
include_directories("../" "${LIBRARY_BASE}/NVLib")

# Add the NVLib library to the folder
add_library(NVLib STATIC IMPORTED)
set_target_properties(NVLib PROPERTIES
    IMPORTED_LOCATION "${LIBRARY_BASE}/NVLib/build/NVLib/libNVLib.a"
    INTERFACE_INCLUDE_DIRECTORIES "${LIBRARY_BASE}/NVLib"
)

# Add opencv to the solution
find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add the available subfolders
add_subdirectory(RealTrackLib)
add_subdirectory(RealTrackTests)
add_subdirectory(RealTrack)

