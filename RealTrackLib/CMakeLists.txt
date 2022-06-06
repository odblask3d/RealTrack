#--------------------------------------------------------
# CMake for generating the RealTrackLib
#
# @author: Wild Boar
#
# Date Created: 2022-06-03
#--------------------------------------------------------

# Setup include directories
include_directories("../" "${LIBRARY_BASE}/NVLib")

# Create Library
add_library (RealTrackLib STATIC
	ArgUtils.cpp
	LoadUtils.cpp
	FastDetector.cpp
	FastTracker.cpp
	PoseImage.cpp
	PhotoMatcher.cpp
	MapMerger.cpp
)


