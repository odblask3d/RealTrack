#--------------------------------------------------------
# CMake for generating the main executable
#
# @author: Wild Boar
#
# Date Created: 2022-06-03
#--------------------------------------------------------

# Setup the includes
include_directories("../")

# Create the executable
add_executable(RealTrack
    Engine.cpp
    Source.cpp
)

# Add link libraries                               
target_link_libraries(RealTrack RealTrackLib  NVLib ${OpenCV_LIBS} uuid cminpack)

# Copy Resources across
add_custom_target(resource_copy ALL
	COMMAND cmake -E copy_directory ${CMAKE_SOURCE_DIR}/Resources ${CMAKE_BINARY_DIR}/RealTrack 
    DEPENDS ${MY_TARGET}
)
