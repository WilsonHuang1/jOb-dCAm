cmake_minimum_required(VERSION 3.10)
project(OrbbecStereoParameterExtractor)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OrbbecSDK
find_path(OrbbecSDK_INCLUDE_DIR
    NAMES libobsensor/ObSensor.hpp
    PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/OrbbecSDK_v2.4.8/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../OrbbecSDK_v2.4.8/include
        /usr/local/include
        /usr/include
)

find_library(OrbbecSDK_LIBRARY
    NAMES OrbbecSDK
    PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/OrbbecSDK_v2.4.8/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/../OrbbecSDK_v2.4.8/lib
        /usr/local/lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu
)

if(OrbbecSDK_INCLUDE_DIR AND OrbbecSDK_LIBRARY)
    set(OrbbecSDK_FOUND TRUE)
    message(STATUS "OrbbecSDK found: ${OrbbecSDK_LIBRARY}")
    message(STATUS "OrbbecSDK include: ${OrbbecSDK_INCLUDE_DIR}")
else()
    set(OrbbecSDK_FOUND FALSE)
    message(FATAL_ERROR "OrbbecSDK not found. Please check the path.")
endif()

# Create executable
add_executable(orbbec_stereo_parameter_extractor
    orbbec_stereo_parameter_extractor.cpp
)

# Include directories
target_include_directories(orbbec_stereo_parameter_extractor PRIVATE 
    ${OrbbecSDK_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(orbbec_stereo_parameter_extractor 
    ${OrbbecSDK_LIBRARY}
)

# Set output directory
set_target_properties(orbbec_stereo_parameter_extractor PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Create executable
add_executable(orbbec_stereo_more_parameter
    orbbec_stereo_more_parameter.cpp
)

# Include directories
target_include_directories(orbbec_stereo_more_parameter PRIVATE 
    ${OrbbecSDK_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(orbbec_stereo_more_parameter 
    ${OrbbecSDK_LIBRARY}
)

# Set output directory
set_target_properties(orbbec_stereo_more_parameter PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)