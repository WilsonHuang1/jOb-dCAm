cmake_minimum_required(VERSION 3.16)
project(aurora_3d_scanner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages (without viz module)
set(OpenCV_DIR "C:/Program Files (x86)/opencv/build")
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found!")
endif()

# Check if viz module is available (optional)
find_package(OpenCV QUIET COMPONENTS viz)
if(OpenCV_viz_FOUND)
    message(STATUS "OpenCV viz module found - enabling 3D visualization")
    add_definitions(-DHAVE_OPENCV_VIZ)
else()
    message(STATUS "OpenCV viz module not found - using alternative visualization")
endif()

# Define device type (adjust based on your Aurora model)
add_definitions(-DDEVICE_TYPE_AURORA900)

# Include directories (adjust paths to match your SDK installation)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../samples/src
    ${OpenCV_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    aurora_3d_scanner.cpp
    # Note: We'll include sample_helper functions directly or link to existing library
)

# Try to find sample_helper.cc if available
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../sample_helper.cc")
    list(APPEND SOURCES "../sample_helper.cc")
    message(STATUS "Found sample_helper.cc - including in build")
else()
    message(STATUS "sample_helper.cc not found - SavePoint functions must be available in linked library")
endif()

# Create executable
add_executable(aurora_3d_scanner ${SOURCES})

# Link libraries
target_link_libraries(aurora_3d_scanner 
    ${DEPTRUM_STREAM_LIB_NAME}  # Aurora SDK library
    ${OpenCV_LIBS}
    ${EXTRA_LIBS}
)

# Install target
install(TARGETS aurora_3d_scanner DESTINATION bin)

# Print configuration info
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")