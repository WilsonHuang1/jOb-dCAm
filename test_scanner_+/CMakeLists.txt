cmake_minimum_required(VERSION 3.16)
project(enhanced_slam_scanner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler-specific options for Windows
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
    # Disable specific warnings for Windows
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-D_ENFORCE_MATCHING_ALLOCATORS=0)
    add_definitions(-DNOMINMAX)
    # Disable specific warning numbers that cause issues
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4819 /wd4267 /wd4996")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

# Find required packages with your specific paths
set(OpenCV_DIR "C:/Program Files (x86)/opencv/build")
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION}")
    message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
else()
    message(FATAL_ERROR "OpenCV not found. Please check OpenCV_DIR path.")
endif()

# Find Open3D with your working configuration
set(Open3D_ROOT "C:/Users/31110/Downloads/Open3D" CACHE PATH "Path to Open3D installation")
find_package(Open3D REQUIRED HINTS ${Open3D_ROOT})

if(Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION}")
    message(STATUS "Open3D include dirs: ${Open3D_INCLUDE_DIRS}")
    message(STATUS "Open3D libraries: ${Open3D_LIBRARIES}")
else()
    message(FATAL_ERROR "Open3D not found. Please check Open3D_ROOT path.")
endif()

# Find Eigen3
set(Eigen3_ROOT "C:/Users/31110/Downloads/Eigen3")
find_package(Eigen3 REQUIRED)

if(Eigen3_FOUND)
    message(STATUS "Found Eigen3 ${Eigen3_VERSION}")
    message(STATUS "Eigen3 include dirs: ${EIGEN3_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Eigen3 not found. Please check Eigen3_ROOT path.")
endif()

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Open3D_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# Add executable - minimal version without SDK dependencies
add_executable(enhanced_slam_scanner
    main.cpp
    enhanced_3d_scanner.cpp
    enhanced_3d_scanner.h
)

# Link libraries
target_link_libraries(enhanced_slam_scanner
    ${OpenCV_LIBS}
    ${Open3D_LIBRARIES}
)

# Set target properties
set_target_properties(enhanced_slam_scanner PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Copy DLLs to output directory (Windows specific)
if(WIN32)
    # Copy OpenCV DLLs
    file(GLOB OPENCV_DLLS "${OpenCV_DIR}/x64/vc*/bin/*.dll")
    if(OPENCV_DLLS)
        foreach(dll ${OPENCV_DLLS})
            add_custom_command(TARGET enhanced_slam_scanner POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${dll}" $<TARGET_FILE_DIR:enhanced_slam_scanner>)
        endforeach()
        message(STATUS "Will copy OpenCV DLLs from: ${OpenCV_DIR}/x64/vc*/bin/")
    else()
        message(WARNING "No OpenCV DLLs found to copy")
    endif()
    
    # Copy Open3D DLLs
    file(GLOB OPEN3D_DLLS "${Open3D_ROOT}/bin/*.dll")
    if(OPEN3D_DLLS)
        foreach(dll ${OPEN3D_DLLS})
            add_custom_command(TARGET enhanced_slam_scanner POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${dll}" $<TARGET_FILE_DIR:enhanced_slam_scanner>)
        endforeach()
        message(STATUS "Will copy Open3D DLLs from: ${Open3D_ROOT}/bin/")
    else()
        message(WARNING "No Open3D DLLs found to copy")
    endif()
endif()

# Installation
install(TARGETS enhanced_slam_scanner
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Create output directories for scanner data
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/scanner_output)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/scanner_output/maps)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/scanner_output/keyframes)

# Print configuration summary
message(STATUS "========== Enhanced SLAM Scanner Build Configuration ==========")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Open3D version: ${Open3D_VERSION}")
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Mode: MOCK MODE (no real SDK dependencies)")
message(STATUS "===============================================================")

# Build instructions
message(STATUS "")
message(STATUS "========== BUILD INSTRUCTIONS ==========")
message(STATUS "1. mkdir build")
message(STATUS "2. cd build") 
message(STATUS "3. cmake .. -G \"Visual Studio 16 2019\" -A x64")
message(STATUS "4. cmake --build . --config Release")
message(STATUS "5. cd Release")
message(STATUS "6. ./enhanced_slam_scanner.exe")
message(STATUS "===========================================")