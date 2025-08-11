cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

if(MSVC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W3 /O2")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3 /O2 /bigobj")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
endif()

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    # MSVC 2019 has excellent C++17 support
    message(STATUS "Using MSVC with C++17 support")
else()
    # For GCC/Clang
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
    if(COMPILER_SUPPORTS_CXX17)
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    else()
        message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
    endif()
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)

# Find Orbbec SDK
option(BUILD_WITH_ORBBEC "Build with Orbbec SDK support" ON)
if(BUILD_WITH_ORBBEC)
    # Set Orbbec SDK path - adjust this path to your Orbbec SDK installation
    set(ORBBEC_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../OrbbecSDK" CACHE PATH "Path to Orbbec SDK")
    
    # Try to find OrbbecSDK using the config file first
    set(OrbbecSDK_DIR "${ORBBEC_SDK_ROOT}/lib")
    find_package(OrbbecSDK QUIET)
    
    # Always do manual search to get proper library paths
    find_path(ORBBEC_INCLUDE_DIR
        NAMES libobsensor/ObSensor.hpp
        PATHS ${ORBBEC_SDK_ROOT}/include
        NO_DEFAULT_PATH
    )
    
    if(WIN32)
        find_library(ORBBEC_LIBRARY
            NAMES OrbbecSDK
            PATHS ${ORBBEC_SDK_ROOT}/lib
            NO_DEFAULT_PATH
        )
        set(ORBBEC_LIB_DIR "${ORBBEC_SDK_ROOT}/lib")
    else()
        find_library(ORBBEC_LIBRARY
            NAMES OrbbecSDK
            PATHS ${ORBBEC_SDK_ROOT}/lib
            NO_DEFAULT_PATH
        )
        set(ORBBEC_LIB_DIR "${ORBBEC_SDK_ROOT}/lib")
    endif()
    
    if(ORBBEC_INCLUDE_DIR AND ORBBEC_LIBRARY)
        set(ORBBEC_FOUND TRUE)
        message(STATUS "Found Orbbec SDK manually")
        message(STATUS "  Include: ${ORBBEC_INCLUDE_DIR}")
        message(STATUS "  Library: ${ORBBEC_LIBRARY}")
    else()
        set(ORBBEC_FOUND FALSE)
        message(WARNING "Orbbec SDK not found. Please check ORBBEC_SDK_ROOT path: ${ORBBEC_SDK_ROOT}")
        message(STATUS "  Searched include: ${ORBBEC_SDK_ROOT}/include")
        message(STATUS "  Searched library: ${ORBBEC_SDK_ROOT}/lib")
    endif()
    
    if(ORBBEC_FOUND)
        add_definitions(-DWITH_ORBBEC)
        message(STATUS "Building with Orbbec SDK support")
        
        # Debug information
        message(STATUS "DEBUG: ORBBEC_FOUND = TRUE")
        message(STATUS "DEBUG: OrbbecSDK_FOUND = ${OrbbecSDK_FOUND}")
        message(STATUS "DEBUG: ORBBEC_SDK_ROOT = ${ORBBEC_SDK_ROOT}")
        message(STATUS "DEBUG: ORBBEC_INCLUDE_DIR = ${ORBBEC_INCLUDE_DIR}")
        message(STATUS "DEBUG: ORBBEC_LIBRARY = ${ORBBEC_LIBRARY}")
        if(TARGET OrbbecSDK::OrbbecSDK)
            message(STATUS "DEBUG: Target OrbbecSDK::OrbbecSDK exists = TRUE")
        else()
            message(STATUS "DEBUG: Target OrbbecSDK::OrbbecSDK exists = FALSE")
        endif()
        
        # Check if the include file actually exists
        set(TEST_INCLUDE_FILE "${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp")
        if(EXISTS ${TEST_INCLUDE_FILE})
            message(STATUS "DEBUG: ObSensor.hpp EXISTS at ${TEST_INCLUDE_FILE}")
        else()
            message(STATUS "DEBUG: ObSensor.hpp NOT FOUND at ${TEST_INCLUDE_FILE}")
        endif()
    else()
        message(WARNING "Orbbec SDK not found - Orbbec examples will not be built")
        message(STATUS "DEBUG: ORBBEC_FOUND = FALSE")
    endif()
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

# Add Orbbec include directories
if(ORBBEC_FOUND)
    if(OrbbecSDK_FOUND AND TARGET OrbbecSDK::OrbbecSDK)
        # Using target-based approach
        message(STATUS "Using OrbbecSDK target")
    else()
        include_directories(${ORBBEC_INCLUDE_DIR})
    endif()
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} STATIC
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h)

add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/Release/DBoW2.lib
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/bin/Release/g2o.lib
)

if(WIN32)
    target_link_libraries(${PROJECT_NAME} 
        C:/dev/vcpkg/installed/x64-windows/lib/boost_serialization-vc142-mt-x64-1_88.lib
        C:/dev/vcpkg/installed/x64-windows/lib/boost_wserialization-vc142-mt-x64-1_88.lib
        C:/dev/vcpkg/installed/x64-windows/lib/libcrypto.lib
        C:/dev/vcpkg/installed/x64-windows/lib/libssl.lib
        crypt32
        ws2_32
    )
endif()

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    )
endif()

# Build examples

# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i
            Examples/RGB-D/rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
endif()

# Orbbec RGB-D examples
if(ORBBEC_FOUND)
    add_executable(rgbd_orbbec_gemini335
            Examples/RGB-D/rgbd_orbbec_gemini335.cc)
    target_link_libraries(rgbd_orbbec_gemini335 ${PROJECT_NAME})
    
    # Add include directories explicitly
    if(ORBBEC_INCLUDE_DIR)
        target_include_directories(rgbd_orbbec_gemini335 PRIVATE ${ORBBEC_INCLUDE_DIR})
        message(STATUS "DEBUG: Added private include ${ORBBEC_INCLUDE_DIR} to rgbd_orbbec_gemini335")
    endif()
    if(ORBBEC_SDK_ROOT)
        target_include_directories(rgbd_orbbec_gemini335 PRIVATE ${ORBBEC_SDK_ROOT}/include)
        message(STATUS "DEBUG: Added private include ${ORBBEC_SDK_ROOT}/include to rgbd_orbbec_gemini335")
    endif()
    
    # Link Orbbec SDK - always use manual library path since target doesn't work properly
    target_link_libraries(rgbd_orbbec_gemini335 ${ORBBEC_LIBRARY})
    message(STATUS "DEBUG: Linked ${ORBBEC_LIBRARY} to rgbd_orbbec_gemini335")
    
    # Debug: Print target include directories
    get_target_property(INCLUDE_DIRS rgbd_orbbec_gemini335 INCLUDE_DIRECTORIES)
    message(STATUS "DEBUG: rgbd_orbbec_gemini335 include dirs = ${INCLUDE_DIRS}")
    
    # For Windows, copy necessary DLLs
    if(WIN32)
        set(ORBBEC_BIN_DIR "${ORBBEC_SDK_ROOT}/bin")
        if(EXISTS ${ORBBEC_BIN_DIR})
            file(GLOB ORBBEC_DLLS "${ORBBEC_BIN_DIR}/*.dll")
            foreach(DLL ${ORBBEC_DLLS})
                add_custom_command(TARGET rgbd_orbbec_gemini335 POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${DLL} $<TARGET_FILE_DIR:rgbd_orbbec_gemini335>)
            endforeach()
            message(STATUS "DEBUG: Will copy DLLs from ${ORBBEC_BIN_DIR}")
        else()
            message(WARNING "Orbbec bin directory not found: ${ORBBEC_BIN_DIR}")
        endif()
        
        # Set Visual Studio working directory
        if(MSVC)
            set_target_properties(rgbd_orbbec_gemini335 PROPERTIES 
                VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
        endif()
    endif()
endif()

# RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i
            Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})
endif()

# Orbbec RGB-D Inertial examples
if(ORBBEC_FOUND)
    add_executable(rgbd_inertial_orbbec_gemini335
            Examples/RGB-D-Inertial/rgbd_inertial_orbbec_gemini335.cc)
    target_link_libraries(rgbd_inertial_orbbec_gemini335 ${PROJECT_NAME})
    
    # Add include directories explicitly
    if(ORBBEC_INCLUDE_DIR)
        target_include_directories(rgbd_inertial_orbbec_gemini335 PRIVATE ${ORBBEC_INCLUDE_DIR})
        message(STATUS "DEBUG: Added private include ${ORBBEC_INCLUDE_DIR} to rgbd_inertial_orbbec_gemini335")
    endif()
    if(ORBBEC_SDK_ROOT)
        target_include_directories(rgbd_inertial_orbbec_gemini335 PRIVATE ${ORBBEC_SDK_ROOT}/include)
        message(STATUS "DEBUG: Added private include ${ORBBEC_SDK_ROOT}/include to rgbd_inertial_orbbec_gemini335")
    endif()
    
    # Link Orbbec SDK - always use manual library path since target doesn't work properly
    target_link_libraries(rgbd_inertial_orbbec_gemini335 ${ORBBEC_LIBRARY})
    message(STATUS "DEBUG: Linked ${ORBBEC_LIBRARY} to rgbd_inertial_orbbec_gemini335")
    
    # Debug: Print target include directories
    get_target_property(INCLUDE_DIRS rgbd_inertial_orbbec_gemini335 INCLUDE_DIRECTORIES)
    message(STATUS "DEBUG: rgbd_inertial_orbbec_gemini335 include dirs = ${INCLUDE_DIRS}")
    
    # For Windows, copy necessary DLLs
    if(WIN32)
        set(ORBBEC_BIN_DIR "${ORBBEC_SDK_ROOT}/bin")
        if(EXISTS ${ORBBEC_BIN_DIR})
            file(GLOB ORBBEC_DLLS "${ORBBEC_BIN_DIR}/*.dll")
            foreach(DLL ${ORBBEC_DLLS})
                add_custom_command(TARGET rgbd_inertial_orbbec_gemini335 POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${DLL} $<TARGET_FILE_DIR:rgbd_inertial_orbbec_gemini335>)
            endforeach()
            message(STATUS "DEBUG: Will copy DLLs from ${ORBBEC_BIN_DIR}")
        else()
            message(WARNING "Orbbec bin directory not found: ${ORBBEC_BIN_DIR}")
        endif()
        
        # Set Visual Studio working directory
        if(MSVC)
            set_target_properties(rgbd_inertial_orbbec_gemini335 PROPERTIES 
                VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
        endif()
    endif()
endif()

#Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
        Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
        Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})

add_executable(stereo_tum_vi
        Examples/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(stereo_realsense_t265
            Examples/Stereo/stereo_realsense_t265.cc)
    target_link_libraries(stereo_realsense_t265 ${PROJECT_NAME})

    add_executable(stereo_realsense_D435i
            Examples/Stereo/stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
endif()

#Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
        Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
        Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
        Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_tum_vi
        Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(mono_realsense_t265
            Examples/Monocular/mono_realsense_t265.cc)
    target_link_libraries(mono_realsense_t265 ${PROJECT_NAME})

    add_executable(mono_realsense_D435i
            Examples/Monocular/mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
endif()

#Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc
        Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi
        Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(mono_inertial_realsense_t265
            Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
    target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})

    add_executable(mono_inertial_realsense_D435i
            Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
endif()

#Stereo Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

add_executable(stereo_inertial_euroc
        Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

add_executable(stereo_inertial_tum_vi
        Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(stereo_inertial_realsense_t265
            Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
    target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})

    add_executable(stereo_inertial_realsense_D435i
            Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
    target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)
if(realsense2_FOUND)
    add_executable(recorder_realsense_D435i
            Examples/Calibration/recorder_realsense_D435i.cc)
    target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})

    add_executable(recorder_realsense_T265
            Examples/Calibration/recorder_realsense_T265.cc)
    target_link_libraries(recorder_realsense_T265 ${PROJECT_NAME})
endif()

# Print configuration summary
message(STATUS "")
message(STATUS "ORB-SLAM3 Configuration Summary:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenCV version: ${OpenCV_VERSION}")
if(ORBBEC_FOUND)
    message(STATUS "  Orbbec SDK: Found")
    message(STATUS "    Path: ${ORBBEC_SDK_ROOT}")
else()
    message(STATUS "  Orbbec SDK: Not found")
endif()
if(realsense2_FOUND)
    message(STATUS "  RealSense SDK: Found")
else()
    message(STATUS "  RealSense SDK: Not found")
endif()
message(STATUS "")