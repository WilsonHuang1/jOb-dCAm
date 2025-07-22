cmake_minimum_required(VERSION 3.10)
project(RGBDIMUViewer)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Platform-specific configurations
if(WIN32)
    # Windows settings - Updated paths for HiViewer SDK
    set(MV3D_RGBD_SDK_SEARCH_PATHS
        "C:/Program Files (x86)/HiViewer/Development"
        "C:/Program Files/HiViewer/Development"
        "C:/Program Files (x86)/MVS/Development"
        "C:/Program Files/MVS/Development"
        "C:/HiViewer/Development"
        "C:/MVS/Development"
        "${CMAKE_CURRENT_SOURCE_DIR}/../HiViewer/Development"
        "${CMAKE_CURRENT_SOURCE_DIR}/../MVS/Development"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../HiViewer/Development"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../MVS/Development"
    )
    
    foreach(path ${MV3D_RGBD_SDK_SEARCH_PATHS})
        if(EXISTS "${path}/Includes")
            set(MV3D_RGBD_SDK_ROOT "${path}")
            break()
        endif()
    endforeach()
    
    if(MV3D_RGBD_SDK_ROOT)
        include_directories("${MV3D_RGBD_SDK_ROOT}/Includes")
        message(STATUS "Found MV3D RGBD SDK: ${MV3D_RGBD_SDK_ROOT}")
        
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            # 64-bit
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/64")
        else()
            # 32-bit
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/32")
        endif()
        
        # Try to find the correct library name for Windows
        find_library(MV3D_RGBD_LIB
            NAMES Mv3dRgbd MV3D_RGBD libMv3dRgbd Mv3dRgbdApi
            PATHS "${MV3D_RGBD_SDK_ROOT}/Libraries/win64"
                  "${MV3D_RGBD_SDK_ROOT}/Libraries/win32"
                  "../lib"
                  "../../lib"
        )
        
        if(MV3D_RGBD_LIB)
            set(MV3D_RGBD_LIBS ${MV3D_RGBD_LIB})
            message(STATUS "Found library: ${MV3D_RGBD_LIB}")
        else()
            message(STATUS "Library not found, using default name: Mv3dRgbd")
            set(MV3D_RGBD_LIBS Mv3dRgbd)
        endif()
    else()
        message(WARNING "MV3D RGBD SDK not found. Please check the installation path.")
        message(STATUS "Expected location: C:/Program Files (x86)/HiViewer/Development")
        set(MV3D_RGBD_LIBS Mv3dRgbd)
    endif()
    
else()
    # Linux/Unix settings
    set(MV3D_RGBD_SDK_SEARCH_PATHS
        "/opt/MVS/Development"
        "/opt/HiViewer/Development"
        "/usr/local/MVS/Development"
    )
    
    foreach(path ${MV3D_RGBD_SDK_SEARCH_PATHS})
        if(EXISTS "${path}/Includes")
            set(MV3D_RGBD_SDK_ROOT "${path}")
            break()
        endif()
    endforeach()
    
    if(MV3D_RGBD_SDK_ROOT)
        include_directories("${MV3D_RGBD_SDK_ROOT}/Includes")
        
        # Linux library paths
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            # 64-bit
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/win64")
            set(MV3D_RGBD_LIBS Mv3dRgbd pthread)
        else()
            # 32-bit
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/win32")
            set(MV3D_RGBD_LIBS Mv3dRgbd pthread)
        endif()
    else()
        message(WARNING "MV3D RGBD SDK not found. Please install the SDK.")
        set(MV3D_RGBD_LIBS Mv3dRgbd pthread)
    endif()
endif()

# Add include directories for the common headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories("../common/include")
include_directories("../../common/include")

# Find OpenCV (required for display)
# Set OpenCV search paths for common Windows installations
if(WIN32)
    set(OpenCV_SEARCH_PATHS
        "C:/Program Files (x86)/opencv/build"
        "C:/Program Files/opencv/build"
        "C:/opencv/build"
        "C:/tools/opencv/build"
        "${CMAKE_CURRENT_SOURCE_DIR}/../opencv/build"
    )
    
    # Try to find OpenCV in common locations
    foreach(path ${OpenCV_SEARCH_PATHS})
        if(EXISTS "${path}")
            set(OpenCV_DIR "${path}")
            message(STATUS "Setting OpenCV_DIR to: ${path}")
            break()
        endif()
    endforeach()
endif()

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    add_definitions(-DUSE_OPENCV)
    include_directories(${OpenCV_INCLUDE_DIRS})
    set(OPENCV_LIBS ${OpenCV_LIBS})
    message(STATUS "OpenCV found - version ${OpenCV_VERSION}")
    message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
else()
    message(FATAL_ERROR "OpenCV is required for real-time display but was not found!")
endif()

# Add executable
add_executable(rgbd_imu_viewer
    rgbd_imu_viewer.cpp
)

# Link libraries
target_link_libraries(rgbd_imu_viewer
    ${MV3D_RGBD_LIBS}
    ${OPENCV_LIBS}
)

# Compiler-specific options
if(MSVC)
    set_property(TARGET rgbd_imu_viewer PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Install target
install(TARGETS rgbd_imu_viewer DESTINATION bin)

# Print build information
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
if(MV3D_RGBD_SDK_ROOT)
    message(STATUS "MV3D RGBD SDK root: ${MV3D_RGBD_SDK_ROOT}")
endif()
message(STATUS "MV3D RGBD libraries: ${MV3D_RGBD_LIBS}")
if(OpenCV_FOUND)
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libs: ${OpenCV_LIBS}")
else()
    message(STATUS "OpenCV: Disabled")
endif()

# Create output directories at build time
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/extracted_data)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/extracted_data/depth)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/extracted_data/color)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/extracted_data/ir_left)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/extracted_data/ir_right)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/extracted_data/mono)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/extracted_data/rgbd)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/extracted_data/unknown)