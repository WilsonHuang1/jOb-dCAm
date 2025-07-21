cmake_minimum_required(VERSION 3.10)

project(RGBDIRExtractor)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to enable OpenCV support (required for display)
option(USE_OPENCV "Enable OpenCV support for real-time display" ON)

# Platform-specific settings
if(WIN32)
    # Windows-specific settings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    
    # Try to find MV3D RGBD SDK in common locations including HiViewer
    set(MV3D_RGBD_SDK_SEARCH_PATHS
        "C:/Program Files (x86)/HiViewer/Development"
        "C:/Program Files/HiViewer/Development"
        "C:/Program Files/MVS/Development"
        "C:/Program Files (x86)/MVS/Development"
        "C:/MVS/Development"
        "${CMAKE_CURRENT_SOURCE_DIR}/../"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../"
    )
    
    foreach(path ${MV3D_RGBD_SDK_SEARCH_PATHS})
        if(EXISTS "${path}/Includes" OR EXISTS "${path}/Libraries/win64")
            set(MV3D_RGBD_SDK_ROOT "${path}")
            message(STATUS "Found SDK at: ${path}")
            break()
        endif()
    endforeach()
    
    if(MV3D_RGBD_SDK_ROOT)
        if(EXISTS "${MV3D_RGBD_SDK_ROOT}/Includes")
            include_directories("${MV3D_RGBD_SDK_ROOT}/Includes")
        endif()
        if(EXISTS "${MV3D_RGBD_SDK_ROOT}/Libraries/win64")
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/win64")
        endif()
        
        # Try to find the correct library name
        find_library(MV3D_RGBD_LIB
            NAMES Mv3dRgbd MV3D_RGBD
            PATHS "${MV3D_RGBD_SDK_ROOT}/Libraries/win64"
            NO_DEFAULT_PATH
        )
        
        if(MV3D_RGBD_LIB)
            set(MV3D_RGBD_LIBS ${MV3D_RGBD_LIB})
            message(STATUS "Found library: ${MV3D_RGBD_LIB}")
        else()
            # Fallback to library name
            set(MV3D_RGBD_LIBS Mv3dRgbd)
            message(STATUS "Using library name: Mv3dRgbd")
        endif()
    else()
        # Use relative path to common headers from New_Cam_Sample
        include_directories("../New_Cam_Sample/C/common/include")
        include_directories("../../New_Cam_Sample/C/common/include")
        
        # Try to find the library in common locations
        find_library(MV3D_RGBD_LIB
            NAMES Mv3dRgbd MV3D_RGBD
            PATHS 
                "C:/Program Files (x86)/HiViewer/Development/Libraries/win64"
                "C:/Program Files/HiViewer/Development/Libraries/win64"
                "C:/Program Files/MVS/Development/Libraries/win64"
                "C:/Program Files (x86)/MVS/Development/Libraries/win64"
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
        link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/64")
    endif()
    
    # Try to find the correct library name for Linux
    find_library(MV3D_RGBD_LIB
        NAMES Mv3dRgbd MV3D_RGBD libMv3dRgbd
        PATHS "${MV3D_RGBD_SDK_ROOT}/Libraries/64" "/usr/local/lib" "/usr/lib"
    )
    
    if(MV3D_RGBD_LIB)
        set(MV3D_RGBD_LIBS ${MV3D_RGBD_LIB} pthread)
    else()
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
add_executable(rgbd_ir_extractor
    rgbd_ir_extractor.cpp
)

# Link libraries
target_link_libraries(rgbd_ir_extractor
    ${MV3D_RGBD_LIBS}
    ${OPENCV_LIBS}
)

# Compiler-specific options
if(MSVC)
    set_property(TARGET rgbd_ir_extractor PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Install target
install(TARGETS rgbd_ir_extractor DESTINATION bin)

# Print build information
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
if(MV3D_RGBD_SDK_ROOT)
    message(STATUS "MV3D RGBD SDK root: ${MV3D_RGBD_SDK_ROOT}")
endif()
message(STATUS "MV3D RGBD libraries: ${MV3D_RGBD_LIBS}")
if(USE_OPENCV AND OpenCV_FOUND)
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