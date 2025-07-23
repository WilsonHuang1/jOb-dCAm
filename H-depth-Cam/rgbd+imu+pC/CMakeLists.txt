cmake_minimum_required(VERSION 3.12)
project(rgbd_imu_pointcloud_viewer)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Find MV3D RGBD SDK
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
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/win64")
        else()
            # 32-bit
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/win32")
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
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/linux64")
            set(MV3D_RGBD_LIBS Mv3dRgbd pthread)
        else()
            # 32-bit
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/linux32")
            set(MV3D_RGBD_LIBS Mv3dRgbd pthread)
        endif()
    else()
        message(WARNING "MV3D RGBD SDK not found. Please install the SDK.")
        set(MV3D_RGBD_LIBS Mv3dRgbd pthread)
    endif()
endif()

# REMOVED: This line was overwriting the correct MV3D_RGBD_LIBS
# set(MV3D_RGBD_LIBS ${MV3D_RGBD_LIBRARY})

# Set custom paths for OpenCV and Open3D
set(OpenCV_DIR "C:/Users/31110/Downloads/open_Stuff/opencv/build")
set(Open3D_DIR "C:/Users/31110/Downloads/open_Stuff/Open3D")

# Find OpenCV
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)
if(OpenCV_FOUND)
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libs: ${OpenCV_LIBS}")
    set(OPENCV_LIBS ${OpenCV_LIBS})
else()
    message(FATAL_ERROR "OpenCV not found at ${OpenCV_DIR}!")
endif()

# Find Open3D
find_package(Open3D REQUIRED PATHS ${Open3D_DIR} NO_DEFAULT_PATH)
if(Open3D_FOUND)
    message(STATUS "Open3D version: ${Open3D_VERSION}")
    message(STATUS "Open3D include dirs: ${Open3D_INCLUDE_DIRS}")
    message(STATUS "Open3D libs: ${Open3D_LIBRARIES}")
    set(OPEN3D_LIBS ${Open3D_LIBRARIES})
else()
    # Try alternative Open3D detection methods
    find_path(Open3D_INCLUDE_DIR
        NAMES Open3D/Open3D.h
        PATHS ${Open3D_DIR}/include
              ${Open3D_DIR}/build/include
              ${Open3D_DIR}/install/include
        NO_DEFAULT_PATH
    )
    
    find_library(Open3D_LIBRARY
        NAMES Open3D
        PATHS ${Open3D_DIR}/lib
              ${Open3D_DIR}/build/lib
              ${Open3D_DIR}/install/lib
              ${Open3D_DIR}/build/lib/Release
              ${Open3D_DIR}/build/lib/Debug
        NO_DEFAULT_PATH
    )
    
    if(Open3D_INCLUDE_DIR AND Open3D_LIBRARY)
        message(STATUS "Open3D found manually")
        message(STATUS "Open3D include dir: ${Open3D_INCLUDE_DIR}")
        message(STATUS "Open3D library: ${Open3D_LIBRARY}")
        set(OPEN3D_INCLUDE_DIRS ${Open3D_INCLUDE_DIR})
        set(OPEN3D_LIBS ${Open3D_LIBRARY})
        set(Open3D_FOUND TRUE)
    else()
        message(FATAL_ERROR "Open3D not found at ${Open3D_DIR}! Please check the path or build Open3D.")
    endif()
endif()

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${OPEN3D_INCLUDE_DIRS}
)

# Include MV3D SDK headers if found
if(MV3D_RGBD_SDK_ROOT)
    include_directories("${MV3D_RGBD_SDK_ROOT}/Includes")
endif()

# Compiler-specific options
if(MSVC)
    add_compile_options(/W3)
    # Set runtime library
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    # Disable specific warnings for cleaner output
    add_compile_options(/wd4267)  # Disable size_t conversion warnings
else()
    add_compile_options(-Wall -Wextra)
endif()

# Add executable
add_executable(rgbd_imu_pointcloud_viewer
    rgbd_imu_viewer.cpp
)

# Link libraries
target_link_libraries(rgbd_imu_pointcloud_viewer
    ${MV3D_RGBD_LIBS}
    ${OPENCV_LIBS}
)

# Open3D specific settings
if(Open3D_FOUND)
    # Link Open3D - check if target exists first
    if(TARGET Open3D::Open3D)
        target_link_libraries(rgbd_imu_pointcloud_viewer Open3D::Open3D)
        
        # Copy Open3D DLLs on Windows
        if(WIN32)
            add_custom_command(TARGET rgbd_imu_pointcloud_viewer POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:Open3D::Open3D>
                $<TARGET_FILE_DIR:rgbd_imu_pointcloud_viewer>
            )
        endif()
    else()
        # Manual linking if target doesn't exist
        target_link_libraries(rgbd_imu_pointcloud_viewer ${OPEN3D_LIBS})
        target_include_directories(rgbd_imu_pointcloud_viewer PRIVATE ${OPEN3D_INCLUDE_DIRS})
    endif()
endif()

# Compiler-specific options
if(MSVC)
    set_property(TARGET rgbd_imu_pointcloud_viewer PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Install target
install(TARGETS rgbd_imu_pointcloud_viewer DESTINATION bin)

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
if(Open3D_FOUND)
    message(STATUS "Open3D version: ${Open3D_VERSION}")
    message(STATUS "Open3D include dirs: ${OPEN3D_INCLUDE_DIRS}")
    message(STATUS "Open3D libs: ${OPEN3D_LIBS}")
else()
    message(STATUS "Open3D: Disabled")
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