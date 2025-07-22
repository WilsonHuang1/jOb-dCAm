cmake_minimum_required(VERSION 3.10)

project(IMUTest)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform-specific settings
if(WIN32)
    # Windows-specific settings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /utf-8")
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-DUNICODE -D_UNICODE)
    
    # Try to find MV3D RGBD SDK in common locations
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
        
        # Linux library paths
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            # 64-bit
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/64")
            set(MV3D_RGBD_LIBS Mv3dRgbd pthread)
        else()
            # 32-bit
            link_directories("${MV3D_RGBD_SDK_ROOT}/Libraries/32")
            set(MV3D_RGBD_LIBS Mv3dRgbd pthread)
        endif()
    else()
        message(WARNING "MV3D RGBD SDK not found. Please install the SDK.")
        set(MV3D_RGBD_LIBS Mv3dRgbd pthread)
    endif()
endif()

# Add executable
add_executable(imu_test imu_test.cpp)

# Link libraries
target_link_libraries(imu_test ${MV3D_RGBD_LIBS})

# Compiler-specific options
if(MSVC)
    set_property(TARGET imu_test PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Install target
install(TARGETS imu_test DESTINATION bin)

# Print build information
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
if(MV3D_RGBD_SDK_ROOT)
    message(STATUS "MV3D RGBD SDK root: ${MV3D_RGBD_SDK_ROOT}")
endif()
message(STATUS "MV3D RGBD libraries: ${MV3D_RGBD_LIBS}")