cmake_minimum_required(VERSION 3.12)
project(YOLODepthIntegration)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Windows-specific settings
if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    if(MSVC)
        add_compile_options(/W3)
        add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
    endif()
    set(CMAKE_GENERATOR_PLATFORM x64)
endif()

# Find OpenCV
set(OpenCV_DIR "C:/Program Files (x86)/opencv/build")
find_package(OpenCV REQUIRED 
    COMPONENTS
    core
    imgproc
    highgui
    videoio
    dnn
)

# Check if CUDA is available
find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "CUDA found - GPU acceleration available")
else()
    message(STATUS "CUDA not found - using CPU only")
endif()

# Deptrum SDK paths
set(DEPTRUM_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/deptrum-stream-aurora900-windows-amd64-v1.1.6")
set(DEPTRUM_INCLUDE_DIR "${DEPTRUM_SDK_ROOT}/include")
set(DEPTRUM_LIB_DIR "${DEPTRUM_SDK_ROOT}/lib")
set(DEPTRUM_SAMPLES_DIR "${DEPTRUM_SDK_ROOT}/samples/src")

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${DEPTRUM_INCLUDE_DIR})
include_directories(${DEPTRUM_SAMPLES_DIR})

# Find Deptrum library
file(GLOB DEPTRUM_LIBS "${DEPTRUM_LIB_DIR}/*.lib")
if(NOT DEPTRUM_LIBS)
    message(FATAL_ERROR "No Deptrum libraries found in ${DEPTRUM_LIB_DIR}")
endif()

# Use the first library found (should be deptrum_stream_aurora900.lib)
list(GET DEPTRUM_LIBS 0 DEPTRUM_STREAM_LIB)
message(STATUS "Using Deptrum library: ${DEPTRUM_STREAM_LIB}")

# Define device type for Aurora cameras
add_compile_definitions(DEVICE_TYPE_AURORA900)

# Source files
set(SOURCES
    main.cpp
    ${DEPTRUM_SAMPLES_DIR}/sample_helper.cc
)

# Add executable
add_executable(yolo_depth_integration ${SOURCES})

# Link libraries
target_link_libraries(yolo_depth_integration 
    ${OpenCV_LIBS}
    ${DEPTRUM_STREAM_LIB}
)

# Windows-specific linking
if(WIN32)
    target_link_libraries(yolo_depth_integration 
        ws2_32
        setupapi
    )
    
    set_target_properties(yolo_depth_integration PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release
    )
    
    # Copy required DLLs to output directory
    if(EXISTS "${DEPTRUM_LIB_DIR}")
        file(GLOB DEPTRUM_DLLS "${DEPTRUM_LIB_DIR}/*.dll")
        foreach(DLL ${DEPTRUM_DLLS})
            add_custom_command(TARGET yolo_depth_integration POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${DLL}
                $<TARGET_FILE_DIR:yolo_depth_integration>
            )
        endforeach()
    endif()
endif()

# Print build information
message(STATUS "Build configuration:")
message(STATUS "    OpenCV version: ${OpenCV_VERSION}")
message(STATUS "    OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "    Deptrum SDK: ${DEPTRUM_SDK_ROOT}")
message(STATUS "    Deptrum library: ${DEPTRUM_STREAM_LIB}")

# Installation
install(TARGETS yolo_depth_integration DESTINATION bin)