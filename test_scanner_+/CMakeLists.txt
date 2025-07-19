cmake_minimum_required(VERSION 3.18)
project(DepthScanner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
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

# Find Open3D - Using the same approach as your working test
set(Open3D_ROOT "C:/Users/31110/Downloads/Open3D" CACHE PATH "Path to Open3D installation")
find_package(Open3D REQUIRED HINTS ${Open3D_ROOT})

if(Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION}")
    message(STATUS "Open3D include dirs: ${Open3D_INCLUDE_DIRS}")
    message(STATUS "Open3D libraries: ${Open3D_LIBRARIES}")
else()
    message(FATAL_ERROR "Open3D not found. Please check Open3D_ROOT path.")
endif()

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

# Verify Deptrum SDK exists
if(NOT EXISTS ${DEPTRUM_SDK_ROOT})
    message(FATAL_ERROR "Deptrum SDK not found at: ${DEPTRUM_SDK_ROOT}")
endif()

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

# Add executable with custom name "scanner"
add_executable(scanner+ ${SOURCES})

# Include directories for target
target_include_directories(scanner+ PRIVATE 
    ${Open3D_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(scanner+ 
    ${OpenCV_LIBS}
    ${Open3D_LIBRARIES}
    ${DEPTRUM_STREAM_LIB}
)

# Add compile definitions
target_compile_definitions(scanner+ PRIVATE 
    ${Open3D_DEFINITIONS}
)

# Windows-specific linking
if(WIN32)
    target_link_libraries(scanner+ 
        ws2_32
        winmm
        setupapi
    )
    
    # Set custom output directory to your desired location
    set(CUSTOM_OUTPUT_DIR "C:/Users/31110/Downloads/work/yolo_test/build/Release")
    
    # Create the directory if it doesn't exist
    file(MAKE_DIRECTORY ${CUSTOM_OUTPUT_DIR})
    
    # Set output directories
    set_target_properties(scanner+ PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CUSTOM_OUTPUT_DIR}
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CUSTOM_OUTPUT_DIR}
        RUNTIME_OUTPUT_DIRECTORY ${CUSTOM_OUTPUT_DIR}
    )
    
    # Copy required DLLs to custom output directory
    
    # Copy Deptrum DLLs
    if(EXISTS "${DEPTRUM_SDK_ROOT}/bin")
        file(GLOB DEPTRUM_DLLS "${DEPTRUM_SDK_ROOT}/bin/*.dll")
        foreach(DLL ${DEPTRUM_DLLS})
            add_custom_command(TARGET scanner+ POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${DLL}
                ${CUSTOM_OUTPUT_DIR}
            )
        endforeach()
        message(STATUS "Will copy Deptrum DLLs to: ${CUSTOM_OUTPUT_DIR}")
    endif()
    
    # Copy Open3D DLLs
    if(EXISTS "${Open3D_ROOT}/bin")
        file(GLOB OPEN3D_DLLS "${Open3D_ROOT}/bin/*.dll")
        foreach(DLL ${OPEN3D_DLLS})
            add_custom_command(TARGET scanner+ POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${DLL}
                ${CUSTOM_OUTPUT_DIR}
            )
        endforeach()
        message(STATUS "Will copy Open3D DLLs to: ${CUSTOM_OUTPUT_DIR}")
    endif()
    
    # Copy OpenCV DLLs (if they exist)
    get_filename_component(OPENCV_BIN_DIR "${OpenCV_DIR}/../../bin" ABSOLUTE)
    if(EXISTS "${OPENCV_BIN_DIR}")
        file(GLOB OPENCV_DLLS "${OPENCV_BIN_DIR}/*.dll")
        foreach(DLL ${OPENCV_DLLS})
            add_custom_command(TARGET scanner+ POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${DLL}
                ${CUSTOM_OUTPUT_DIR}
            )
        endforeach()
        message(STATUS "Will copy OpenCV DLLs to: ${CUSTOM_OUTPUT_DIR}")
    endif()
    
    # Copy YOLO model file if it exists in the source directory
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/yolov8n.onnx")
        add_custom_command(TARGET scanner+ POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/yolov8n.onnx"
            ${CUSTOM_OUTPUT_DIR}
        )
        message(STATUS "Will copy YOLO model to: ${CUSTOM_OUTPUT_DIR}")
    endif()
endif()

# Print build information
message(STATUS "=================== Scanner Build Configuration ===================")
message(STATUS "Executable name: scanner")
message(STATUS "Output directory: ${CUSTOM_OUTPUT_DIR}")
message(STATUS "CMake version: ${CMAKE_VERSION}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "")
message(STATUS "OpenCV:")
message(STATUS "    Version: ${OpenCV_VERSION}")
message(STATUS "    Include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    Libraries: ${OpenCV_LIBS}")
message(STATUS "")
message(STATUS "Open3D:")
message(STATUS "    Version: ${Open3D_VERSION}")
message(STATUS "    Root: ${Open3D_ROOT}")
message(STATUS "    Include dirs: ${Open3D_INCLUDE_DIRS}")
message(STATUS "    Libraries: ${Open3D_LIBRARIES}")
message(STATUS "")
message(STATUS "Deptrum SDK:")
message(STATUS "    Root: ${DEPTRUM_SDK_ROOT}")
message(STATUS "    Library: ${DEPTRUM_STREAM_LIB}")
message(STATUS "")
message(STATUS "CUDA support: ${CUDA_FOUND}")
message(STATUS "=================================================================")

# Installation to custom directory
install(TARGETS scanner+ DESTINATION "${CUSTOM_OUTPUT_DIR}")