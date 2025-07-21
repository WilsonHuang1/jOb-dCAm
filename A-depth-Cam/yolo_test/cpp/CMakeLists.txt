cmake_minimum_required(VERSION 3.12)
project(YOLOv8_Camera_Detection)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Windows-specific settings
if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    if(MSVC)
        add_compile_options(/W3)
        add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
        # Set runtime library
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
    endif()
endif()

# Set architecture explicitly for Windows
if(WIN32)
    set(CMAKE_GENERATOR_PLATFORM x64)
endif()

# Find OpenCV with specific components
set(OpenCV_DIR "C:/Program Files (x86)/opencv/build")
find_package(OpenCV REQUIRED 
    COMPONENTS
    core
    imgproc
    highgui
    videoio
    dnn
    objdetect
)

# Check if CUDA is available
find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "CUDA found - GPU acceleration available")
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 14)
else()
    message(STATUS "CUDA not found - using CPU only")
endif()

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add missing includes for chrono
if(MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

# Add executable
add_executable(yolov8_camera_detection main.cpp)

# Link libraries - be explicit about OpenCV modules
target_link_libraries(yolov8_camera_detection 
    ${OpenCV_LIBS}
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_videoio
    opencv_dnn
    opencv_objdetect
)

# Windows-specific linking
if(WIN32 AND MSVC)
    set_target_properties(yolov8_camera_detection PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release
    )
    
    # Copy OpenCV DLLs to output directory (if needed)
    if(OpenCV_FOUND)
        get_target_property(OPENCV_DLL_DIR opencv_core IMPORTED_LOCATION_RELEASE)
        get_filename_component(OPENCV_DLL_DIR ${OPENCV_DLL_DIR} DIRECTORY)
        message(STATUS "OpenCV DLL directory: ${OPENCV_DLL_DIR}")
    endif()
endif()

# Print detailed OpenCV information
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    library directories: ${OpenCV_LIBRARY_DIRS}")

# Installation
install(TARGETS yolov8_camera_detection DESTINATION bin)