cmake_minimum_required(VERSION 3.16)
project(aurora_camera_parameters)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find Aurora SDK using the system-wide installation
find_package(AuroraSDK REQUIRED)

if(AuroraSDK_FOUND)
    message(STATUS "Aurora SDK found")
    message(STATUS "  Version: ${AuroraSDK_VERSION}")
    message(STATUS "  Include dirs: ${AuroraSDK_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${AuroraSDK_LIBRARIES}")
else()
    message(FATAL_ERROR "Aurora SDK not found! Please install it system-wide first.")
endif()

# Set compiler flags
if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    endif()
endif()

# Create executable for camera parameter extractor
add_executable(aurora_camera_params
    aurora_camera_params.cpp
)

# Link against Aurora SDK
target_link_libraries(aurora_camera_params 
    AuroraSDK::AuroraSDK
)

# Set output directory
set_target_properties(aurora_camera_params PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Optional: Install the executable
install(TARGETS aurora_camera_params 
    RUNTIME DESTINATION bin
)

# Print build information
message(STATUS "")
message(STATUS "Build Configuration:")
message(STATUS "  Project: ${PROJECT_NAME}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Output directory: ${CMAKE_BINARY_DIR}/bin")
message(STATUS "")
message(STATUS "To build:")
message(STATUS "  mkdir build && cd build")
message(STATUS "  cmake ..")
message(STATUS "  make")
message(STATUS "")
message(STATUS "To run:")
message(STATUS "  ./bin/aurora_camera_params")
message(STATUS "")