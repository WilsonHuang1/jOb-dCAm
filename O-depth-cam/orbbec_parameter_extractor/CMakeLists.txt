# Copyright (c) Orbbec Inc. All Rights Reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.5)
project(orbbec_parameter_extractor)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OrbbecSDK
# Option 1: Try to find system-wide installation first
find_library(ORBBEC_LIBRARY
    NAMES OrbbecSDK libOrbbecSDK
    PATHS /usr/local/lib /usr/lib
    NO_DEFAULT_PATH
)

find_path(ORBBEC_INCLUDE_DIR
    NAMES libobsensor/ObSensor.hpp
    PATHS /usr/local/include /usr/include
    NO_DEFAULT_PATH
)

if(ORBBEC_LIBRARY AND ORBBEC_INCLUDE_DIR)
    message(STATUS "Found OrbbecSDK system-wide installation:")
    message(STATUS "  Library: ${ORBBEC_LIBRARY}")
    message(STATUS "  Include: ${ORBBEC_INCLUDE_DIR}")
    set(OrbbecSDK_FOUND TRUE)
    include_directories(${ORBBEC_INCLUDE_DIR})
else()
    # Option 2: Try CMake package finder
    find_package(OrbbecSDK QUIET)
    
    if(NOT OrbbecSDK_FOUND)
        # Option 3: Manual path setup (adjust these paths to your OrbbecSDK installation)
        # Try common installation paths
        set(ORBBEC_SDK_SEARCH_PATHS
            "/opt/OrbbecSDK_v2.4.8"
            "/usr/local/OrbbecSDK_v2.4.8"
            "${CMAKE_CURRENT_SOURCE_DIR}/../OrbbecSDK_v2.4.8"
            "$ENV{HOME}/OrbbecSDK_v2.4.8"
        )
        
        set(ORBBEC_SDK_ROOT "" CACHE PATH "Path to OrbbecSDK")
        
        # If no path specified, try to find it automatically
        if(NOT ORBBEC_SDK_ROOT OR NOT EXISTS ${ORBBEC_SDK_ROOT})
            foreach(SEARCH_PATH ${ORBBEC_SDK_SEARCH_PATHS})
                if(EXISTS ${SEARCH_PATH}/include/libobsensor/ObSensor.hpp)
                    set(ORBBEC_SDK_ROOT ${SEARCH_PATH})
                    message(STATUS "Found OrbbecSDK at: ${ORBBEC_SDK_ROOT}")
                    break()
                endif()
            endforeach()
        endif()
        
        if(NOT ORBBEC_SDK_ROOT OR NOT EXISTS ${ORBBEC_SDK_ROOT})
            message(FATAL_ERROR "OrbbecSDK not found. Please install system-wide or specify ORBBEC_SDK_ROOT path.")
        endif()
        
        # Include directories
        include_directories(${ORBBEC_SDK_ROOT}/include)
        
        # Determine the library directory based on platform
        if(WIN32)
            if(CMAKE_SIZEOF_VOID_P EQUAL 8)
                set(ORBBEC_LIB_DIR ${ORBBEC_SDK_ROOT}/lib/windows/x64)
            else()
                set(ORBBEC_LIB_DIR ${ORBBEC_SDK_ROOT}/lib/windows/x86)
            endif()
            set(ORBBEC_LIB_NAME OrbbecSDK)
        elseif(UNIX AND NOT APPLE)
            set(ORBBEC_LIB_DIR ${ORBBEC_SDK_ROOT}/lib)
            set(ORBBEC_LIB_NAME OrbbecSDK)
        elseif(APPLE)
            set(ORBBEC_LIB_DIR ${ORBBEC_SDK_ROOT}/lib/macos)
            set(ORBBEC_LIB_NAME OrbbecSDK)
        endif()
        
        # Find the library
        find_library(ORBBEC_LIBRARY
            NAMES ${ORBBEC_LIB_NAME}
            PATHS ${ORBBEC_LIB_DIR}
            NO_DEFAULT_PATH
        )
        
        if(NOT ORBBEC_LIBRARY)
            message(FATAL_ERROR "OrbbecSDK library not found. Please check ORBBEC_SDK_ROOT path: ${ORBBEC_SDK_ROOT}")
        endif()
        
        message(STATUS "Found OrbbecSDK library: ${ORBBEC_LIBRARY}")
        message(STATUS "Using OrbbecSDK include: ${ORBBEC_SDK_ROOT}/include")
    endif()
endif()

# Create the executable
add_executable(${PROJECT_NAME} 
    camera_imu_parameter_extractor.cpp
)

# Set properties
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

# Link libraries
if(OrbbecSDK_FOUND AND TARGET OrbbecSDK::OrbbecSDK)
    target_link_libraries(${PROJECT_NAME} OrbbecSDK::OrbbecSDK)
elseif(ORBBEC_LIBRARY)
    target_link_libraries(${PROJECT_NAME} ${ORBBEC_LIBRARY})
else()
    message(FATAL_ERROR "No OrbbecSDK library found to link")
endif()

# Platform-specific settings
if(WIN32)
    # For Windows, copy DLLs to output directory
    if(EXISTS ${ORBBEC_LIB_DIR})
        file(GLOB ORBBEC_DLLS "${ORBBEC_LIB_DIR}/*.dll")
        foreach(DLL ${ORBBEC_DLLS})
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${DLL} $<TARGET_FILE_DIR:${PROJECT_NAME}>)
        endforeach()
    endif()
    
    # Set Visual Studio working directory
    if(MSVC)
        set_target_properties(${PROJECT_NAME} PROPERTIES 
            VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    endif()
    
elseif(UNIX AND NOT APPLE)
    # For Linux, set RPATH
    set_target_properties(${PROJECT_NAME} PROPERTIES
        INSTALL_RPATH "${ORBBEC_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH TRUE)
        
elseif(APPLE)
    # For macOS, set RPATH
    set_target_properties(${PROJECT_NAME} PROPERTIES
        INSTALL_RPATH "${ORBBEC_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH TRUE)
endif()

# Installation
install(TARGETS ${PROJECT_NAME} 
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

# Print build information
message(STATUS "Build configuration:")
message(STATUS "  Target: ${PROJECT_NAME}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "  Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(ORBBEC_LIBRARY)
    message(STATUS "  OrbbecSDK Library: ${ORBBEC_LIBRARY}")
endif()