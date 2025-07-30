if(CMAKE_HOST_WIN32)
  cmake_minimum_required(VERSION 3.19)
else()
  cmake_minimum_required(VERSION 3.5)
endif()

project(TexturedPointCloudToPcl)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE release CACHE STRING "Build Type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 14)

set(Files ${PROJECT_SOURCE_DIR}/main.cpp)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_compile_options(-mno-outline-atomics)
endif()

if(CMAKE_HOST_WIN32)
 if(MSVC_VERSION LESS 1920)
    find_package(PCL 1.8.1...1.9.1 REQUIRED)
  elseif(MSVC_VERSION LESS 1930)
    find_package(PCL 1.8.1...1.12.1 REQUIRED)
  else()
    find_package(PCL 1.8.1 REQUIRED)
  endif()
elseif(CMAKE_HOST_UNIX)
  find_package(PCL REQUIRED)
endif()
if(NOT PCL_FOUND)
  message(
    FATAL_ERROR
      "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake)."
  )
endif()

if(CMAKE_CL_64)
    set(MV3DRGBD_DEV_LIBRARIES $ENV{MV3DRGBD_DEV_ENV}/Libraries/win64)
else(CMAKE_CL_64)
    set(MV3DRGBD_DEV_LIBRARIES $ENV{MV3DRGBD_DEV_ENV}/Libraries/win32)
endif(CMAKE_CL_64)

set(MV3DRGBD_DEV_INCLUDE $ENV{MV3DRGBD_DEV_ENV}/Includes)
set (MV3DRGBD_DEV_LIB "Mv3dRgbd.lib")
include_directories(${MV3DRGBD_DEV_INCLUDE})
link_directories(${MV3DRGBD_DEV_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} ${Files})

target_link_libraries(${PROJECT_NAME} ${MV3DRGBD_DEV_LIB} ${PCL_LIBRARIES})

add_compile_definitions(NOMINMAX)