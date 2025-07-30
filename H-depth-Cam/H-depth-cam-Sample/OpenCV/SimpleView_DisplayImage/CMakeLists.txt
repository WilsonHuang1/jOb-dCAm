if(CMAKE_HOST_WIN32)
  cmake_minimum_required(VERSION 3.19)
else()
  cmake_minimum_required(VERSION 3.5)
endif()

project(SimpleView_DisplayImage)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build Type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 14)

set(Files ${PROJECT_SOURCE_DIR}/main.cpp)

find_package(OpenCV REQUIRED)

if(CMAKE_CL_64)
    set(MV3DRGBD_DEV_LIBRARIES $ENV{MV3DRGBD_DEV_ENV}/Libraries/win64)
else(CMAKE_CL_64)
    set(MV3DRGBD_DEV_LIBRARIES $ENV{MV3DRGBD_DEV_ENV}/Libraries/win32)
endif(CMAKE_CL_64)

set(MV3DRGBD_DEV_INCLUDE $ENV{MV3DRGBD_DEV_ENV}/Includes)
set (MV3DRGBD_DEV_LIB "Mv3dRgbd.lib")
include_directories(${MV3DRGBD_DEV_INCLUDE})
link_directories(${MV3DRGBD_DEV_LIBRARIES})

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBS})

add_executable(${PROJECT_NAME} ${Files})

target_link_libraries(${PROJECT_NAME} ${MV3DRGBD_DEV_LIB} ${OpenCV_LIBS})

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

add_compile_definitions(NOMINMAX)