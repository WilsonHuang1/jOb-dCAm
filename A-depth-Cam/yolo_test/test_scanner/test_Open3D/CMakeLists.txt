cmake_minimum_required(VERSION 3.18)
project(test_Open3D)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Open3D REQUIRED)

add_executable(test_Open3D test_open3d.cpp)

target_link_libraries(test_Open3D ${Open3D_LIBRARIES})
target_include_directories(test_Open3D PRIVATE ${Open3D_INCLUDE_DIRS})

target_compile_definitions(test_Open3D PRIVATE ${Open3D_DEFINITIONS})