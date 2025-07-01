cmake_minimum_required(VERSION 3.16)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(sample_firmware_upgrade)
add_subdirectory(sample_hello_deptrum)

# opencv required
if(${OpenCV_FOUND})
  add_subdirectory(sample_viewer)
  add_subdirectory(sample_lite)
endif(${OpenCV_FOUND})

