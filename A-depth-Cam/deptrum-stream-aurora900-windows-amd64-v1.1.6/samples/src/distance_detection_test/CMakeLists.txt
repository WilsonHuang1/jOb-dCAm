cmake_minimum_required(VERSION 3.16)
project(distance_detection_test)

add_executable(distance_detection_test distance_detection_test.cc ../sample_helper.cc)
target_link_libraries(distance_detection_test ${DEPTRUM_STREAM_LIB_NAME} ${OpenCV_LIBS} ${EXTRA_LIBS})
install(TARGETS distance_detection_test DESTINATION samples/bin)