#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(sample_firmware_upgrade)

add_executable(firmware_upgrade firmware_upgrade.cc)
target_link_libraries(firmware_upgrade ${DEPTRUM_STREAM_LIB_NAME} ${EXTRA_LIBS})
install(TARGETS firmware_upgrade DESTINATION samples/bin)