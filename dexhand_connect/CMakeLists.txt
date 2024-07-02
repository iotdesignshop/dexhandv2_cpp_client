cmake_minimum_required(VERSION 3.0.0)
project(dexhand_connect VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

# Dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUDEV REQUIRED libudev)

# Include directories
include_directories(${LIBUDEV_INCLUDE_DIRS})

add_subdirectory(dexhandv2_binary_msgs)

add_library(dexhand_connect STATIC 
    src/dexhand_connect.cpp
    src/dexhand_message.cpp
    src/dexhand_command.cpp
    src/dexhand_servomgr.cpp
    src/dexhand_servo.cpp
    src/dexhand_joint_angle_controller.cpp)
target_link_libraries(dexhand_connect dexhandv2_binary_msgs ${Protobuf_LIBRARIES} ${LIBUDEV_LIBRARIES})

target_include_directories(dexhand_connect PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
