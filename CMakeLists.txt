cmake_minimum_required(VERSION 3.0.0)
project(sample-client VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

add_executable(sample-client main.cpp)

# Add dexhand-connect library
add_subdirectory(dexhand_connect)
add_subdirectory(dexhandv2_binary_msgs)
target_link_libraries(sample-client dexhand_connect dexhandv2_binary_msgs)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
