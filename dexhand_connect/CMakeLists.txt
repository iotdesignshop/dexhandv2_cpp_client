cmake_minimum_required(VERSION 3.0.0)
project(dexhand_connect VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()


add_library(dexhand_connect STATIC src/dexhand_connect.cpp)

target_include_directories(dexhand_connect PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)