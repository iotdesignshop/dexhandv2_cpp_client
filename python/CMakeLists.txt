cmake_minimum_required(VERSION 3.0.0)
project(dexhand_connect_py VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Ensure PyBind11 is available
include (FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git 
    GIT_TAG        v2.12.0
)
FetchContent_MakeAvailable(pybind11)


# Compile in our bindings file
pybind11_add_module(dexhand_connect_py bindings.cpp)

# Link against dexhand-connect library
add_subdirectory(../dexhand_connect ${CMAKE_BINARY_DIR}/dexhand_connect)
target_link_libraries(dexhand_connect_py PRIVATE dexhand_connect)


# Add dexhand-connect library

set_target_properties(dexhand_connect_py PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
