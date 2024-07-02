cmake_minimum_required(VERSION 3.0.0)
project(sample-client VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()


add_executable(native_message_example
    native_messages.cpp
)

add_executable(joint_angle_example
    joint_angles.cpp
)


# Add dexhand-connect library
add_subdirectory(${CMAKE_SOURCE_DIR}/../dexhand_connect dexhand_connect)
target_link_libraries(native_message_example 
    dexhand_connect     
)
target_link_libraries(joint_angle_example 
    dexhand_connect     
)
