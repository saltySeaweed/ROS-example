cmake_minimum_required(VERSION 3.10)
project(ROS_example)

set(CMAKE_CXX_STANDARD 17)

add_executable(rs_app src/main.cpp)

target_include_directories(rs_app PRIVATE include)

target_link_libraries(rs_app
    realsense2
    glfw
    GL
    GLU
    dl
    pthread
    m
    X11
    Xrandr
    Xi
)
