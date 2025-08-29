cmake_minimum_required(VERSION 3.10)
project(ROS_example)

set(CMAKE_CXX_STANDARD 17)
add_executable(rs_app main.cpp)

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


