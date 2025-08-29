cmake_minimum_required(VERSION 3.10)
project(ROS_example)

set(CMAKE_CXX_STANDARD 17)


set(SOURCES
    main.cpp
    src/rs-capture.cpp
    src/rs-save-to-disk.cpp
    src/rs-depth.c        
)

add_executable(rs_app ${SOURCES})

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
