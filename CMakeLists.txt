cmake_minimum_required(VERSION 3.0.0)
project(opengl_project_0 VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set MinGW compilers explicitly
set(CMAKE_C_COMPILER "C:/msys64/mingw64/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "C:/msys64/mingw64/bin/g++.exe")

cmake_policy(SET CMP0072 NEW)

find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(opengl_project_0 src/main.cpp src/renderer/glad.h src/renderer/glad.c)
target_link_libraries(opengl_project_0 glfw OpenGL::GL)
include_directories(${EIGEN3_INCLUDE_DIR})
