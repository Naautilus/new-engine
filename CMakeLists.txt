cmake_minimum_required(VERSION 3.16)
project(new_engine LANGUAGES C CXX)

# Use C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable aggressive optimizations (match your g++ flags)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast -march=native -flto=12 -fomit-frame-pointer -fdiagnostics-color=always")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Og -march=native -flto=12 -fomit-frame-pointer -fdiagnostics-color=always -g")

# add_compile_options(-Ofast -march=native -flto=12 -ffast-math -fomit-frame-pointer -fdiagnostics-color=always)

# Add include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/src/eigen
    ${CMAKE_SOURCE_DIR}/src/f16
)

# Collect all .cpp files recursively, excluding eigen/
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    src/*.cpp
)
list(FILTER SRC_FILES EXCLUDE REGEX ".*/src/eigen/.*")

# Add glad.c manually
set(SRC_FILES ${SRC_FILES} ${CMAKE_SOURCE_DIR}/src/renderer/glad.c)

# Create the executable
add_executable(main ${SRC_FILES})

# Link libraries (same as -lglfw3 -lopengl32 -lm)
target_link_libraries(main PRIVATE glfw3 opengl32 m)