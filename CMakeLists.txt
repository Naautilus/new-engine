cmake_minimum_required(VERSION 3.16)
project(new_engine LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast -march=native -flto=12 -fomit-frame-pointer -fdiagnostics-color=always")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Og -march=native -flto=12 -fomit-frame-pointer -fdiagnostics-color=always -g")

include_directories(
    ${CMAKE_SOURCE_DIR}/external/eigen
)

# Collect all .cpp files recursively, excluding eigen/
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    src/*.cpp
)
#list(FILTER SRC_FILES EXCLUDE REGEX ".*/src/eigen/.*")

# Add glad.c manually
set(SRC_FILES ${SRC_FILES} ${CMAKE_SOURCE_DIR}/src/renderer/glad.c)

# Create the executable
add_executable(main ${SRC_FILES})

# Link libraries (same as -lglfw3 -lopengl32 -lm)
target_link_libraries(main PRIVATE glfw3 opengl32 m)