cmake_minimum_required(VERSION 3.22.1)
project(rwa3 VERSION 1.0 LANGUAGES C CXX)


add_executable(rwa3_cpp src/main.cpp src/robot_control.cpp src/robot_types.cpp)
target_include_directories(rwa3_cpp PRIVATE include)
add_compile_options(-Wall -Werror -Wextra -pedantic-errors)

# Set C++17 standard for the targets
set_property(TARGET rwa3_cpp PROPERTY CXX_STANDARD 17)
set_property(TARGET rwa3_cpp PROPERTY CXX_STANDARD_REQUIRED ON)
