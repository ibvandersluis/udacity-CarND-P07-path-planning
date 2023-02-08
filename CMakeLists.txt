cmake_minimum_required(VERSION 3.5)
project(udacity_path_planning)

add_definitions(-std=c++11)

add_compile_options(-Wall -Wextra -pedantic -Werror)

include_directories(
  include
  /usr/include/eigen3/
)

add_executable(path_planning
  src/main.cpp
)

target_link_libraries(path_planning z ssl uv uWS)
