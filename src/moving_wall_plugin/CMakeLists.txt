cmake_minimum_required(VERSION 3.8)
project(moving_wall_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(moving_wall_plugin SHARED
  src/moving_wall_plugin.cpp
)

target_link_libraries(moving_wall_plugin
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS moving_wall_plugin
  LIBRARY DESTINATION lib
)

ament_package()
