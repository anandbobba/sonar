cmake_minimum_required(VERSION 3.8)
project(whistle_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)

install(DIRECTORY
  worlds
  launch
  models
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
