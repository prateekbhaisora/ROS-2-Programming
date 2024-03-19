cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# This tells the compiler to install the directory launch, inside the share/my_robot_bringup folder
# which will be inside the install folder of my ros2_ws

