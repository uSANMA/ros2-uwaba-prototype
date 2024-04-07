cmake_minimum_required(VERSION 3.8)
project(uwaba_prototype_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

if(NOT EXISTS ros2_control)
  file(MAKE_DIRECTORY ros2_control)
endif()

install(
  DIRECTORY launch meshes rviz urdf config ros2_control
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
