cmake_minimum_required(VERSION 3.8)
project(uwaba_prototype_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)

install(
  DIRECTORY config description launch world rviz maps params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
