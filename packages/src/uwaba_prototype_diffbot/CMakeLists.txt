cmake_minimum_required(VERSION 3.8)
project(uwaba_prototype_diffbot LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) #-Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  uwaba_prototype_diffbot
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(uwaba_prototype_diffbot PUBLIC cxx_std_17)
target_include_directories(uwaba_prototype_diffbot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/uwaba_prototype_diffbot>
)
ament_target_dependencies(
  uwaba_prototype_diffbot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "UWABA_PROTOTYPE_DIFFBOT_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface uwaba_prototype_diffbot.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/uwaba_prototype_diffbot
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/uwaba_prototype_diffbot
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/uwaba_prototype_diffbot
)
install(TARGETS uwaba_prototype_diffbot
  EXPORT export_uwaba_prototype_diffbot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_uwaba_prototype_diffbot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
