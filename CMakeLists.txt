cmake_minimum_required(VERSION 3.8)
project(necrodrive_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # CAN
  ros2_socketcan 
)

# specify required version of ros2_control
find_package(controller_manager 4.0.0)
# if not found, complain
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control v4.0.0 or higher is required! ")
endif()

find_package(backward_ros REQUIRED) # idk what this is for
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()


# compile
add_library(
  necrodrive_ros2
  SHARED
  hardware/necrodrive_system.cpp
  hardware/can_reader.cpp
)
target_compile_features(necrodrive_ros2 PUBLIC cxx_std_20)
target_include_directories(necrodrive_ros2 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/necrodrive_ros2>
)

ament_target_dependencies(
  necrodrive_ros2 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# export hw plugin
pluginlib_export_plugin_description_file(hardware_interface necrodrive_ros2.xml)

# install
install(
  DIRECTORY hardware/include 
  DESTINATION include/necrodrive_ros2
)

install(
  DIRECTORY description bringup_test
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS necrodrive_ros2
  EXPORT export_necrodrive_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# export
ament_export_targets(export_necrodrive_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()