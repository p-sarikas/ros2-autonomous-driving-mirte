cmake_minimum_required(VERSION 3.8)
project(pcl_obstacle_detector)

# Fix for PCL_ROOT warning 
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcl_obstacle_detector_node
  src/pcl_obstacle_detector_main.cpp
  src/pcl_obstacle_detector.cpp
)

ament_target_dependencies(pcl_obstacle_detector_node
  rclcpp
  sensor_msgs
  vision_msgs
  pcl_conversions
  PCL
)

target_link_libraries(pcl_obstacle_detector_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  pcl_obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
