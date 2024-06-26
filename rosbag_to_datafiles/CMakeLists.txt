cmake_minimum_required(VERSION 3.8)
project(rosbag_to_datafiles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)

# Add executable
add_executable(rosbag_subscriber src/rosbag_subscriber.cpp)

# Include directories
target_include_directories(rosbag_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# target_include_directories(rosbag_subscriber PRIVATE include)

# Link libraries
ament_target_dependencies(rosbag_subscriber 
  rclcpp 
  sensor_msgs 
  nav_msgs
  pcl_conversions 
  pcl_ros 
  std_srvs 
)


install(TARGETS rosbag_subscriber
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
