cmake_minimum_required(VERSION 3.8)
project(waypoint_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(msg_set_msgs REQUIRED)

# Check for GeographicLib
find_package(PkgConfig REQUIRED)
pkg_check_modules(GeoLib REQUIRED geographiclib)

# Include header directories
include_directories(
  include
)

# Create the navigation core library
add_library(nav_core
  src/nav_core.cpp
  src/utils.cpp
)
target_include_directories(nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nav_core PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(nav_core
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  msg_set_msgs
)
target_link_libraries(nav_core ${GeoLib_LIBRARIES})
target_include_directories(nav_core PRIVATE ${GeoLib_INCLUDE_DIRS})

# Create the navigation FSM library
add_library(nav_fsm
  src/nav_fsm.cpp
)
target_include_directories(nav_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nav_fsm PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(nav_fsm
  rclcpp
  std_msgs
)

# Create the main navigation node executable
add_executable(nav_waypoint_node
  src/nav_waypoint_node.cpp
)
target_compile_features(nav_waypoint_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(nav_waypoint_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  message_filters
  msg_set_msgs
)
target_link_libraries(nav_waypoint_node
  nav_core
  nav_fsm
)
target_link_libraries(nav_waypoint_node ${GeoLib_LIBRARIES})
target_include_directories(nav_waypoint_node PRIVATE ${GeoLib_INCLUDE_DIRS})

# Install executables
install(TARGETS
  nav_waypoint_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(nav_core nav_fsm)
ament_package()