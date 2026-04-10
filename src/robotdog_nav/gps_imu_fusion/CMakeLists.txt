cmake_minimum_required(VERSION 3.8)
project(gps_imu_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Check for GeographicLib
find_package(PkgConfig REQUIRED)
pkg_check_modules(GeoLib REQUIRED geographiclib)

# Include header directories
include_directories(
  include
)

# Create the fusion node executable
add_executable(gps_imu_fusion_node
  src/gps_imu_fusion_node.cpp
)
target_compile_features(gps_imu_fusion_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(gps_imu_fusion_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(gps_imu_fusion_node ${GeoLib_LIBRARIES})
target_include_directories(gps_imu_fusion_node PRIVATE ${GeoLib_INCLUDE_DIRS})

# Install executables
install(TARGETS
  gps_imu_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()