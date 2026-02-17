cmake_minimum_required(VERSION 3.8)
project(composable_player)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_srvs REQUIRED)

# Player component
add_library(player_component SHARED
  src/player_node.cpp
)
target_include_directories(player_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(player_component PUBLIC cxx_std_17)
ament_target_dependencies(player_component
  rclcpp
  rclcpp_components
  rosbag2_cpp
  rosbag2_storage
  std_srvs
)
rclcpp_components_register_node(player_component
  PLUGIN "composable_player::PlayerNode"
  EXECUTABLE player_node
)

# Recorder component
add_library(recorder_component SHARED
  src/recorder_node.cpp
)
target_include_directories(recorder_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(recorder_component PUBLIC cxx_std_17)
ament_target_dependencies(recorder_component
  rclcpp
  rclcpp_components
  rosbag2_cpp
  rosbag2_storage
  std_srvs
)
rclcpp_components_register_node(recorder_component
  PLUGIN "composable_player::RecorderNode"
  EXECUTABLE recorder_node
)

install(TARGETS player_component recorder_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
