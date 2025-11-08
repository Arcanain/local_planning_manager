cmake_minimum_required(VERSION 3.8)
project(local_planning_manager)

# warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# core library (component and node)
add_library(local_planning_manager_library
  src/local_planning_manager_component.cpp
  src/local_planning_manager_node.cpp
)

ament_target_dependencies(local_planning_manager_library
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

target_include_directories(local_planning_manager_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(local_planning_manager_library PUBLIC cxx_std_17)

# node entry (main)
add_executable(local_planning_manager
  src/local_planning_manager.cpp
)

target_link_libraries(local_planning_manager
  local_planning_manager_library
)

ament_target_dependencies(local_planning_manager
  rclcpp
)

# install
install(TARGETS
  local_planning_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  local_planning_manager_library
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# (optional) install configs
# install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})

# lint / tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()
  # add_subdirectory(test)
endif()

ament_package()