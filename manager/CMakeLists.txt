cmake_minimum_required(VERSION 3.8)
project(manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(calculator REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# find header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(manager_node 
  src/manager_node_start.cpp 
  src/manager.cpp
)

add_executable(global_map_node
  src/global_map_start.cpp
  src/global_map.cpp
)

ament_target_dependencies(global_map_node 
  rclcpp
  interfaces
  calculator
  nav_msgs
  tf2_ros
)

ament_target_dependencies(manager_node 
  rclcpp
  interfaces
  calculator
  nav_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS
  manager_node
  global_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

add_library(manager SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/manager.cpp)
ament_target_dependencies(manager 
  rclcpp
  interfaces
  calculator
  nav_msgs
  tf2_ros
  visualization_msgs
)
install(
  TARGETS manager
  LIBRARY DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()