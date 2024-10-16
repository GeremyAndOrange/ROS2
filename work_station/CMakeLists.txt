cmake_minimum_required(VERSION 3.8)
project(work_station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

# find header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# build ui files
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(UI_FILES src/console_gui.ui)
QT5_WRAP_UI(UI_HEADERS ${UI_FILES})

set(MOC_FILES include/console_gui/console_gui.hpp)
QT5_WRAP_CPP(MOC_HEADERS ${MOC_FILES})

# system
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

# node
add_executable(console_node 
  src/console_gui_start.cpp
  src/console_gui.cpp
)

ament_target_dependencies(console_node 
  rclcpp
)

# Qt
target_sources(console_node PRIVATE ${UI_HEADERS})
target_sources(console_node PRIVATE ${MOC_HEADERS})

target_link_libraries(console_node 
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

install(TARGETS
  console_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()