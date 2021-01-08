cmake_minimum_required(VERSION 3.5)
project(action_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_msgs REQUIRED)

# The action server.
set(SERVER_NAME ${PROJECT_NAME}_exec)

set(SRC_FILES
  src/action_server.cpp
  src/pump_node.cpp
)

add_executable(${SERVER_NAME} ${SRC_FILES})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${SERVER_NAME} PUBLIC -Wall -Wextra -pedantic)
endif()

ament_target_dependencies(${SERVER_NAME}
  rclcpp
  rclcpp_action
  rclcpp_components
  example_msgs
)

install(TARGETS ${SERVER_NAME}
  # PROJECT_NAME __MUST__ be used here for launch files to work.
  DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()  # BUILD_TESTING

# Installs the package.xml file.
ament_package()
