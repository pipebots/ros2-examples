cmake_minimum_required(VERSION 3.5)
project(leeds_pump_single_action_client)

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
find_package(leeds_pump_msgs REQUIRED)

# The Leeds pump client.
set(CLIENT_NAME combined_client)

add_executable(${CLIENT_NAME}
  src/client_example.cpp
  src/pump_action_client.cpp
)

target_include_directories(${CLIENT_NAME} PRIVATE
${CMAKE_SOURCE_DIR}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${CLIENT_NAME} PUBLIC -Wall -Wextra -pedantic)
endif()

ament_target_dependencies(${CLIENT_NAME}
  rclcpp
  rclcpp_action
  rclcpp_components
  leeds_pump_msgs
)

if(BUILD_TESTING)
  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()  # BUILD_TESTING

# Installs the package.xml file.
ament_package()
