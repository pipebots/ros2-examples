cmake_minimum_required(VERSION 3.5)
project(leeds_pump_single_service_server)

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
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(leeds_pump_msgs REQUIRED)


# The combined Leeds Pump.
set(SRC_FILES
  src/leeds_pump.cpp
  src/communications_fake.cpp
  src/gimbal_node.cpp
  src/pump_node.cpp
  src/status_node.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILES})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra -pedantic)
endif()

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_action
  leeds_pump_msgs
)

# Testing
if(BUILD_TESTING)
  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()  # BUILD_TESTING

# Installs the package.xml file.
ament_package()
