cmake_minimum_required(VERSION 3.5)
project(leeds_pump_combined_client)

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
# If you get an error like this:
#  CMake Error at CMakeLists.txt:15 (find_package):
#    By not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH this project
#    has asked CMake to find a package configuration file provided by
#    "ament_cmake", but CMake did not find one.
# you need to source the /opt/ros/foxy/setup.bash file!
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(leeds_pump_msgs REQUIRED)

# The Leeds pump client.
set(CLIENT_NAME combined_client_exec)

set(SRC_FILES
  src/client_example.cpp
  src/status_subscriber_client.cpp
  src/gimbal_service_client.cpp
  src/pump_action_client.cpp
)

add_executable(${CLIENT_NAME}
  ${SRC_FILES}
)

target_include_directories(${CLIENT_NAME} PRIVATE
  ${CMAKE_SOURCE_DIR}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${CLIENT_NAME} PUBLIC -Wall -Wextra -pedantic)
endif()

ament_target_dependencies(${CLIENT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_action
  leeds_pump_msgs
)

install(TARGETS ${CLIENT_NAME}
  DESTINATION lib/${CLIENT_NAME})

if(BUILD_TESTING)
  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()  # BUILD_TESTING

# Installs the package.xml file.
ament_package()
