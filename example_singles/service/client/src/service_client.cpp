// Copyright 2020 University of Leeds.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Code based on examples from here: https://github.com/ros2/examples

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_msgs/srv/gimbal.hpp"
#include "gimbal_service_client.hpp"

// Static vars for sharing objects with RunTests().
static std::shared_ptr<GimbalServiceClient> gimbal_client;

// Pointer to executor so that the spin loop can be kill by the thread.
static std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec;

/**
 * @brief Blocks until the server is ready.
 * @return true if terminated, false on success.
 */
bool WaitForServer()
{
  RCLCPP_INFO(rclcpp::get_logger("client"), "%s Waiting for server...", __func__);
  bool terminated = true;
  // Wait for gimbal service.
  const std::chrono::milliseconds kWaitDelayMs(250);
  int gimbal_ready = 0;
  do {
    gimbal_ready = gimbal_client->IsServerReady(kWaitDelayMs);
  } while (gimbal_ready == 0 && rclcpp::ok());
  if (rclcpp::ok()) {
    RCLCPP_INFO(rclcpp::get_logger("client"), "%s gimbal server ready", __func__);
    terminated = false;
  }
  return terminated;
}

bool RunGimbalExample()
{
  bool terminated = false;
  // Set pitch to -10, yaw to +20
  int pitch = -10;
  int yaw = +20;
  RCLCPP_INFO(
    rclcpp::get_logger(
      "client"), "Gimbal being moved to pitch %d, yaw %d.", pitch, yaw);
  int result = gimbal_client->Move(&pitch, &yaw);
  RCLCPP_INFO(
    rclcpp::get_logger(
      "client"), "Gimbal moved to pitch %d, yaw %d.", pitch, yaw);
  if (result && rclcpp::ok()) {
    // Move gimbal again.
    pitch = +80;
    yaw = -70;
    RCLCPP_INFO(
      rclcpp::get_logger(
        "client"), "Gimbal being moved to pitch %d, yaw %d.", pitch, yaw);
    result = gimbal_client->Move(&pitch, &yaw);
    RCLCPP_INFO(
      rclcpp::get_logger(
        "client"), "Gimbal moved to pitch %d, yaw %d.", pitch, yaw);
    if (rclcpp::ok()) {
      if (result && rclcpp::ok()) {
        RCLCPP_INFO(rclcpp::get_logger("client"), "Finished gimbal example.");
      } else {
        RCLCPP_INFO(rclcpp::get_logger("client"), "Gimbal example failed.");
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("client"), "Gimbal terminated.");
      terminated = true;
    }
  }
  return terminated;
}

void RunExamples()
{
  RCLCPP_INFO(rclcpp::get_logger("client"), "Examples started.");
  bool terminated = WaitForServer();
  if (!terminated) {
    terminated = RunGimbalExample();
    if (!terminated) {
      RCLCPP_INFO(rclcpp::get_logger("client"), "Examples complete.");
      // Stop the spin loop to automatically quit.
      exec->cancel();
    }
  }
  if (terminated) {
    RCLCPP_INFO(rclcpp::get_logger("client"), "Examples terminated early.");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  gimbal_client = std::make_shared<GimbalServiceClient>();
  // Add nodes to executor.
  // NOTE: The gimbal service is NOT an rclcpp::Node so does not need an
  // executor.  However, an executor is still needed to cause this thread
  // to wait until the test code completes.
  exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  // Start thread to exercise the service and action clients.
  auto test_thread = std::make_unique<std::thread>(&RunExamples);
  // Blocks until Ctrl+C.
  exec->spin();
  // Tidy up.
  test_thread->join();
  // Delete the thread before the executor to prevent std::runtime_error.
  test_thread.reset(nullptr);
  // Make sure clients are closed before shutdown.
  // Can cause seg fault on exit if not done.
  gimbal_client = nullptr;
  rclcpp::shutdown();
  return 0;
}
