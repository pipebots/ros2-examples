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
#include "rclcpp_action/rclcpp_action.hpp"

#include "leeds_pump_msgs/msg/leeds_pump_status.hpp"

#include "status_subscriber_client.hpp"

// Static vars for sharing objects with RunTests().
static std::shared_ptr<StatusSubscriber> status_client;

/**
 * @brief Blocks until the server is ready.
 * @return true if terminated, false on success.
 */
bool WaitForServer()
{
  RCLCPP_INFO(rclcpp::get_logger("leeds_pump_client"), "%s Waiting for server...", __FUNCTION__);
  bool terminated = true;
  // Wait for gimbal service.
  const std::chrono::milliseconds kWaitDelayMs(250);
  // Wait for the status node to be connected.
  bool status_ready = false;
  do {
    status_ready = status_client->IsServerReady();
    std::this_thread::sleep_for(kWaitDelayMs);
  } while (!status_ready && rclcpp::ok());
  if (rclcpp::ok()) {
    RCLCPP_INFO(rclcpp::get_logger(
        "leeds_pump_client"), "%s status publisher ready", __FUNCTION__);
    // Wait for the micro-controller to be connected.
    bool status_connected = false;
    do {
      status_connected = status_client->connected();
      std::this_thread::sleep_for(kWaitDelayMs);
    } while (!status_connected && rclcpp::ok());
    RCLCPP_INFO(rclcpp::get_logger("leeds_pump_client"), "%s connected", __FUNCTION__);
    if (rclcpp::ok()) {
      terminated = false;
    }
  }
  return terminated;
}

void RunExamples()
{
  RCLCPP_INFO(rclcpp::get_logger("leeds_pump_client"), "Examples started.");
  bool terminated = WaitForServer();
  if (!terminated) {
    // Listen to the status client for 30 loops then quit.
    // This should show logging output from the server and client.
    int loop_count = 0;
    bool status_ready = false;
    const std::chrono::milliseconds kWaitDelayMs(1000);
    do {
      status_ready = status_client->IsServerReady();
      std::this_thread::sleep_for(kWaitDelayMs);
    } while (!status_ready && rclcpp::ok() && loop_count < 30);
  }
  if (terminated) {
    RCLCPP_INFO(rclcpp::get_logger("leeds_pump_client"), "Examples terminated early.");
  }
  RCLCPP_INFO(rclcpp::get_logger("leeds_pump_client"), "Press Ctrl+c to exit.");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  status_client = std::make_shared<StatusSubscriber>();
  // Add nodes to executor.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(status_client);
  // Start thread to exercise the service and action clients.
  auto test_thread = new std::thread(&RunExamples);
  // Start the nodes (blocks until Ctrl-c).
  exec.spin();
  // Tidy up.
  test_thread->join();
  delete test_thread;
  // Make sure clients are destroyed before shutdown.
  // Will cause seg fault on exit if not done.
  status_client = nullptr;
  rclcpp::shutdown();
  return 0;
}
