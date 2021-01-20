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

#include "example_msgs/msg/status.hpp"
#include "example_msgs/srv/gimbal.hpp"
#include "example_msgs/action/pump.hpp"

#include "status_subscriber_client.hpp"
#include "gimbal_service_client.hpp"
#include "pump_action_client.hpp"

// Static vars for sharing objects with RunTests().
static std::shared_ptr<StatusSubscriber> status_client;
static std::shared_ptr<GimbalServiceClient> gimbal_client;
static std::shared_ptr<PumpActionClient> pump_client;

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
  } while (!gimbal_ready && rclcpp::ok());
  if (rclcpp::ok()) {
    RCLCPP_INFO(rclcpp::get_logger("client"), "%s gimbal server ready", __func__);
    // Wait for pump action.
    int pump_ready = 0;
    do {
      pump_ready = gimbal_client->IsServerReady(kWaitDelayMs);
    } while (!pump_ready && rclcpp::ok());
    if (rclcpp::ok()) {
      RCLCPP_INFO(rclcpp::get_logger("client"), "%s pump server ready", __func__);
      // Wait for the status node to be connected.
      bool status_ready = false;
      do {
        status_ready = status_client->IsServerReady();
        std::this_thread::sleep_for(kWaitDelayMs);
      } while (!status_ready && rclcpp::ok());
      if (rclcpp::ok()) {
        RCLCPP_INFO(
          rclcpp::get_logger(
            "client"), "%s status publisher ready", __func__);
        // Wait for the micro-controller to be connected.
        bool status_connected = false;
        do {
          status_connected = status_client->connected();
          std::this_thread::sleep_for(kWaitDelayMs);
        } while (!status_connected && rclcpp::ok());
        RCLCPP_INFO(rclcpp::get_logger("client"), "%s connected", __func__);
        if (rclcpp::ok()) {
          terminated = false;
        }
      }
    }
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

// Bodge variable for testing.
static bool pump_example_done = false;

void PumpExampleCallback(bool done, float litres_pumped)
{
  pump_example_done = done;
  RCLCPP_INFO(
    rclcpp::get_logger("client"), "%s pumped %f litres",
    __func__, litres_pumped);
}

bool RunPumpExample()
{
  bool terminated = false;
  const std::chrono::milliseconds kSpinDelay(100);
  const std::chrono::milliseconds kStatusDelay(600);
  RCLCPP_INFO(rclcpp::get_logger("client"), "%s called", __func__);
  // Send 1.0 litres
  RCLCPP_INFO(rclcpp::get_logger("client"), "%s send 1.0l", __func__);
  float start_level_l = status_client->litres_remaining();
  pump_client->SendGoal(1.0, &PumpExampleCallback);
  // Wait until the pump has finished pumping.
  while (!pump_example_done) {
    if (!rclcpp::ok()) {
      terminated = true;
      break;
    }
    std::this_thread::sleep_for(kSpinDelay);
  }
  RCLCPP_INFO(rclcpp::get_logger("client"), "%s t1 %d", __func__, terminated);
  if (!terminated) {
    RCLCPP_INFO(rclcpp::get_logger("client"), "%s send 0.4l", __func__);
    // Send 0.4 litres
    bool result = pump_client->SendGoalWait(0.4, &PumpExampleCallback);
    if (result) {
      RCLCPP_INFO(rclcpp::get_logger("client"), "%s goal accepted", __func__);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("client"), "%s goal rejected", __func__);
      terminated = true;
    }
    if (!terminated) {
      // Wait until the pump has finished pumping.
      pump_example_done = false;
      while (!pump_example_done) {
        if (!rclcpp::ok()) {
          terminated = true;
          break;
        }
        std::this_thread::sleep_for(kSpinDelay);
      }
      if (!terminated) {
        // The status can take 500ms to arrive so wait.
        std::this_thread::sleep_for(kStatusDelay);
        float end_level_l = status_client->litres_remaining();
        RCLCPP_INFO(
          rclcpp::get_logger("client"), "Used %fl",
          start_level_l - end_level_l);
      }
    }
  }
  if (terminated) {
    RCLCPP_INFO(rclcpp::get_logger("client"), "Pump terminated.");
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
      terminated = RunPumpExample();
      if (!terminated) {
        RCLCPP_INFO(rclcpp::get_logger("client"), "Examples complete.");
        // Stop the executor spin loop to automatically quit.
        exec->cancel();
      }
    }
  }
  if (terminated) {
    RCLCPP_INFO(rclcpp::get_logger("client"), "Examples terminated early.");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  status_client = std::make_shared<StatusSubscriber>();
  gimbal_client = std::make_shared<GimbalServiceClient>();
  pump_client = std::make_shared<PumpActionClient>();
  // Add nodes to executor.
  exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  // NOTE: The gimbal service is NOT an rclcpp::Node so does not need an executor.
  exec->add_node(status_client);
  exec->add_node(pump_client);
  // Start thread to exercise the service and action clients.
  auto test_thread = std::make_unique<std::thread>(&RunExamples);
  // Start the nodes (blocks until Ctrl+C).
  exec->spin();
  // Tidy up.
  test_thread->join();
  // Delete the thread before the executor to prevent std::runtime_error.
  test_thread.reset(nullptr);
  // Make sure clients are closed before shutdown.
  // Can cause seg fault on exit if not done.
  status_client = nullptr;
  gimbal_client = nullptr;
  pump_client = nullptr;
  rclcpp::shutdown();
  return 0;
}
