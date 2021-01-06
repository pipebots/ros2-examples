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

#include "example_msgs/action/pump.hpp"

#include "pump_action_client.hpp"

// Variable for testing.
static bool pump_example_done = false;

// Static vars for sharing objects with RunTests().
static std::shared_ptr<PumpActionClient> pump_client;

/**
 * @brief Blocks until the server is ready.
 * @return true if terminated, false on success.
 */
bool WaitForServer()
{
  RCLCPP_INFO(rclcpp::get_logger("client"), "%s Waiting for server...", __FUNCTION__);
  bool terminated = true;
  const std::chrono::milliseconds kWaitDelayMs(250);
  // Wait for pump action.
  int pump_ready = 0;
  do {
    pump_ready = pump_client->IsServerReady(kWaitDelayMs);
  } while (!pump_ready && rclcpp::ok());
  if (rclcpp::ok()) {
    RCLCPP_INFO(rclcpp::get_logger("client"), "%s pump server ready", __FUNCTION__);
    terminated = false;
  }
  return terminated;
}

void PumpExampleCallback(bool done, float litres_pumped)
{
  pump_example_done = done;
  RCLCPP_INFO(rclcpp::get_logger("client"), "%s pumped %f litres",
    __FUNCTION__, litres_pumped);
}

bool RunPumpExample()
{
  bool terminated = false;
  const std::chrono::milliseconds kSpinDelay(100);
  const std::chrono::milliseconds kStatusDelay(600);
  RCLCPP_INFO(rclcpp::get_logger("client"), "%s called", __FUNCTION__);
  // Send 1.0 litres
  RCLCPP_INFO(rclcpp::get_logger("client"), "%s send 1.0l", __FUNCTION__);
  pump_client->SendGoal(1.0, &PumpExampleCallback);
  // Wait until the pump has finished pumping.
  while (!pump_example_done) {
    if (!rclcpp::ok()) {
      terminated = true;
      break;
    }
    std::this_thread::sleep_for(kSpinDelay);
  }
  RCLCPP_INFO(rclcpp::get_logger("client"), "%s t1 %d", __FUNCTION__, terminated);
  if (!terminated) {
    RCLCPP_INFO(rclcpp::get_logger("client"), "%s send 0.4l", __FUNCTION__);
    // Send 0.4 litres
    bool result = pump_client->SendGoalWait(0.4, &PumpExampleCallback);
    if (result) {
      RCLCPP_INFO(rclcpp::get_logger("client"), "%s goal accepted", __FUNCTION__);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("client"), "%s goal rejected", __FUNCTION__);
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
    terminated = RunPumpExample();
    if (!terminated) {
      RCLCPP_INFO(rclcpp::get_logger("client"), "Examples complete.");
    }
  }
  if (terminated) {
    RCLCPP_INFO(rclcpp::get_logger("client"), "Examples terminated early.");
  }
  RCLCPP_INFO(rclcpp::get_logger("client"), "Press Ctrl+c to exit.");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  pump_client = std::make_shared<PumpActionClient>();
  // Add node to executor.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(pump_client);
  // Start thread to exercise the service and action clients.
  auto test_thread = new std::thread(&RunExamples);
  // Start the nodes (blocks until Ctrl-c).
  exec.spin();
  // Tidy up.
  test_thread->join();
  delete test_thread;
  // Make sure clients are closed before shutdown.
  // Can cause seg fault on exit if not done.
  pump_client = nullptr;
  rclcpp::shutdown();
  return 0;
}
