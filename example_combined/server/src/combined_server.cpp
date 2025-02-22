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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gimbal_node.hpp"
#include "status_node.hpp"
#include "pump_node.hpp"
#include "communications.hpp"
#include "communications_fake.hpp"

// Pointer to executor so that the spin loop can be kill by the thread.
static std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec;

static void AutoStop()
{
  // Wait for delay seconds.
  int delay_s = 11;
  for (int i = 0; rclcpp::ok() && i < delay_s; ++i) {
    const std::chrono::seconds kWaitDelay(1);
    std::this_thread::sleep_for(kWaitDelay);
  }
  // Stop the spin loop to automatically quit.
  exec->cancel();
  RCLCPP_INFO(rclcpp::get_logger("server"), "Completed :-)");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create the nodes.
  rclcpp::NodeOptions options;
  auto gimbal_node = std::make_shared<GimbalNode>(options);
  auto pump_node = std::make_shared<PumpNode>(options);
  auto status_node = std::make_shared<StatusNode>(options);
  // The virtual class is used so that fake, mocked or real implementations
  // can be used (dependency injection).
  auto comms = std::make_shared<CommunicationsFake>();
  // Instantiate communications and open port.
  // If this fails, an exception will be thrown.
  comms->Init();
  // Add comms to nodes.
  gimbal_node->AddComms(comms);
  pump_node->AddComms(comms);
  status_node->AddComms(comms);
  // Add nodes to executor.
  exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(gimbal_node);
  exec->add_node(pump_node);
  exec->add_node(status_node);
  // Start auto-stop thread.
  auto test_thread = std::make_unique<std::thread>(&AutoStop);
  // Spin the executor.
  exec->spin();
  // Tidy up.
  test_thread->join();
  // Delete the thread before the executor to prevent std::runtime_error.
  test_thread.reset(nullptr);
  // Make sure executor and clients are closed before shutdown.
  // Can cause seg fault on exit if not done.
  exec.reset(nullptr);
  comms = nullptr;
  rclcpp::shutdown();
  return 0;
}
