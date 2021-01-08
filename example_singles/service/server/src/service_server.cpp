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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create the nodes.
  rclcpp::NodeOptions options;
  auto gimbal_node = std::make_shared<GimbalNode>(options);
  // Add nodes to executor.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gimbal_node);
  RCLCPP_INFO(rclcpp::get_logger("server"), "Spinning.");
  exec.spin();
  RCLCPP_INFO(rclcpp::get_logger("server"), "Stopped.");
  rclcpp::shutdown();
  return 0;
}
