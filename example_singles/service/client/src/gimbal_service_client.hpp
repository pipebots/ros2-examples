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

#ifndef GIMBAL_SERVICE_CLIENT_HPP_
#define GIMBAL_SERVICE_CLIENT_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_msgs/srv/gimbal.hpp"


class GimbalServiceClient
{
public:
  GimbalServiceClient();
  ~GimbalServiceClient();

  /**
   * @brief Waits for the server to be ready for a maximum of the given time.
   *
   * @param wait_time_ms Time in std::chrono::milliseconds.
   * @return int 1 = server ready, -1 = termination requested, 0 server not ready.
   */
  int IsServerReady(const std::chrono::milliseconds & wait_time_ms);

  /**
   * @brief Send a request to move the gimbal.
   * Blocks until response is received.
   *
   * @param pitch New angle to set in degrees. 0 is centered.
   * On success, set to the actual value moved.
   * @param yaw New angle to set in degrees. 0 is centered.
   * On success, set to the actual value moved.
   * @return 0 = Failed, 1 = success.
   */
  int Move(int * pitch, int * yaw);

private:
  rclcpp::Client<example_msgs::srv::Gimbal>::SharedPtr client_;
  rclcpp::Node::SharedPtr node_;
};

#endif  // GIMBAL_SERVICE_CLIENT_HPP_
