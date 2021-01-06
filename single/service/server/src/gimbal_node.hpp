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

#ifndef GIMBAL_NODE_HPP_
#define GIMBAL_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "leeds_pump_msgs/srv/leeds_pump_gimbal.hpp"
#include "communications.hpp"

class GimbalNode : public rclcpp::Node
{
public:
  explicit GimbalNode(rclcpp::NodeOptions options);
  void AddComms(Communications * comms);

private:
  void HandleService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<leeds_pump_msgs::srv::LeedsPumpGimbal::Request> request,
    const std::shared_ptr<leeds_pump_msgs::srv::LeedsPumpGimbal::Response> response);
  std::shared_ptr<rclcpp::Service<leeds_pump_msgs::srv::LeedsPumpGimbal>> server_;
  Communications * comms_;
};

#endif  // GIMBAL_NODE_HPP_
