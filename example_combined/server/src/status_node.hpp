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

#ifndef STATUS_NODE_HPP_
#define STATUS_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_msgs/msg/status.hpp"
#include "communications.hpp"

class StatusNode : public rclcpp::Node
{
public:
  explicit StatusNode(rclcpp::NodeOptions options);
  void AddComms(std::shared_ptr<Communications> comms);
  void PublishStatus(bool connected, bool running, float litres_remaining);

private:
  void TimerCallback();
  rclcpp::Publisher<example_msgs::msg::Status>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Communications> comms_;
};

#endif  // STATUS_NODE_HPP_
