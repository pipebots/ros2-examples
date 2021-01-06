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

#ifndef STATUS_SUBSCRIBER_CLIENT_HPP_
#define STATUS_SUBSCRIBER_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "example_msgs/msg/status.hpp"

/**
 * @brief Listens to Leeds pump status messages and stores the last reported values.
 */
class StatusSubscriber : public rclcpp::Node
{
public:
  StatusSubscriber();
  bool connected() {return connected_;}
  bool running() {return running_;}
  float litres_remaining() {return litres_remaining_;}
  bool IsServerReady();

private:
  void StatusCallback(const example_msgs::msg::Status::SharedPtr msg);
  rclcpp::Subscription<example_msgs::msg::Status>::SharedPtr subscription_;
  bool connected_;
  bool running_;
  float litres_remaining_;
  rclcpp::Time last_callback_time_;
};

#endif   // STATUS_SUBSCRIBER_CLIENT_HPP_
