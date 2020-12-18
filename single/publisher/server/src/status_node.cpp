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

#include "status_node.hpp"
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "leeds_pump_msgs/msg/leeds_pump_status.hpp"


// Interval between each status message.
static const std::chrono::milliseconds kQueryIntervalMs(500);
// Topic names.
static const char * kStatusTopicName = "pump/status";

StatusNode::StatusNode(rclcpp::NodeOptions options)
: Node("status_node", options), timer_call_count_(0)
{
  RCLCPP_DEBUG(this->get_logger(), "%s: Called", __FUNCTION__);
  publisher_ = create_publisher<leeds_pump_msgs::msg::LeedsPumpStatus>(kStatusTopicName, 10);
  // Connect TimerCallback() to a timer to publish the message every kQueryIntervalMs.
  timer_ = create_wall_timer(kQueryIntervalMs, std::bind(&StatusNode::TimerCallback, this));
}

void StatusNode::PublishStatus(bool connected, bool running, float litres_remaining)
{
  RCLCPP_DEBUG(
    this->get_logger(), "%s: Called, connected to %d subscribers\n",
    __FUNCTION__, publisher_->get_subscription_count());
  auto message = leeds_pump_msgs::msg::LeedsPumpStatus();
  message.connected = connected;
  message.running = running;
  message.litres_remaining = litres_remaining;
  publisher_->publish(message);
}

void StatusNode::TimerCallback()
{
  RCLCPP_DEBUG(
    this->get_logger(), "%s: Called, connected to %d subscribers\n",
    __FUNCTION__, publisher_->get_subscription_count());
  bool connected = false;
  bool pump_running = false;
  float litres_remaining = 0.0;
#if 1  // This is demo code.
  // Call a function here to update the values for the message.
  // Show we are connected.
  connected = true;
  // To show something changing, increment a call count and report it using
  // litres_remaining.
  ++timer_call_count_;
  litres_remaining = static_cast<float>(timer_call_count_);
#endif
  // Now publish the message.
  PublishStatus(connected, pump_running, litres_remaining);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(StatusNode)
