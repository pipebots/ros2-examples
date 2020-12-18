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

#include "status_subscriber_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "leeds_pump_msgs/msg/leeds_pump_status.hpp"

// Constants.
static const char * kStatusTopicName = "pump/status";
// Interval between each status message (same as node).
static const std::chrono::milliseconds kQueryIntervalMs(500);
// Add a bit on to allow for processing interruptions.
static const std::chrono::milliseconds kProcessingAllowanceMs(100);
static const rclcpp::Duration kCallbackIntervalMs(kQueryIntervalMs + kProcessingAllowanceMs);


StatusSubscriber::StatusSubscriber()
: Node("status_client"),
  connected_(false),
  running_(false),
  litres_remaining_(0.0),
  last_callback_time_(now())
{
  subscription_ = create_subscription<leeds_pump_msgs::msg::LeedsPumpStatus>(
    kStatusTopicName, 10, std::bind(&StatusSubscriber::StatusCallback, this,
    std::placeholders::_1));
}

bool StatusSubscriber::IsServerReady()
{
  bool ready = false;
  rclcpp::Duration difference = now() - last_callback_time_;
  if (difference < kCallbackIntervalMs) {
    ready = true;
  }
  return ready;
}

void StatusSubscriber::StatusCallback(const leeds_pump_msgs::msg::LeedsPumpStatus::SharedPtr msg)
{
  connected_ = msg->connected;
  running_ = msg->running;
  litres_remaining_ = msg->litres_remaining;
  last_callback_time_ = now();
  RCLCPP_DEBUG(
    get_logger(), "%s: connected %d, running %d, litres remaining %f",
    __FUNCTION__, connected_, running_, litres_remaining_);
}
