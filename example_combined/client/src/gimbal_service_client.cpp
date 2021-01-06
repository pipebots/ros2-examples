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

#include "gimbal_service_client.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_msgs/srv/gimbal.hpp"

// Topic/service/action names.
static const char * kGimbalServiceNodeName = "gimbal_client_node";
static const char * kGimbalServiceName = "pump/gimbal";

GimbalServiceClient::GimbalServiceClient()
{
  node_ = rclcpp::Node::make_shared(kGimbalServiceNodeName);
  client_ = node_->create_client<example_msgs::srv::Gimbal>(kGimbalServiceName);
  // Uncomment for debug logging.
  auto ret = rcutils_logging_set_logger_level(
    node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  (void) ret;
}

GimbalServiceClient::~GimbalServiceClient()
{
  node_ = nullptr;
}

int GimbalServiceClient::IsServerReady(const std::chrono::milliseconds & wait_time_ms)
{
  RCLCPP_DEBUG(node_->get_logger(), "%s: called", __FUNCTION__);
  int result = -1;
  if (rclcpp::ok()) {
    bool server_ready = client_->wait_for_service(wait_time_ms);
    result = server_ready ? 1 : 0;
  }
  return result;
}

int GimbalServiceClient::Move(int * pitch, int * yaw)
{
  RCLCPP_DEBUG(node_->get_logger(), "%s: setting pitch %d, yaw %d", __FUNCTION__, *pitch, *yaw);
  int success = 0;
  // Send request.
  auto request = std::make_shared<example_msgs::srv::Gimbal::Request>();
  request->pitch = *pitch;
  request->yaw = *yaw;
  auto result_future = client_->async_send_request(request);
  RCLCPP_DEBUG(node_->get_logger(), "%s: sent request", __FUNCTION__);
  if (rclcpp::spin_until_future_complete(node_, result_future) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    RCLCPP_DEBUG(node_->get_logger(), "%s: response %d, yaw %d", __FUNCTION__, *pitch, *yaw);
    // Set return values.
    *pitch = result->pitch;
    *yaw = result->yaw;
    success = 1;
  } else {
    RCLCPP_WARN(node_->get_logger(), "%s: Failed to call service", __FUNCTION__);
  }
  return success;
}
