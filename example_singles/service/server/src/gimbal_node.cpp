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

#include "gimbal_node.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_msgs/srv/gimbal.hpp"

// Topic/servic/action names.
static const char * kGimbalServiceName = "pump/gimbal";

// The Arduino takes a while to reply to a command.
// This delay is just large enough so that the do/while loop is rarely used.
const std::chrono::milliseconds kCommunicationsDelay(10);


GimbalNode::GimbalNode(rclcpp::NodeOptions options)
: Node("gimbal_server", options)
{
  server_ =
    create_service<example_msgs::srv::Gimbal>(
    kGimbalServiceName,
    std::bind(
      &GimbalNode::HandleService, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void GimbalNode::HandleService(
  const std::shared_ptr<rmw_request_id_t>/* request_header */,
  const std::shared_ptr<example_msgs::srv::Gimbal::Request> request,
  const std::shared_ptr<example_msgs::srv::Gimbal::Response> response)
{
  RCLCPP_DEBUG(
    get_logger(), "%s: setting pitch %d, yaw %d",
    __func__, request->pitch, request->yaw);
#if 1
  // Demo code.

  // Wait for a time to simulate delay in response.
  rclcpp::sleep_for(kCommunicationsDelay);
  // Set fixed values
  response->pitch = 90;
  response->yaw = 89;
#else
  // This is an example of what you might add.

  // Send command with pitch and yaw values.
  bool result = comms_->SetGimbal(request->pitch, request->yaw);
  if (result) {
    // Wait for the reply.
    int pitch = 0;
    int yaw = 0;
    bool success = false;
    do {
      rclcpp::sleep_for(kCommunicationsDelay);
      success = comms_->GetGimbal(&pitch, &yaw);
    } while (!success && rclcpp::ok());
    // Set response.
    response->pitch = pitch;
    response->yaw = yaw;
  } else {
    // Set error value.
    response->pitch = response->yaw = -1000;
  }
#endif
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(GimbalNode)
