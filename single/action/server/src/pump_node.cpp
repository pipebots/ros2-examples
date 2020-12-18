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

#include "pump_node.hpp"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "leeds_pump_msgs/action/leeds_pump_pump.hpp"

// Consts
// Rate of looping. 2.0 = 500ms per loop.
#define UPDATE_RATE_PER_S (2.0)
// There is a small but significant delay when communicating with the Arduino.
static const std::chrono::milliseconds kCommunicationsDelay(10);

// Action names.
static const char * kPumpActionName = "pump/pump";


PumpNode::PumpNode(rclcpp::NodeOptions options)
: Node("pump_node", options)
{
  server_ = rclcpp_action::create_server<
    leeds_pump_msgs::action::LeedsPumpPump>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    kPumpActionName,
    std::bind(&PumpNode::Goal, this,
    std::placeholders::_1, std::placeholders::_2),
    std::bind(&PumpNode::Cancel, this, std::placeholders::_1),
    std::bind(&PumpNode::Accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse PumpNode::Goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const leeds_pump_msgs::action::LeedsPumpPump::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal request: %f litres", goal->litres_to_pump);
  (void)uuid;
  // Reject if not enough volume remaining or not connected.
  bool connected = false;
  float litres_remaining = 0.0;
  RCLCPP_INFO(get_logger(), "AJB: GetStatus: connected %d litres %f", connected, litres_remaining);
  // Add a bit on to make sure that float rounding errors don't affect functionality.
  if (!connected || (litres_remaining + 0.1) < goal->litres_to_pump) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PumpNode::Cancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<leeds_pump_msgs::action::LeedsPumpPump>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PumpNode::Accepted(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<leeds_pump_msgs::action::LeedsPumpPump>> goal_handle)
{
  // Spin up a new thread to avoid blocking the caller.
  std::thread{std::bind(&PumpNode::Execute, this, std::placeholders::_1), goal_handle}.detach();
}

void PumpNode::Execute(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<leeds_pump_msgs::action::LeedsPumpPump>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(UPDATE_RATE_PER_S);
  auto feedback = std::make_shared<leeds_pump_msgs::action::LeedsPumpPump::Feedback>();
  auto result = std::make_shared<leeds_pump_msgs::action::LeedsPumpPump::Result>();
  float litres_at_start = 0.0;
  float litres_to_pump = goal_handle->get_goal()->litres_to_pump;
  float litres_remaining = 0.0;
  float litres_pumped = 0.0;
  bool connected = false;
  bool running = false;

  // Start pump.
#if 1
  RCLCPP_INFO(get_logger(), "TODO something useful!");
#else
  if (connected) {
    if (connected) {
      // Wait for pump to start.
      do {
        rclcpp::sleep_for(kCommunicationsDelay);
      } while (!running && rclcpp::ok());

      // Run feedback loop.
      while (rclcpp::ok()) {
        // Cancelled?
        if (goal_handle->is_canceling()) {
          break;
        }
        // Update status.
        if (!connected) {
          break;
        }
        litres_pumped = litres_at_start - litres_remaining;
        // Send feedback.
        feedback->litres_pumped = litres_pumped;
        goal_handle->publish_feedback(feedback);
        // Has the pump stopped?
        if (!running) {
          break;
        }
        // Has enough water been pumped?
        if (litres_pumped >= litres_to_pump) {
          break;
        }
        // Sleep until next update.  Exits if rclcpp:ok() == false.
        loop_rate.sleep();
      }

      // Stop the pump if running.
      if (running) {
        do {
          rclcpp::sleep_for(kCommunicationsDelay);
        } while (running && connected && rclcpp::ok());
      }
    }
  }

  // Report final status.
  if (!connected) {
    result->litres_pumped = litres_pumped;
    goal_handle->abort(result);
    RCLCPP_INFO(get_logger(), "Goal aborted (no connection).");
  } else {
    litres_pumped = litres_at_start - litres_remaining;
    result->litres_pumped = litres_pumped;
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Goal cancelled.");
    } else {
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeeded.");
    }
  }
#endif
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(PumpNode)
