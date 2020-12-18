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

#include "pump_action_client.hpp"
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "leeds_pump_msgs/action/leeds_pump_pump.hpp"

// Names.
static const char * kPumpActionName = "pump/pump";

PumpActionClient::PumpActionClient()
: Node("pump_action_client"),
  goal_accepted_(false),
  goal_responded_(false)
{
  client_ = rclcpp_action::create_client<leeds_pump_msgs::action::LeedsPumpPump>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    kPumpActionName);
}

int PumpActionClient::IsServerReady(const std::chrono::milliseconds & wait_time_ms)
{
  int result = -1;
  if (rclcpp::ok()) {
    bool server_ready = client_->wait_for_action_server(wait_time_ms);
    result = server_ready ? 1 : 0;
  }
  return result;
}

void PumpActionClient::SendGoal(float litres_to_pump, callback_t callback_function)
{
  if (!client_) {
    RCLCPP_ERROR(get_logger(), "Action client not initialized");
  }
  goal_responded_ = false;
  goal_accepted_ = false;
  callback_ = callback_function;
  auto goal_msg = leeds_pump_msgs::action::LeedsPumpPump::Goal();
  goal_msg.litres_to_pump = litres_to_pump;
  RCLCPP_INFO(get_logger(), "Sending goal");
  auto send_goal_options =
    rclcpp_action::Client<leeds_pump_msgs::action::LeedsPumpPump>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&PumpActionClient::GoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&PumpActionClient::FeedbackCallback, this, std::placeholders::_1,
      std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&PumpActionClient::ResultCallback, this, std::placeholders::_1);
  auto goal_handle_future = client_->async_send_goal(goal_msg, send_goal_options);
}

bool PumpActionClient::SendGoalWait(float litres_to_pump, callback_t callback_function)
{
  const std::chrono::milliseconds kWaitTimeMs(100);
  // goal_responded_ and goal_accepted_ set false by SendGoal.
  SendGoal(litres_to_pump, callback_function);
  // Wait until the response is received.  The response updates goal_accepted_.
  while (!goal_responded_) {
    if (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kWaitTimeMs));
    } else {
      break;
    }
  }
  return goal_responded_ && goal_accepted_;
}

void PumpActionClient::GoalResponseCallback(
  std::shared_future<
    rclcpp_action::ClientGoalHandle<leeds_pump_msgs::action::LeedsPumpPump>::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    goal_accepted_ = true;
  }
  // This is set last so that SendGoalWait so the while loop stops only when
  // goal_accepted_ has been set.
  goal_responded_ = true;
}

void PumpActionClient::FeedbackCallback(
  rclcpp_action::ClientGoalHandle<leeds_pump_msgs::action::LeedsPumpPump>::SharedPtr,
  const std::shared_ptr<const leeds_pump_msgs::action::LeedsPumpPump::Feedback> feedback)
{
  float litres_pumped = feedback->litres_pumped;
  RCLCPP_INFO(get_logger(), "Feedback: Pumped %fl", litres_pumped);
  Notify(false, litres_pumped);
}

void PumpActionClient::ResultCallback(
  const rclcpp_action::ClientGoalHandle<
    leeds_pump_msgs::action::LeedsPumpPump>::WrappedResult & result)
{
  float litres_pumped = result.result->litres_pumped;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Total pumped %fl", litres_pumped);
      Notify(true, litres_pumped);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      break;
  }
}

void PumpActionClient::Notify(bool done, float litres_pumped)
{
  callback_(done, litres_pumped);
}
