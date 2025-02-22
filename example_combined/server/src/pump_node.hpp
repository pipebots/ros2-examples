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

#ifndef PUMP_NODE_HPP_
#define PUMP_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "example_msgs/action/pump.hpp"
#include "communications.hpp"

class PumpNode : public rclcpp::Node
{
public:
  explicit PumpNode(rclcpp::NodeOptions options);
  void AddComms(std::shared_ptr<Communications> comms);

private:
  rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const example_msgs::action::Pump::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<example_msgs::action::Pump>> goal_handle);
  void HandleAccepted(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<example_msgs::action::Pump>> goal_handle);
  void Execute(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<example_msgs::action::Pump>> goal_handle);

  rclcpp_action::Server<example_msgs::action::Pump>::SharedPtr server_;
  std::shared_ptr<Communications> comms_;
};

#endif  // PUMP_NODE_HPP_
