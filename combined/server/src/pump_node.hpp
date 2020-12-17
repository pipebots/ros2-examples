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

#include "leeds_pump_msgs/action/leeds_pump_pump.hpp"
#include "communications.hpp"

class PumpNode : public rclcpp::Node
{
public:
  explicit PumpNode(rclcpp::NodeOptions options);
  void AddComms(Communications * comms);

private:
  rclcpp_action::GoalResponse Goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const leeds_pump_msgs::action::LeedsPumpPump::Goal> goal);
  rclcpp_action::CancelResponse Cancel(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<leeds_pump_msgs::action::LeedsPumpPump>> goal_handle);
  void Accepted(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<leeds_pump_msgs::action::LeedsPumpPump>> goal_handle);
  void Execute(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<leeds_pump_msgs::action::LeedsPumpPump>> goal_handle);

  rclcpp_action::Server<leeds_pump_msgs::action::LeedsPumpPump>::SharedPtr server_;
  Communications * comms_;
};

#endif  // PUMP_NODE_HPP_
