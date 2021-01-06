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

#ifndef PUMP_ACTION_CLIENT_HPP_
#define PUMP_ACTION_CLIENT_HPP_

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_msgs/action/pump.hpp"


class PumpActionClient : public rclcpp::Node
{
public:
  typedef std::function<void (bool done, float litres_pumped)> callback_t;

  PumpActionClient();

  /**
   * @brief Waits for the server to be ready for a maximum of the given time.
   * @param wait_time_ms Time in milliseconds.
   * @return int 1 = server ready, -1 = termination requested,
   * 0 server not ready/timeout.
   */
  int IsServerReady(const std::chrono::milliseconds & wait_time_ms);

  /**
   * @brief Tell the pump to pump the given number of litres.
   * @param litres_to_pump The number of number of litres to pump.
   * @param callback_function std::function to call with updates.
   * @note If the goal is accepted, the callback will be called
   * multiple times with feedback and goal done information.
   */
  void SendGoal(float litres_to_pump, callback_t callback_function);

  /**
   * @brief Tell the pump to pump the given number of litres.
   * This function blocks until the goal response is received.
   * @param litres_to_pump The number of number of litres to pump.
   * @param callback_function std::function to call with updates.
   * @return true if goal accepted.
   * @note If the goal is accepted, the callback will be called
   * multiple times with feedback and goal done information.
   */
  bool SendGoalWait(float litres_to_pump, callback_t callback_function);

private:
  void GoalResponseCallback(
    std::shared_future<
      rclcpp_action::ClientGoalHandle<example_msgs::action::Pump>::SharedPtr> future);
  void FeedbackCallback(
    rclcpp_action::ClientGoalHandle<example_msgs::action::Pump>::SharedPtr,
    const std::shared_ptr<const example_msgs::action::Pump::Feedback> feedback);
  void ResultCallback(
    const rclcpp_action::ClientGoalHandle<
      example_msgs::action::Pump>::WrappedResult & result);
  void Notify(bool done, float litres_pumped);

  rclcpp_action::Client<example_msgs::action::Pump>::SharedPtr client_;
  callback_t callback_;
  bool goal_accepted_;
  bool goal_responded_;
};  // class PumpActionClient

#endif  // PUMP_ACTION_CLIENT_HPP_
