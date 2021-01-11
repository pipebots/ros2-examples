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


#ifndef COMMUNICATIONS_FAKE_HPP_
#define COMMUNICATIONS_FAKE_HPP_

#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "communications.hpp"

/** Handles communications with the pump micro-controller.
 * When an action is started, the getXXX functions are polled regularly and
 * return true when the action is complete.
 */
class CommunicationsFake : public Communications
{
public:
  CommunicationsFake();
  ~CommunicationsFake() override;
  void Init() override;
  bool GetGimbal(int * pitch, int * yaw) const override;
  bool SetGimbal(const int pitch, const int yaw) override;
  bool SetPump(const bool running) override;
  void GetStatus(bool * connected, bool * pump_running, float * litres_remaining) const override;

private:
  // This is here so that the values can be simulated so the tests run.
  struct Status
  {
    bool connected;  // true when Arduino connected.
    bool running;  // true when pump running.  Updated from Arduino.
    float litres_remaining;  // Returned by Arduino.
    Status()
    : connected(false), running(false), litres_remaining(0.0)
    {
    }
  };
  // Pump simulation thread
  void SimulatePump();
  // ROS interface variables
  Status status_;
  // Simulation vars.
  bool simulate_pump_;
};  // CommunicationsFake

#endif  // COMMUNICATIONS_FAKE_HPP_
