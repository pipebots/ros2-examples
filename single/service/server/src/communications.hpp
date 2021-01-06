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


#ifndef COMMUNICATIONS_HPP_
#define COMMUNICATIONS_HPP_

/** Base class for communications with the pump micro-controller.
 * This allows different implementations including a fake version for testing.
 */
class Communications
{
public:
  Communications() {}
  virtual ~Communications() {}

  /**
   * @brief Initialise the instance.
   */
  virtual void Init() = 0;

  /** Gets the last reported value of the gimbal.
   * @param pitch The pitch value in degrees.
   * @param yaw The yaw value in degrees.
   * @return true if the action has been completed.
   */
  virtual bool GetGimbal(int * pitch, int * yaw) const = 0;

  /** Set the pitch and yaw values for the gimbal.
   * @param pitch The pitch value in degrees.
   * @param yaw The yaw value in degrees.
   * @return true if command sent or nothing to do.
   */
  virtual bool SetGimbal(const int pitch, const int yaw) = 0;

  /** Start or stop the pump.
   * @param running true to start the pump, false to stop.
   * @return true if command sent, false if the write failed e.g. not connected.
   */
  virtual bool SetPump(const bool running) = 0;

  /**
   * @brief Updates the given values with the most recently set values.
   *
   * @param connected true if the micro-controller is connected.
   * @param running true when the pump is running.
   * @param litres_remaining Number of litres remaining.
   * @note This function allows the parameters to be set to nullptr if the value
   * is not needed.
   */
  virtual void GetStatus(bool * connected, bool * pump_running, float * litres_remaining) const = 0;
};  // Communications

#endif  // COMMUNICATIONS_HPP_
