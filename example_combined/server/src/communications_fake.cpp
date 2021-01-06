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


#include "communications_fake.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"


CommunicationsFake::CommunicationsFake()
{
  // Need to use the result to prevent "warning: ignoring return value of".
  // '(void) function' doesn't work!  Probably due to -Wpedantic being enabled.
  rcutils_ret_t result = rcutils_logging_set_logger_level("Comms", RCUTILS_LOG_SEVERITY_DEBUG);
  (void) result;
}

CommunicationsFake::~CommunicationsFake()
{
}

void CommunicationsFake::Init()
{
  RCUTILS_LOG_DEBUG_NAMED("Comms", "Called: %s", __FUNCTION__);
}

bool CommunicationsFake::GetGimbal(int * pitch, int * yaw) const
{
  RCUTILS_LOG_DEBUG_NAMED("Comms", "Called: %s", __FUNCTION__);
  *pitch = 0.0;
  *yaw = 0.0;
  bool result = false;
  return result;
}

bool CommunicationsFake::SetGimbal(const int pitch, const int yaw)
{
  RCUTILS_LOG_DEBUG_NAMED("Comms", "Called: %s, pitch %d, yaw %d.", __FUNCTION__, pitch, yaw);
  bool result = false;
  return result;
}

bool CommunicationsFake::SetPump(const bool running)
{
  RCUTILS_LOG_DEBUG_NAMED("Comms", "Called: %s, running %d", __FUNCTION__, running);
  bool result = false;
  return result;
}

void CommunicationsFake::GetStatus(
  bool * connected, bool * pump_running,
  float * litres_remaining) const
{
  RCUTILS_LOG_DEBUG_NAMED("Comms", "Called: %s", __FUNCTION__);
  *connected = true;
  *pump_running = false;
  *litres_remaining = 1.0;
}
