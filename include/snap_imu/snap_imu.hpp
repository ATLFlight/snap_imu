/****************************************************************************
 *   Copyright (c) 2018 Michael Shomin. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
 ****************************************************************************/
#ifndef SNAP_IMU_HPP_
#define SNAP_IMU_HPP_

#include "SnapdragonImuManager.hpp"
#include "SnapdragonDebugPrint.h"

#include <iostream>
#include <chrono>
#include <thread>

#include <ros/ros.h>


#include <snap_msgs/ImuArray.h>

class SnapImuDriver : public Snapdragon::Imu_IEventListener {
public:
  SnapImuDriver(ros::NodeHandle nh);
  ~SnapImuDriver();

  bool Start();
  void Stop();
  void Spin();

  void advertiseTopics();

  /**
   * Compute difference between monotonic and realtime clock
   */
  void GetMonotonicClockOffset();

  /**
   * The IMU callback handler to add the accel/gyro data into VISLAM.
   * @param samples
   *  The imu Samples to be added to VISLAM.
   * @param count
   *  The number of samples in the buffer.
   * @return int32_t
   *  0 = success;
   * otherwise = false;
   **/
  int32_t Imu_IEventListener_ProcessSamples( sensor_imu* samples, uint32_t count );

  Snapdragon::ImuManager * snap_imu_man_;

protected:

  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  ros::Publisher imu_std_pub_;
  bool running_;
  ros::Duration monotonic_offset;

private:
  int32_t CleanUp();
  std::string frame_id_;
};

#endif // SNAP_IMU_HPP_
