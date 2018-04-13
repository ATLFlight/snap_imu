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

#include "snap_imu/snap_imu.hpp"

SnapImuDriver::SnapImuDriver(ros::NodeHandle nh)
  : nh_(nh) {

  running_=false;
  snap_imu_man_=nullptr;
}


SnapImuDriver::~SnapImuDriver(){
  CleanUp();
}


bool SnapImuDriver::Start(){
  std::cout << "Start" << std::endl;

  ros::NodeHandle pnh("~"); 
  pnh.param<std::string>("frame_id", frame_id_, "imu");

  GetMonotonicClockOffset();
  advertiseTopics();

  snap_imu_man_ = new Snapdragon::ImuManager();
  snap_imu_man_->AddHandler( this );

  if( snap_imu_man_->Start() != 0 ) {
    ROS_ERROR("ImuManager::Start() failed");
    return false;
  }

  running_=true;
  return true;
}


void SnapImuDriver::Stop(){
  CleanUp();
  running_=false;
}


void SnapImuDriver::Spin() {
  if (running_ && snap_imu_man_ != nullptr) {
    snap_imu_man_->ReadImuData();
  }
}


int32_t SnapImuDriver::CleanUp() {
  if( snap_imu_man_ != nullptr ) {
    snap_imu_man_->RemoveHandler( this );
    snap_imu_man_->Stop();
    delete snap_imu_man_;
    snap_imu_man_ = nullptr;
  }
  return 0;
}


void SnapImuDriver::advertiseTopics(){
  imu_pub_ = nh_.advertise<snap_msgs::ImuArray>("imu_raw_array", 100);
}


void SnapImuDriver::GetMonotonicClockOffset()
{
  // Imu samples are timestamped with the monotonic clock
  // ROS timestamps use the realtime clock.  Here we compute the difference
  // and apply to messages
  // Note: timestamp_in_us is apps monotonic clock, raw_timestamp_in_us is DSP monotonic clock
  // Here, we use timestamp_in_us
  struct timespec time_monotonic;
  struct timespec time_realtime;
  clock_gettime(CLOCK_REALTIME, &time_realtime);
  clock_gettime(CLOCK_MONOTONIC, &time_monotonic);

  ros::Time realtime(time_realtime.tv_sec, time_realtime.tv_nsec);
  ros::Time monotonic(time_monotonic.tv_sec, time_monotonic.tv_nsec);

  monotonic_offset = realtime - monotonic;

  ROS_INFO_STREAM("Monotonic offset: " << monotonic_offset);
}


int32_t SnapImuDriver::Imu_IEventListener_ProcessSamples( sensor_imu* imu_samples, uint32_t sample_count ) {

  int32_t rc = 0;
  const float kNormG = 9.80665f;
  static uint32_t sequence_number_last = 0;
  sensor_msgs::Imu sample;
  sample.header.frame_id = frame_id_;

  snap_msgs::ImuArray array;
  array.header.frame_id = frame_id_;
  array.imu_samples.reserve(sample_count);
  array.sequence_numbers.reserve(sample_count);

  for (int ii = 0; ii < sample_count; ++ii)
  {
    int64_t current_timestamp_ns = (int64_t) imu_samples[ii].timestamp_in_us * 1000;

    int num_dropped_samples = 0;
    if (sequence_number_last != 0)
    {
      // The diff should be 1, anything greater means we dropped samples
      num_dropped_samples = imu_samples[ii].sequence_number
        - sequence_number_last - 1;
      if (num_dropped_samples > 0)
      {
        WARN_PRINT("Current IMU sample = %u, last IMU sample = %u",
            imu_samples[ii].sequence_number, sequence_number_last);
      }
    }
    sequence_number_last = imu_samples[ii].sequence_number;

    // Pack
    sample.header.stamp.fromNSec(current_timestamp_ns);
    sample.header.stamp += monotonic_offset;
    sample.header.seq = imu_samples[ii].sequence_number;

    sample.angular_velocity.x = imu_samples[ii].angular_velocity[0];
    sample.angular_velocity.y = imu_samples[ii].angular_velocity[1];
    sample.angular_velocity.z = imu_samples[ii].angular_velocity[2];

    sample.linear_acceleration.x = imu_samples[ii].linear_acceleration[0] * kNormG;
    sample.linear_acceleration.y = imu_samples[ii].linear_acceleration[1] * kNormG;
    sample.linear_acceleration.z = imu_samples[ii].linear_acceleration[2] * kNormG;

    // Always fill the array timestamp with the most recent IMU sample timestamp
    array.header.stamp = sample.header.stamp;
    array.sequence_numbers.push_back(imu_samples[ii].sequence_number);
    array.imu_samples.push_back(sample);

  }
  imu_pub_.publish(array);
  return rc;
}
