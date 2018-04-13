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
#include "SnapdragonImuManager.hpp"
#include "SnapdragonDebugPrint.h"

Snapdragon::ImuManager::ImuManager() {
  initialized_ = false;
  imu_listener_ = nullptr;
  imu_api_handle_ptr_ = nullptr;
}


int32_t Snapdragon::ImuManager::SetupImuApi() {
  int16_t api_rc = 0;
  //get the imu_api handle first.
  imu_api_handle_ptr_ = sensor_imu_attitude_api_get_instance();
  if( imu_api_handle_ptr_ == nullptr ) {
    //error
    ERROR_PRINT( "Error getting the IMU API handler." );
    return -1;
  }

  // check the version is the correct one.
  char* api_version = sensor_imu_attitude_api_get_version( imu_api_handle_ptr_ );
  std::string str_version( api_version );
  std::string str_expected_version( SENSOR_IMU_AND_ATTITUDE_API_VERSION );
  if( str_version != str_expected_version ) {
    ERROR_PRINT( "Error: Imu API version Mismatch.  Make sure to link against the correct version(%s)",
      str_expected_version.c_str() );
    return -2;
  }

  //the API' are good. Call the API's initialize method.
  api_rc = sensor_imu_attitude_api_initialize( imu_api_handle_ptr_, SENSOR_CLOCK_SYNC_TYPE_MONOTONIC );
  if( api_rc != 0 ) {
    ERROR_PRINT( "Error calling the IMU API's initialize method api_rc(%d)", api_rc );
    return api_rc;
  }

  api_rc = sensor_imu_attitude_api_wait_on_driver_init( imu_api_handle_ptr_ );
  if( api_rc != 0 ) {
    ERROR_PRINT( "Error calling the IMU API for driver init completion(%d)", api_rc );
    return api_rc;
  }
  return api_rc;
}


int32_t Snapdragon::ImuManager::Start() {

  if( SetupImuApi() != 0 ) {
    ERROR_PRINT( "Error initializing the IMU API's; exiting." );
    return -1;
  }

  initialized_ = true;
  return 0;
}


int32_t Snapdragon::ImuManager::Stop() {
  initialized_ = false;
  sensor_imu_attitude_api_terminate( imu_api_handle_ptr_ );
  imu_api_handle_ptr_ = nullptr;
  return 0;
}


int32_t Snapdragon::ImuManager::AddHandler( Snapdragon::Imu_IEventListener* handler ) {
  imu_listener_ = handler;
  return 0;
}


int32_t Snapdragon::ImuManager::RemoveHandler( Snapdragon::Imu_IEventListener* handler ) {
  imu_listener_ = nullptr;
  return 0;
}


void Snapdragon::ImuManager::ReadImuData() {

  sensor_imu imu_buffer[ 1000 ];
  int32_t returned_sample_count = 0;
  uint32_t current_sequence_number;
  static uint32_t prev_sequence_number = 0;

  int16_t api_rc = sensor_imu_attitude_api_get_imu_raw( imu_api_handle_ptr_, imu_buffer, 1000, &returned_sample_count );
  if( api_rc != 0 ) {
    WARN_PRINT( "WARN: Error getting imu samples from imu api(%d)", api_rc );
  }
  else {
    if( returned_sample_count > 0 ) {
      current_sequence_number = imu_buffer[0].sequence_number;
      if( prev_sequence_number != 0 && prev_sequence_number + 1 != current_sequence_number ) {
        WARN_PRINT( "Missed IMU Samples: Expected:(%u) Got(%u) sample count: (%d)",
          (prev_sequence_number+1), current_sequence_number, returned_sample_count );
      }
      prev_sequence_number = imu_buffer[returned_sample_count-1].sequence_number;

      // call the handlers.
      if (imu_listener_ != nullptr) {
        imu_listener_->Imu_IEventListener_ProcessSamples( imu_buffer, returned_sample_count );
      }
    }
  }
}

