# /****************************************************************************
#  *   Copyright (c) 2018 Michael Shomin. All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions
#  * are met:
#  *
#  * 1. Redistributions of source code must retain the above copyright
#  *    notice, this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above copyright
#  *    notice, this list of conditions and the following disclaimer in
#  *    the documentation and/or other materials provided with the
#  *    distribution.
#  * 3. Neither the name ATLFlight nor the names of its contributors may be
#  *    used to endorse or promote products derived from this software
#  *    without specific prior written permission.
#  *
#  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  * POSSIBILITY OF SUCH DAMAGE.
#  *
#  * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
#  ****************************************************************************/
cmake_minimum_required(VERSION 2.8.3)
project(snap_imu)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  snap_msgs
  nodelet
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    snap_msgs
    nodelet
  LIBRARIES snap_imu_nodelet
)

###########
## Build ##
###########


include_directories(include ${catkin_INCLUDE_DIRS} src)

add_library(SnapdragonImuManager
  src/SnapdragonImuManager.cpp
  src/SnapdragonDebugPrint.cpp
)
target_link_libraries(SnapdragonImuManager
  sensor_imu
)

add_library(snap_imu src/snap_imu.cpp)
add_dependencies(snap_imu ${catkin_EXPORTED_TARGETS})
target_link_libraries(snap_imu
  ${catkin_LIBRARIES}
  SnapdragonImuManager
)

add_executable(snap_imu_node src/snap_imu_node.cpp)
add_dependencies(snap_imu_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(snap_imu_node
  ${catkin_LIBRARIES}
  SnapdragonImuManager
  snap_imu
)

add_library(snap_imu_nodelet src/snap_imu_nodelet.cpp)
add_dependencies(snap_imu_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(snap_imu_nodelet
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  SnapdragonImuManager
  snap_imu
)


install(TARGETS SnapdragonImuManager snap_imu snap_imu_node snap_imu_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES snap_imu_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)