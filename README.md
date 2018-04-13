# snap_imu
#### Qualcomm Flight Board ROS imu package

This ROS package contains node and nodelet imu data publishers for the Qualcomm Flight<sup>TM</sup> board.

### Getting Started

Clone this repo into your catkin_ws on your flight board:

```bash
cd ~/catkin_ws/src
git clone https://github.com/ATLFlight/snap_cpa.git
rosdep install . --from-paths
cd ..
catkin_make
```

Note that you will also need the [snap_msgs](https://github.com/ATLFlight/snap_msgs) package.  snap_imu publishes snap_msgs/ImuArray messages.  snap_imu puts on average 5 sensor_msgs/Imu into an array to make better use of bandwith with [snap_vio](https://github.com/ATLFlight/snap_vio).  snap_imu is probably most useful in conjunction with snap_vio, but can also be run standalone:

```bash
roslaunch snap_imu imu.launch
```
Before running, however, an IMU server needs to be running on the DSP.  This can either be SNAV or the imu_app.  To run without SNAV, first start the imu_app, then run snap_imu:

```bash
sudo su
source /home/linaro/catkin_ws/devel/setup.bash
imu_app -s 2 & # if running on 8096, you will also need to add the -p 10 option to use the correct imu
roslaunch snap_imu imu.launch
```
