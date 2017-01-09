# Imlidar Driver for ROS
## Overview
IMLIDAR is a low cost LIDAR sensor suitable for indoor robotic SLAM application. It provides 360 degree scan field and supports 3600 samples per second, 0~10hz rotating frequency with guaranteed 0.15~10 meter ranger distance.  
This is the documentation of a driver for the IMLIDAR ILD26TRI laser range finder.  
The driver is based upon the widespread boost asio library (<http://www.boost.org>) and publishes device-dependent sensor_msgs/LaserScan data.  
Website: https://robot.imscv.com  
Email: support.robot@imscv.com

## Version
0.2.0  

## Getting started
### Building and installation 
1. Install wstool  
`sudo apt-get update`  
`sudo apt-get install -y python-wstool`  
2. Create a new workspace in 'catkin_ws'  
`mkdir catkin_ws`  
`cd catkin_ws`  
`wstool init src`  
3. Fetch code  
`cd src`  
`git clone https://github.com/wenhust/imlidar_driver.git`  
4. Build and install  
`cd ..`  
`catkin_make`  
`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`  
`source ~/.bashrc`  

### Run imlidar_driver
1. Qiuck start  
`roscore`  
`rosrun imlidar_driver imlidar_node`  
Otherwise£¬if require to configure parameters£º  
`roslaunch imlidar_driver imlidar.launch`  
**ATTENTION**  
If prompted to open the serial port fails, the serial port permission may be not enough.Check the permission of serial port:(such as /dev/ttyUSB0)  
`ls -l /dev/ttyUSB0`  
If show "crw-rw----"£¬to add permission make it appear as "crw-rw-rw-"(such as /dev/ttyUSB0)  
`sudo chmod a+rw /dev/ttyUSB0`  
Use the `ls /dev` command can view port, the sample serial port is `ttyUSB0`
3. View and test  
`roslaunch imlidar_driver view_imlidar.launch`  
This starts RViz (http://wiki.ros.org/rviz) and you should see the measuring output of the scanner.

### Parameters  
If necessary, parameters in the file launch/imlidar.launch can be changed.  
* `port` (string, default: /dev/ttyUSB0)  
The device path  
* `baud_rate` (int, default: 115200)  
The baud rate to receive lidar data. Should not be changed from 115200 unless the lidar's baud rate is changed (by a firmware upgrade, for example).  
* `frame_id` (string, default: imlidar)  
The lidar data frame. This frame should be at the optical center of the lidar, with zero angle being forward along with the x-axis, and angle increse along with data_sequence_direction.(If data_sequence_direction is cw, then the y-axis along the 3*pi/2 degree ray. If data_sequence_direction is ccw, then the y-axis along the pi/2 degree ray.) 
* `rps` (int, default: 8)  
The scan frequency (revolutions per second) in Hz in the range [0,10].  
* `data_sequence_direction` (string, default: cw)  
Lidar data sequence increment direction.This direction would be changed according to the actual position that lidar is placed.

### Published Topics
* scan (sensor_msgs/LaserScan)  
Scan data from the laser. Under normal conditions, contains 360 pings at 1 degree increments.