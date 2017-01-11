/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Inmotion Robot, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Inmotion Robot nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <imlidar_driver.h>

void SigintHandler(int sig)
{
	/* Acquire lidar */
	ros::NodeHandle priv_nh("~");
	std::string port;
	int baud_rate;
	priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
	priv_nh.param("baud_rate", baud_rate, 115200);
	boost::asio::io_service io;
	imlidar_driver::IMLidar lidar(port, baud_rate, io);

	lidar.close();
	ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imlidar_publisher");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	/* Sigal interrupt callback for ctrl+c */
	signal(SIGINT,SigintHandler);
	
	std::string port;
	int baud_rate;
	std::string frame_id;
	int rps;
	std::string data_sequence_direction;

	priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
	priv_nh.param("baud_rate", baud_rate, 115200);
	priv_nh.param("frame_id", frame_id, std::string("imlidar"));
	priv_nh.param("rps", rps, 8);
	priv_nh.param("data_sequence_direction", data_sequence_direction, std::string("cw"));

	/* Check parameters */
	if (rps < 0 || rps > 10)
	{
		ROS_ERROR("Please check the rps value in [0,10]?");
		return -1;
	}
	if (data_sequence_direction != "ccw" && data_sequence_direction != "cw")
	{
		ROS_ERROR("Please check the data_sequence_direction value, is cw or ccw?");
		return -1;
	}

	boost::asio::io_service io;
	try {
 		imlidar_driver::IMLidar lidar(port, baud_rate, io, rps, data_sequence_direction);
		ros::Publisher lidar_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
		lidar.start_lidar();

		sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
		while (ros::ok()) {
			scan->header.frame_id = frame_id;
			scan->header.stamp = ros::Time::now();
			lidar.poll(scan)
			lidar_pub.publish(scan);
		}
		return 0;
	}
	catch (boost::system::system_error ex) {
		ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
		return -1;
	}
}

/************************* END OF FILE *******************************/