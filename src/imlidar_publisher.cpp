
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <imlidar_driver.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imlidar_publisher");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	std::string port;
	int baud_rate;
	std::string frame_id;
	int rps;
	double angle_min;
	double angle_max;
	std::string angle_increment_direction;

	priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
	priv_nh.param("baud_rate", baud_rate, 115200);
	priv_nh.param("frame_id", frame_id, std::string("base_link"));
	priv_nh.param("rps", rps, 7);
	priv_nh.param("angle_min", angle_min, 2 * M_PI);
	priv_nh.param("angle_max", angle_max, 0.0);
	priv_nh.param("angle_increment_direction", angle_increment_direction, std::string("cw"));

	boost::asio::io_service io;

	try {
		imlidar_driver::IMLidar laser(port, baud_rate, io, rps, angle_min, angle_max, angle_increment_direction);
		ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
		laser.start_lidar();

		while (ros::ok()) {
			sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
			scan->header.frame_id = frame_id;
			scan->header.stamp = ros::Time::now();
			if (laser.poll(scan))
			{
				laser_pub.publish(scan);
			}
		}
		laser.close();
		return 0;
	}
	catch (boost::system::system_error ex) {
		ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
		return -1;
	}
}
