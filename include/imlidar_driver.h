
#ifndef IMLIDAR_DRIVER_H_
#define IMLIDAR_DRIVER_H_

#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include <ros/ros.h>
#include <pro.h>
#include <rec.h>
#include <transmit.h>

namespace imlidar_driver {
	class IMLidar {
	public:
		/**
		  * @brief Constructs a new XV11Laser attached to the given serial port
		  * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
		  * @param baud_rate The baud rate to open the serial port at.
		  * @param io Boost ASIO IO Service to use when creating the serial port object
		  */
		IMLidar(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io, uint16_t rps = 7,
			double angle_min = 2 * M_PI, double angle_max = 0.0, std::string angle_increment_direction = "cw");

		/**
		  * @brief Default destructor
		  */
		~IMLidar() {};

		/**
		  * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
		  * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
		  */
		void poll(sensor_msgs::LaserScan::Ptr scan);

		/**
		  * @brief Close the driver down and prevent the polling loop from advancing
		  */
		void close() { shutting_down_ = true; };

		/**
		  * @brief Config lidar speed and start rotation by writing data through serial port to lidar,
		  */
		void start_lidar();

		/**
		  * @brief Config lidar speed and start rotation by writing data through serial port to lidar,
		  */
		void set_lidar_speed();

		/**
		  * @brief send package to lidar, through serial port
		  */
		void send_lidar_cmd(const PackageDataStruct &package_out);

	private:
		std::string port_; ///< @brief The serial port the driver is attached to
		uint32_t baud_rate_; ///< @brief The baud rate for the serial connection

		bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
		boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the imlidar
		uint8_t lidar_rps_; 	//to set the lidar speed, this value should be 1 to 10,the default value is 7Hz

		uint8_t *ptr_data_in_buffer_; 		// to storage the whole lidar input data frame
		PackageDataStruct package_in_; 	// the package, received from lidar
		PackageDataStruct package_out_; 	// to config lidar rotation speed and then  start rotation, the package will be send to lidar through serial port
		LidarDataStructDef *ptr_lidar_data_; // the pure lidar data, contains the usful information
		uint8_t *ptr_data_to_pack_; 		//the pure data(NO headers,etc) we want send to the lidar
		uint8_t *ptr_packed_data_to_lidar_; //the final data(contain headers and tails and checksum) we want send to the lidar
		double angle_min_;			//lidar scan minmum angle ,To adapt to the cartographer default value is 2*PI
		double angle_max_;			//lidar scan maxmum angle ,To adapt to the cartographer default value is 0
		std::string angle_increment_direction_;	//lidar scan angle increment for each scan. To adapt to the cartographer default value is -(2.0*M_PI/360.0)
	};
};

#endif /* IMLIDAR_DRIVER_H_ */