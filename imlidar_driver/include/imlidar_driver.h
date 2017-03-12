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

//define system parameters
#define SCAN_RESOLUTION		(360)
#define ANGLE_MIN			(0.0)
#define ANGLE_MAX			(2.0 * M_PI)
#define ANGLE_INCREMENT		(2.0 * M_PI / SCAN_RESOLUTION)
#define RANGE_MIN			(0.15)
#define RANGE_MAX			(10.0)

namespace imlidar_driver {
	class IMLidar {
	public:
		/**
		  * @brief Constructs a new IMLidar attached to the given serial port
		  * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
		  * @param baud_rate The baud rate to open the serial port at, e.g. "115200"
		  * @param io Boost ASIO IO Service to use when creating the serial port object
		  * @param rps The revolutions per second(rps) to set the lidar at, e.g. "8"
		  * @param angle_increment_direction The angle increment direction between measurements, e.g. "cw"
		  */
		IMLidar(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io,
			uint16_t rps = 8, std::string data_sequence_direction = "cw", int mount_angle_ccw_postive = 0);

		/**
		  * @brief Default destructor
		  */
		~IMLidar() {};

		/**
		* @brief Config lidar and start rotation by writing data through serial port to lidar
		*/
		void start_lidar();

		/**
		* @brief Set lidar speed by writing data through serial port to lidar
		*/
		void set_lidar_speed(uint8_t lidar_rps);

		/**
		* @brief Send package to lidar through serial port
		*/
		void send_lidar_cmd(const PackageDataStruct &package_out);

		/**
		  * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
		  * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
		  */
		void poll(sensor_msgs::LaserScan::Ptr scan);

		/**
		  * @brief Close the driver down and prevent the polling loop from advancing
		  */
		void close();

	private:
		std::string port_;						// @brief The serial port the driver is attached to
		uint32_t baud_rate_;					// @brief The baud rate for the serial connection
		bool shutting_down_;					// @brief Flag for whether the driver is supposed to be shutting down or not
		boost::asio::serial_port serial_;		// @brief Actual serial port object for reading/writing to the imlidar
		uint8_t lidar_rps_;						// @brief The revolutions per second(rps) to set the lidar at, value range[0,10], default 8rps
		std::string data_sequence_direction_;	// @brief The angle increment direction between measurements
		int mount_angle_ccw_postive_;
		int offset_angle_;
		
		uint8_t *ptr_data_in_buffer_; 			// @brief The pointer that point the whole lidar input data frame
		uint32_t data_in_buffer_len_cnt_;		// @brief The length counter to the pointer *ptr_data_in_buffer_
		PackageDataStruct package_in_;			// @brief The package that received from lidar
		PackageDataStruct package_out_;			// @brief The package that will be sent to lidar
		LidarDataStructDef *ptr_lidar_data_;	// @brief The pointer that point the pure lidar data(no headers etc)
		uint8_t *ptr_data_to_pack_; 			// @brief The pointer that point the pure data(no headers etc) which will be packed and sent to the lidar
		uint8_t *ptr_packed_data_to_lidar_;		// @brief The pointer that point the packed data(contain headers etc) which was packed and will be sent to the lidar
	};
};

#endif /* IMLIDAR_DRIVER_H_ */

/************************* END OF FILE *******************************/