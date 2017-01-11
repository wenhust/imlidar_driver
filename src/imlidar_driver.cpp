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
#include <imlidar_driver.h>

namespace imlidar_driver {
	IMLidar::IMLidar(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io,
		uint16_t rps, std::string data_sequence_direction) :
		port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_),
		lidar_rps_(rps), data_sequence_direction_(data_sequence_direction) {

		/* Config the serial port parameters */
		serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
		serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
		serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		serial_.set_option(boost::asio::serial_port_base::character_size(8));

		/* Init the pointer */
		ptr_data_in_buffer_ = new uint8_t[PARSE_LEN];
		ptr_packed_data_to_lidar_ = new uint8_t[20];
		ptr_data_to_pack_ = new uint8_t[20];
		package_out_.DataOutLen = new uint32_t;
		package_in_.DataOutLen = new uint32_t;
		data_in_buffer_len_cnt_ = 0;

		memset(ptr_data_in_buffer_, 0, sizeof(ptr_data_in_buffer_));
		memset(ptr_packed_data_to_lidar_, 0, sizeof(ptr_packed_data_to_lidar_));
		memset(ptr_data_to_pack_, 0, sizeof(ptr_data_to_pack_));
		memset(package_out_.DataOutLen, 0, sizeof(package_out_.DataOutLen));
		memset(package_in_.DataOutLen, 0, sizeof(package_in_.DataOutLen));
	}
	
	void IMLidar::start_lidar() {

		ResultTypeDef result;
		
		memset(ptr_data_to_pack_, 0, sizeof(ptr_data_to_pack_));
		ptr_data_to_pack_[0] = 1;		//"start" cmd. This byte should be 1
		package_out_.DataID = PACK_START_ROTATE;
		package_out_.DataInBuff = ptr_data_to_pack_;
		package_out_.DataInLen = 2;		//the length of "start" cmd should be 2, the length and cmd value(first byte 1, second byte 0)
		package_out_.DataOutBuff = ptr_packed_data_to_lidar_;
		
		result = Package(package_out_);	//pack the data

		if (result != PACK_FAIL) {
			send_lidar_cmd(package_out_);
		}
		else {
			ROS_ERROR("Start imlidar error!");
			return;
		}

		/* We need to delay for a while after config start */
		usleep(1000 * 100);
		set_lidar_speed(lidar_rps_);
	}

	void IMLidar::set_lidar_speed(uint8_t lidar_rps) {

		ResultTypeDef result;
		memset(ptr_data_to_pack_, 0, sizeof(ptr_data_to_pack_));			//clear the buffer
		/* Config the lidar rotation speed parameters */
		((uint16_t*)ptr_data_to_pack_)[0] = lidar_rps * SCAN_RESOLUTION;	//this byte should be the rotation speed of lidar
		package_out_.DataID = PACK_SET_SPEED;
		package_out_.DataInBuff = ptr_data_to_pack_;
		package_out_.DataInLen = 2;											//the length of "SET_SPEED" cmd should be 2
		package_out_.DataOutBuff = ptr_packed_data_to_lidar_;

		result = Package(package_out_);							//pack the data
		if (result != PACK_FAIL) {
			send_lidar_cmd(package_out_);						//send the command package to lidar
		}
		else {
			ROS_ERROR("Set speed of imlidar error!");
			return;
		}
	}

	void IMLidar::send_lidar_cmd(const PackageDataStruct &package_out) {
		boost::system::error_code ec;
		try {
			/* Send data through serial port */
			boost::asio::write(serial_, boost::asio::buffer(package_out.DataOutBuff, *package_out.DataOutLen), ec);
		}
		catch (boost::system::error_code ec) {
			ROS_ERROR("Send command to imlidar error!");
			return;
		}
	}

	void IMLidar::poll(sensor_msgs::LaserScan::Ptr scan) {
		bool got_scan = false;
		uint8_t temp_data;
		while (!shutting_down_ && !got_scan)
		{
			boost::asio::read(serial_, boost::asio::buffer(&temp_data, 1));
			ptr_data_in_buffer_[data_in_buffer_len_cnt_++] = temp_data;
			/* If the buffer is full, shows the device communication failure */
			if (data_in_buffer_len_cnt_ >= PARSE_LEN)
			{
				ROS_ERROR("Communication failure, please check the device!");
				data_in_buffer_len_cnt = 0;
				continue;
			}
			package_in_.DataInBuff = ptr_data_in_buffer_;
			package_in_.DataInLen = data_in_buffer_len_cnt_;

			if (Unpacking(&package_in_) == PACK_OK and package_in_.DataID == PACK_LIDAR_DATA)
			{
				ptr_lidar_data_ = (LidarDataStructDef *)package_in_.DataOutBuff;

				/* Prepare the scan to publish */
				scan->angle_min = ANGLE_MIN;
				scan->angle_max = ANGLE_MAX;
				scan->angle_increment = ANGLE_INCREMENT;
				scan->time_increment = 1.f / ptr_lidar_data_->CurrSpeed / SCAN_RESOLUTION;
				scan->scan_time = 1.f / ptr_lidar_data_->CurrSpeed;
				scan->range_min = RANGE_MIN;
				scan->range_max = RANGE_MAX;
				scan->ranges.clear();
				scan->intensities.clear();

				/* Check data_sequence_direction for data sequence */
				if (data_sequence_direction_ == "ccw")
				{
					for (uint16_t i = 0; i < SCAN_RESOLUTION; i++)
					{
						scan->ranges.push_back((ptr_lidar_data_->Data)[i].Distance / 1000.f);
						scan->intensities.push_back((ptr_lidar_data_->Data)[i].Confidence);
					}	
				}
				else if (data_sequence_direction_ == "cw")
				{
					for (uint16_t i = 0; i < SCAN_RESOLUTION; i++)
					{
						scan->ranges.push_back((ptr_lidar_data_->Data)[SCAN_RESOLUTION - i].Distance / 1000.f);
						scan->intensities.push_back((ptr_lidar_data_->Data)[SCAN_RESOLUTION - i].Confidence);
					}
				}

				data_in_buffer_len_cnt_ = 0;
				package_in_.DataID = PACK_NULL;
				got_scan = true;
			}
		}
		return;
	}

	void IMLidar::close() {

		set_lidar_speed(0);
		shutting_down_ = true;
	}
};

/************************* END OF FILE *******************************/