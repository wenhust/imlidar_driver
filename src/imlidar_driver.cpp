
#include <imlidar_driver.h>

namespace imlidar_driver {

	#define LENGTH_OF_HEADER 2   //inmotion Laser header length is 2
	#define LENGTH_OF_TAIL 2   //inmotion Laser tail length is 2

	typedef enum _receive_status_t {
		Waiting_header_1 = 0,
		Waiting_header_2,
		Waiting_tail_1,
		Waiting_tail_2,
		Tail_received
	}receive_status_t;

	IMLidar::IMLidar(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io, uint16_t rps,
		double angle_min, double angle_max, std::string angle_increment_direction) :
		port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_), lidar_rps_(rps),
		angle_min_(angle_min), angle_max_(angle_max), angle_increment_direction_(angle_increment_direction) {

		serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));					//波特率
		serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none)); //流控制
		serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));            //奇偶校验
		serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));        //停止位
		serial_.set_option(boost::asio::serial_port_base::character_size(8));                       //数据位

		//init the pointer
		//Warning this length may be less for further use !!!!!
		ptr_data_in_buffer_ = new uint8_t[PARSE_LEN];
		ptr_packed_data_to_lidar_ = new uint8_t[20];
		ptr_data_to_pack_ = new uint8_t[20];
		package_out_.DataOutLen = new uint32_t;
		package_in_.DataOutLen = new uint32_t;

		memset(ptr_data_in_buffer_, 0, sizeof(ptr_data_in_buffer_));
		memset(ptr_packed_data_to_lidar_, 0, sizeof(ptr_packed_data_to_lidar_));
		memset(ptr_data_to_pack_, 0, sizeof(ptr_data_to_pack_));
		memset(package_out_.DataOutLen, 0, sizeof(package_out_.DataOutLen));
		memset(package_in_.DataOutLen, 0, sizeof(package_in_.DataOutLen));
	}

	void IMLidar::start_lidar() {

		ResultTypeDef result;
		/* Send command to start rotation */
		memset(ptr_data_to_pack_, 0, sizeof(ptr_data_to_pack_));
		ptr_data_to_pack_[0] = 1;	// "start" cmd. This byte should be 1, but the protocol didn't mention it
		package_out_.DataID = PACK_START_ROTATE;
		package_out_.DataInBuff = ptr_data_to_pack_;
		package_out_.DataInLen = 2; //the length of "start" cmd should be 2, the length and cmd value(first byte 1, second 0) were not mentioned in protocol
		package_out_.DataOutBuff = ptr_packed_data_to_lidar_;
		
		result = Package(package_out_);	//pack the data

		if (result != PACK_FAIL) {
			send_lidar_cmd(package_out_);
		}
		else {
			ROS_ERROR("Start imlidar error!");
			return;
		}

		/* we need to delay for a while after config start */
		usleep(1000 * 100);
		set_lidar_speed();
	}

	/* set lidar rotation speed default is 7Hz */
	void IMLidar::set_lidar_speed() {

		ResultTypeDef result;
		memset(ptr_data_to_pack_, 0, sizeof(ptr_data_to_pack_));	//clear the buffer
		/* To  config the lidar rotation speed */
		((uint16_t*)ptr_data_to_pack_)[0] = lidar_rps_ * 360;	// This byte should be the rotation of lidar, default is 7Hz
		package_out_.DataID = PACK_SET_SPEED;
		package_out_.DataInBuff = ptr_data_to_pack_;
		package_out_.DataInLen = 2; //the length of "SET_SPEED" cmd should be 2, the length is not mentioned in protocol
		package_out_.DataOutBuff = ptr_packed_data_to_lidar_;

		result = Package(package_out_);	//pack the data
		if (result != PACK_FAIL) {
			send_lidar_cmd(package_out_);
		}
		else {
			ROS_ERROR("Set speed of imlidar error!");
			return;
		}
	}

	 /* send lidar package */
	void IMLidar::send_lidar_cmd(const PackageDataStruct &package_out) {
		boost::system::error_code ec;
		try {
			/* send data through serial port */
			boost::asio::write(serial_, boost::asio::buffer(package_out.DataOutBuff, *package_out.DataOutLen), ec);
		}
		catch (boost::system::error_code ec) {
			ROS_ERROR("Send command to imlidar error!");
			return;
		}
	}

	void IMLidar::poll(sensor_msgs::LaserScan::Ptr scan) {
		uint8_t temp_char = 0;
		uint16_t tail_pos_verify = 0;
		uint16_t head_pos_verify = 0;
		uint16_t buffer_len_cnt = 0;
		uint16_t fault_cnt = 0;
		bool got_scan = false;
		receive_status_t receive_status = Waiting_header_1;
		ResultTypeDef result;

		while (!shutting_down_ && !got_scan) {
			/* wait for header */
			while (receive_status == Waiting_header_1 or receive_status == Waiting_header_2) {
				boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
				buffer_len_cnt++;
				fault_cnt++;
				if (fault_cnt > PARSE_LEN) {
					ROS_INFO("Can not find header 0XAA 0XAA");
					fault_cnt = 0;
					break;
				}
				if (receive_status == Waiting_header_1) {
					if (temp_char == 0xAA) {
						head_pos_verify = buffer_len_cnt;
						receive_status = Waiting_header_2;
						*(ptr_data_in_buffer_ + Waiting_header_1) = temp_char;
					}
				}
				else if (receive_status == Waiting_header_2) {
					if (temp_char == 0xAA and head_pos_verify == buffer_len_cnt - LENGTH_OF_HEADER + 1) {
						receive_status = Waiting_tail_1;
						*(ptr_data_in_buffer_ + Waiting_header_2) = temp_char;
						buffer_len_cnt = LENGTH_OF_HEADER - 1;
					}
					else if (temp_char == 0xAA) {
						head_pos_verify = buffer_len_cnt;
					}
				}
			}//while(receive_status == Waiting_header_1 or receive_status == Waiting_header_2){

			 /* wait for tail */
			while (receive_status == Waiting_tail_1 or receive_status == Waiting_tail_2) {
				boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
				buffer_len_cnt++;
				*(ptr_data_in_buffer_ + buffer_len_cnt) = temp_char;

				fault_cnt++;
				if (fault_cnt > PARSE_LEN) {
					ROS_INFO("Can not find tail 0X55 0X55");
					fault_cnt = 0;
					break;
				}

				if (receive_status == Waiting_tail_1) {
					if (temp_char == 0x55) {
						tail_pos_verify = buffer_len_cnt;
						receive_status = Waiting_tail_2;
					}
				}
				else if (receive_status == Waiting_tail_2) {
					if (temp_char == 0x55 and tail_pos_verify == buffer_len_cnt - LENGTH_OF_TAIL + 1) {
						receive_status = Tail_received;
					}
					else if (temp_char == 0x55) {
						tail_pos_verify = buffer_len_cnt;
					}
				}
			}// while(receive_status == Waiting_tail_1 or receive_status == Waiting_tail_2){

			 /* A frame of data is received, then pack the frame into PackageDataStruct structure */
			if (receive_status == Tail_received) {
				package_in_.DataInBuff = ptr_data_in_buffer_;
				package_in_.DataInLen = buffer_len_cnt + 1;
				result = Unpacking(&package_in_);

				if (result == PACK_OK and package_in_.DataID == PACK_LIDAR_DATA) {
					/* This frame of data is lidar data, we will put lidar data into scan */
					ptr_lidar_data_ = (LidarDataStructDef *)package_in_.DataOutBuff;
					scan->ranges.reserve(360);
					scan->intensities.reserve(360);

					/* put range and intensities data into scan */
					for (uint16_t i = 0; i<360; i++) {
						scan->ranges.push_back((ptr_lidar_data_->Data)[i].Distance / 1000.f);
						scan->intensities.push_back((ptr_lidar_data_->Data)[i].Confidence);
					}
					scan->time_increment = 1.f / ptr_lidar_data_->CurrSpeed / 360;
					scan->scan_time = 1.f / ptr_lidar_data_->CurrSpeed;
					scan->angle_min = angle_min_;
					scan->angle_max = angle_max_;
					scan->angle_increment = (angle_increment_direction_ == "cw" ? -1 : 1)*(2.0*M_PI / 360.0);
					scan->range_min = 0.15;
					scan->range_max = 10.0;
				}
				receive_status = Waiting_header_1;
				buffer_len_cnt = 0;
				got_scan = true;
				fault_cnt = 0;
			}
		}
		return;
	}
};
