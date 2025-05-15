#include <libusb-1.0/libusb.h>
#include <iostream>
#include <iomanip>
#include <cstddef>
#include <chrono>
#include <rclcpp/node.hpp>
#include <vector>
#include <queue>
#include <ctime>
#include <memory>
#include <functional>
#include "crc16_modbus.h"
#include "doraemon_pack.h"
#include "rclcpp/rclcpp.hpp"
#include "ros_robot_controller_msgs/msg/motors_state.hpp"
#include "ros_robot_controller_msgs/msg/motor_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

// #define __DEBUG__

#define USBLINK_VID 0x7676
#define USBLINK_PID 0x2302

double gx, gy, gz, ax, ay, az;
int16_t l_speed, r_speed;

enum {
	USBLINK_PACK_CMD_IDEL = 0,
	USBLINK_PACK_CMD_UPLOAD_REP,
	USBLINK_PACK_CMD_SET_MOTO,
	USBLINK_PACK_CMD_SET_MOTO_ACK,
};

typedef struct {
	uint8_t cmd;
	std::vector<uint8_t> dat;
}rx_packet_t;

std::queue<rx_packet_t> rx_packet_queue;

// function declaration
void package_motor_data(uint8_t cmd, uint16_t len, uint8_t *dat, std::vector<uint8_t> &dst);
// function declaration end

int64_t get_time_stamp(void)
{
	auto now = std::chrono::system_clock::now();

	return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

std::time_t get_time_stamp_c(void)
{
	auto now = std::chrono::system_clock::now();

	return std::chrono::system_clock::to_time_t(now);
}


class usblink_publisher : public rclcpp::Node
{
	public:
		usblink_publisher() :
			Node("usblink_publisher") {
			motors_rsp_publisher = this->create_publisher<ros_robot_controller_msgs::msg::MotorsState>("motors_rsp", 1);
			imu_raw_rsp = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 1);
			motors_speed_subscription = this->create_subscription<ros_robot_controller_msgs::msg::MotorsState>(
					"motors_set",
					1,
					std::bind(&usblink_publisher::motors_set_callback, this, std::placeholders::_1));
		}

		void pub_imu(double ax, double ay, double az, double gx, double gy, double gz) {
			auto dat = sensor_msgs::msg::Imu();
			
			dat.header.frame_id = "imu_link";//表示该数据是基于哪个坐标系测量
			dat.header.stamp = this->now();

			dat.orientation.w = 0.0;
			dat.orientation.x = 0.0;
			dat.orientation.y = 0.0;
			dat.orientation.z = 0.0;
			dat.orientation_covariance = {
				0.01, 0.00, 0.00,
				0.00, 0.01, 0.00,
				0.00, 0.00, 0.01,};

			dat.linear_acceleration.set__x((double)ax);
			dat.linear_acceleration.set__y((double)ay);
			dat.linear_acceleration.set__z((double)az);
			dat.linear_acceleration_covariance = {
				0.004, 0.00, 0.00,
				0.00, 0.004, 0.00,
				0.00, 0.00, 0.004,};

			dat.angular_velocity.set__x((double)gx);
			dat.angular_velocity.set__y((double)gy);
			dat.angular_velocity.set__z((double)gz);
			dat.angular_velocity_covariance = {
				0.01, 0.00, 0.00,
				0.00, 0.01, 0.00,
				0.00, 0.00, 0.01,};

			imu_raw_rsp->publish(dat);
		}

		void pub_moto_state(int16_t l_speed, int16_t r_speed) {
			auto dat = ros_robot_controller_msgs::msg::MotorsState();
			auto motor_state = ros_robot_controller_msgs::msg::MotorState();

			motor_state.id = 2;
			motor_state.rps = ((double)l_speed) / 60.0l;
			dat.data.push_back(motor_state);

			motor_state.id = 4;
			motor_state.rps = ((double)r_speed) / 60.0l;
			dat.data.push_back(motor_state);

			motors_rsp_publisher->publish(dat);
		}

		void set_usb_handle(libusb_device_handle *p) {
			this->_handle = p;
		}
	private:
		void motors_set_callback(const ros_robot_controller_msgs::msg::MotorsState &msg) const {
			std::vector<uint8_t> tmp;
			uint8_t dat[4];
			double speed;
			bool moto_chk2 = false, moto_chk4 = false;

			if (msg.data.size() < 2) {
				RCLCPP_INFO(this->get_logger(), "|*ERROR*| /motors_set subscription get data not enough.");
				return;
			}
			for (auto i = msg.data.begin(); i != msg.data.end(); i++) {
				if (i->id == 2) {
					speed = i->rps * 60.0l;
					int16_t speed_tmp = static_cast<int16_t>(round(speed));
					dat[0] = DP_UINT16_H(speed_tmp);
					dat[1] = DP_UINT16_L(speed_tmp);
					moto_chk2 = true;
				}
				if (i->id == 4) {
					speed = i->rps * 60.0l;
					int16_t speed_tmp = static_cast<int16_t>(round(speed));
					dat[2] = DP_UINT16_H(speed_tmp);
					dat[3] = DP_UINT16_L(speed_tmp);
					moto_chk4 = true;
				}
			}

			if (!(moto_chk2 && moto_chk4)) {
				RCLCPP_INFO(this->get_logger(), "|*ERROR*|  /motors_set subscription miss moto.");
				return;
			}
			package_motor_data(USBLINK_PACK_CMD_SET_MOTO,
					4,
					dat,
					tmp);

			if (this->_handle == NULL) {
				RCLCPP_INFO(this->get_logger(), "|*ERROR*|  can't find usb handle in node.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "sub recv: l:%d r:%d", dp_u8_2_u16_msb(&dat[0]), dp_u8_2_u16_msb(&dat[2]));
			for (auto i = tmp.begin(); i != tmp.end(); i++) {
				printf("%02x ", *i.base());
			}
			printf("\n");
			int data_tx_len;
			// RCLCPP_INFO(this->get_logger(), "&");
			libusb_bulk_transfer(_handle,
					0x01,
					tmp.data(),
					tmp.size(),
					&data_tx_len,
					20);

		}

		rclcpp::Publisher<ros_robot_controller_msgs::msg::MotorsState>::SharedPtr motors_rsp_publisher;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_rsp;
		rclcpp::Subscription<ros_robot_controller_msgs::msg::MotorsState>::SharedPtr motors_speed_subscription;
		libusb_device_handle *_handle;
};

void package_motor_data(uint8_t cmd, uint16_t len, uint8_t *dat, std::vector<uint8_t> &dst) {
	if (dat == NULL) {
		dst.clear();
		return;
	}

	dst.emplace_back(0x39);
	dst.emplace_back(0x93);
	dst.emplace_back(cmd);
	dst.emplace_back(DP_UINT16_H(len));
	dst.emplace_back(DP_UINT16_L(len));
	for (int i = 0; i < len; i++) {
		dst.emplace_back(dat[i]);
	}
	uint16_t crc = calc_modbus_crc16(dst.data() + 5, len);	
	dst.emplace_back(DP_UINT16_H(crc));
	dst.emplace_back(DP_UINT16_L(crc));
}	

void execute_rx_data(unsigned char *dat, int len)
{
	static rx_packet_t rx_packet;
	static uint16_t data_len = 0;
	static uint8_t state = 0;
	static uint16_t crc;

	for (int i = 0; i < len; i++) {
		switch (state) {
		case 5:
			rx_packet.dat.push_back(dat[i]);
			if (rx_packet.dat.size() == data_len) {
				state++;
			}
			break;
		case 0:
			if (dat[i] == 0x39) {
				state++;
				rx_packet.cmd = USBLINK_PACK_CMD_IDEL;
				rx_packet.dat.clear();
			}
			break;
		case 1:
			if (dat[i] == 0x93) {
				state++;
			} else {
				state = 0;
			}
			break;
		case 2:
			rx_packet.cmd = dat[i];
			state++;
			break;
		case 3:
			data_len = dat[i];
			data_len <<= 8;
			state++;
			break;
		case 4:
			data_len += dat[i];
			state++;
			break;
		case 6:
			crc = dat[i];
			state++;
			break;
		case 7:
			crc = crc << 8 | dat[i];
			if (calc_modbus_crc16(rx_packet.dat.data(), rx_packet.dat.size()) == crc) {
				rx_packet_queue.push(rx_packet);
			} else {
				i = 0;
			}
			state = 0;
			break;
		default:
			state = 0;
			rx_packet.cmd = USBLINK_PACK_CMD_IDEL;
			rx_packet.dat.clear();
			break;
		}
	}
	// for (int i = 0; i < len; i++) {
	//         printf("0x%02x ", dat[i]);
	// }
	// std::cout << std::endl;
}

void __package_upload_data(uint8_t *dat)
{
	uint32_t tmp;

	tmp = dp_u8_2_u32_lsb(&dat[0]);
	gx = (double)*((float *)(&tmp));
	tmp = dp_u8_2_u32_lsb(&dat[4]);
	gy = (double)*((float *)(&tmp));
	tmp = dp_u8_2_u32_lsb(&dat[8]);
	gz = (double)*((float *)(&tmp));
	tmp = dp_u8_2_u32_lsb(&dat[12]);
	ax = (double)*((float *)(&tmp));
	tmp = dp_u8_2_u32_lsb(&dat[16]);
	ay = (double)*((float *)(&tmp));
	tmp = dp_u8_2_u32_lsb(&dat[20]);
	az = (double)*((float *)(&tmp));
	l_speed = (int16_t)dp_u8_2_u16_lsb(&dat[24]);
	r_speed = -(int16_t)dp_u8_2_u16_lsb(&dat[26]);
}

void execute_rx_packet(std::shared_ptr<usblink_publisher> up)
{
	static uint32_t error;
	static int64_t t_ms_old;

	while (!rx_packet_queue.empty()) {
		rx_packet_t rx_pack = rx_packet_queue.front();
		switch (rx_pack.cmd) {
		case USBLINK_PACK_CMD_UPLOAD_REP:
			// std::cout << "recv_dat: ";
			// for (uint8_t i : rx_pack.dat) {
			//         std::cout << "0x" << std::setw(2) << std::setfill('0') << std::hex << (int)i << " ";
			// }
			__package_upload_data(rx_pack.dat.data());

			//XXX
			up->pub_imu(ax, ay, az, gx, gy, gz);
			up->pub_moto_state(l_speed, r_speed);
#if defined(__DEBUG__)
			printf("\ngx: %f gy: %f gz: %f\n", gx, gy, gz);
			printf("\nax: %f ay: %f az: %f\n", ax, ay, az);
			std::cout << "l_speed: " << l_speed << " r_speed: " << r_speed << std::endl;
			std::time_t t;
			int64_t t_ms;
			t_ms = get_time_stamp() % 1000;
			t = get_time_stamp_c();
			std::cout << " " << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S") << ":";
			printf("%04ld", t_ms);
			if (t_ms - t_ms_old > 25) {
				error++;
			}
			t_ms_old = t_ms;
			printf(" error: %d\n", error);
			std::cout << std::endl;
#endif
			break;
		case USBLINK_PACK_CMD_IDEL:
		default:
			std::cout << "|*ERROR*| receive packet cmd error." << std::endl;
			break;
		}
		rx_packet_queue.pop();
	}
}

void get_device_info(libusb_device_handle *handle, libusb_device_descriptor *d_desc)
{
	unsigned char buf[1024];

	std::cout << std::endl;
	libusb_get_string_descriptor_ascii(handle, d_desc->iManufacturer, buf, sizeof(buf));
	printf("Manufacturer Descriptor: %s\n", buf);
	libusb_get_string_descriptor_ascii(handle, d_desc->iProduct, buf, sizeof(buf));
	printf("Device Descriptor: %s\n", buf);
	libusb_get_string_descriptor_ascii(handle, d_desc->iSerialNumber, buf, sizeof(buf));
	printf("Serial Number: %s\n\n", buf);
}

void get_endpoint_descriptor(const struct libusb_endpoint_descriptor *ed_desc)
{
	printf("\t\tEndpoint Descriptor:\n");
	printf("\t\t\tbEndpointAddress:    0x%02x (%s)\n", ed_desc->bEndpointAddress,
	   (ed_desc->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) ? "IN" : "OUT");
	printf("\t\t\tbmAttributes:        0x%02x\n", ed_desc->bmAttributes);
	printf("\t\t\twMaxPacketSize:      %d\n", ed_desc->wMaxPacketSize);
	printf("\t\t\tbInterval:           %d\n", ed_desc->bInterval);
	printf("\t\t\tbRefresh:            %d\n", ed_desc->bRefresh);
	printf("\t\t\tbSynchAddress:       %d\n\n", ed_desc->bSynchAddress);
}

void get_interface_descriptor(const struct libusb_interface_descriptor *inf_desc)
{
	printf("\tInterface Descriptor:\n");
	printf("\t\tbInterfaceNumber:    %d\n", inf_desc->bInterfaceNumber);
	printf("\t\tbAlternateSetting:   %d\n", inf_desc->bAlternateSetting);
	printf("\t\tbNumEndpoints:       %d\n\n", inf_desc->bNumEndpoints);	
	
	for (int i = 0; i < inf_desc->bNumEndpoints; i++) {
		get_endpoint_descriptor(&(inf_desc->endpoint[i]));
	}
}

void get_device_config_descriptor(libusb_config_descriptor *config)
{
	const struct libusb_interface *interface;

	printf("Configuration Descriptor:\n");
	printf("\twTotalLength:         %d\n", config->wTotalLength);
	printf("\tbNumInterfaces:       %d\n", config->bNumInterfaces);
	printf("\tbConfigurationValue:  %d\n\n", config->bConfigurationValue);

	for (int i = 0; i < config->bNumInterfaces; i++) {
		interface = &(config->interface[i]);
		for (int j = 0; j < interface->num_altsetting; j++) {
			get_interface_descriptor(&interface->altsetting[j]);
			printf("\n");
		}
	}

}


int main(int argc, char *argv[])
{
	libusb_device **devs;
	libusb_device_handle *handle;
	libusb_device_descriptor d_desc;
	libusb_endpoint_descriptor ed_desc;
	libusb_config_descriptor *config;
	ssize_t cnt;

	int r;

	rclcpp::init(argc, argv);

	r = libusb_init(NULL);

	if (r < 0) {
		return r;
	}
	cnt = libusb_get_device_list(NULL, &devs);
	if (cnt < 0) {
		libusb_exit(NULL);
		return (int)cnt;
	}
	handle = NULL;
	// std::cout << "Device list:\n" << std::endl;
	for (ssize_t i = 0; i < cnt; i++) {
		struct libusb_device_descriptor desc;
		libusb_get_device_descriptor(devs[i], &desc);
		printf("Device %ld: Vendor ID: 0x%04x, Product ID: 0x%04x", i, desc.idVendor, desc.idProduct);
		printf(" Serial Number:%d\n", desc.iSerialNumber);
		if (desc.idVendor == USBLINK_VID && desc.idProduct == USBLINK_PID) {
			int res = libusb_open(devs[i], &handle);
			if (handle == NULL) {
				std::cout << "|*ERROR*| Open device failed. error number:";
				std::cout << res << " " << libusb_error_name(res) << std::endl;
				break;
			} else {
				std::cout << "Open device succeed." << std::endl;
				d_desc = desc;
				get_device_info(handle, &d_desc);

				printf("Config descriptor num: %d\n\n", d_desc.bNumConfigurations);
				for (int idx = 0; idx < d_desc.bNumConfigurations; idx++) {
					r = libusb_get_config_descriptor(devs[i], idx,&config);
					if (r != LIBUSB_SUCCESS) {
						printf("|*ERROR*| Get device config failed. Error:%s\n", libusb_error_name(r));
						exit(1);
					} else {
						get_device_config_descriptor(config);
						std::cout << std::endl;
					}
				}
				break;
			}
		}
	}

	unsigned char rx_buf[256];
	int data_recv_len, data_tx_len;
	int64_t now, last;

	last = get_time_stamp();

	if (handle == NULL) {
		printf("|*ERROR*| usb handle is NULL, didn't find device.\n");
		libusb_free_device_list(devs, 1);
		libusb_exit(NULL);
		return 1;
	}

	auto up = std::make_shared<usblink_publisher>();
	up->set_usb_handle(handle);
	// std::shared_ptr<usblink_publisher> upp;

	while (1) {
		libusb_bulk_transfer(handle,
				0x82,
				rx_buf,
				256,
				&data_recv_len,
				1);
		if (data_recv_len > 0) {
			execute_rx_data(rx_buf, data_recv_len);
			data_recv_len = 0;
		}
		execute_rx_packet(up);
		now = get_time_stamp();
		// if (now - last >= 33) {
		//         int res = libusb_bulk_transfer(handle,
		//                         0x01,
		//                         (unsigned char *)"\x39\x93\x02\x00\x02\x00\x00\xb0\x01",
		//                         sizeof("\x39\x93\x02\x00\x02\x00\x00\x01\xb0") - 1,
		//                         &data_tx_len,
		//                         20);
		//         last = now;
		// }
		rclcpp::spin_some(up);
	}

	libusb_release_interface(handle, 1);
	libusb_free_config_descriptor(config);
	libusb_close(handle);
	std::cout << "Close device succeed." << std::endl;
	libusb_free_device_list(devs, 1);
	libusb_exit(NULL);

	rclcpp::shutdown();

	return 0;
}



