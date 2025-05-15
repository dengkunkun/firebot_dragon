cmake_minimum_required(VERSION 3.10)
project(usblink)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable)

include_directories(/usr/include/libusb-1.0)

link_directories(/usr/lib/aarch64-linux-gnu/)

link_libraries(usb-1.0)

find_package(rclcpp REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(usblink)
target_sources(usblink PRIVATE
	./src/usb.cpp
	./src/crc16_modbus.c
	./src/doraemon_pack.c)
# target_link_libraries(usblink PUBLIC
#         /usr/lib/aarch64-linux-gnu/libusb-1.0.so)
# target_include_directories(usblink PUBLIC
#         "/usr/lib/gcc/x86_64-linux-gnu/12/")
ament_target_dependencies(usblink 
	rclcpp 
	ros_robot_controller_msgs 
	sensor_msgs)
install(TARGETS
	usblink
	DESTINATION
	lib/${PROJECT_NAME})


ament_package()


