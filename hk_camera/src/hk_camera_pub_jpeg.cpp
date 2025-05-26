#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "HCNetSDK.h"

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HKCameraLifecycleNode : public LifecycleNode {
public:
    HKCameraLifecycleNode() : LifecycleNode("hk_camera_pub") {

        this->declare_parameter<std::string>("ip", "192.168.1.64");
        this->declare_parameter<std::string>("username", "admin");
        this->declare_parameter<std::string>("password", "");
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {

        std::string camera_ip = this->get_parameter("ip").as_string();
        std::string camera_username = this->get_parameter("username").as_string();
        std::string camera_password = this->get_parameter("password").as_string();
        RCLCPP_INFO(this->get_logger(), "Attempting to login to camera at IP: %s with username: %s", camera_ip.c_str(), camera_username.c_str());

        pub_ = this->create_publisher<sensor_msgs::msg::Image>("hk_camera/image_raw", 10);

        NET_DVR_Init();
        NET_DVR_USER_LOGIN_INFO loginInfo = {0};
        NET_DVR_DEVICEINFO_V40 deviceInfo = {0};
        strcpy(loginInfo.sDeviceAddress, camera_ip.c_str());
        strcpy(loginInfo.sUserName, camera_username.c_str());
        strcpy(loginInfo.sPassword, camera_password.c_str());
        loginInfo.wPort = 8000;
        loginInfo.bUseAsynLogin = false;

        lUserID_ = NET_DVR_Login_V40(&loginInfo, &deviceInfo);
        if (lUserID_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Login failed: %d", NET_DVR_GetLastError());
            return CallbackReturn::FAILURE;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        pub_->on_activate();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HKCameraLifecycleNode::capture_and_publish, this)
        );
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        timer_.reset();
        pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        if (lUserID_ >= 0) {
            NET_DVR_Logout(lUserID_);
            lUserID_ = -1;
        }
        NET_DVR_Cleanup();
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        if (lUserID_ >= 0) {
            NET_DVR_Logout(lUserID_);
            lUserID_ = -1;
        }
        NET_DVR_Cleanup();
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    // void capture_and_publish() {
    //     NET_DVR_JPEGPARA jpegPara = {0};
    //     jpegPara.wPicQuality = 0; // 0-最好
    //     jpegPara.wPicSize = 0xff; // 0xff-自动

    //     char filename[128] = "/tmp/hk_tmp.jpg";
    //     if (!NET_DVR_CaptureJPEGPicture(lUserID_, 1, &jpegPara, filename)) {
    //         RCLCPP_ERROR(this->get_logger(), "CaptureJPEGPicture failed: %d", NET_DVR_GetLastError());
    //         return;
    //     }

    //     cv::Mat img = cv::imread(filename, cv::IMREAD_COLOR);
    //     if (img.empty()) {
    //         RCLCPP_ERROR(this->get_logger(), "cv::imread failed");
    //         return;
    //     }

    //     auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
    //     msg->header.stamp = this->now();
    //     pub_->publish(*msg);
    // }

    void capture_and_publish() {
        NET_DVR_JPEGPARA jpegPara = {0};
        jpegPara.wPicQuality = 0; // 0-最好
        jpegPara.wPicSize = 0xff; // 0xff-自动

        // 使用NET_DVR_CaptureJPEGPicture_NEW直接写入内存
        DWORD dwPicSize = 10*1024 * 1024; // 1MB缓冲区，按需调整
        static std::vector<BYTE> jpegBuf(dwPicSize);
        DWORD retPicSize = 0;

        if (!NET_DVR_CaptureJPEGPicture_NEW(lUserID_, 2, &jpegPara, reinterpret_cast<char*>(jpegBuf.data()), dwPicSize, &retPicSize)) {
            RCLCPP_ERROR(this->get_logger(), "CaptureJPEGPicture_NEW failed: %d", NET_DVR_GetLastError());
            return;
        }
        if (retPicSize == 0) {
            RCLCPP_ERROR(this->get_logger(), "CaptureJPEGPicture_NEW returned 0 bytes");
            return;
        }

        // 用OpenCV解码内存中的JPEG数据
        cv::Mat jpegData(1, retPicSize, CV_8UC1, jpegBuf.data());
        cv::Mat img = cv::imdecode(jpegData, cv::IMREAD_COLOR);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "cv::imdecode failed");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = this->now();
        pub_->publish(*msg);
    }

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    LONG lUserID_ = -1;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HKCameraLifecycleNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}