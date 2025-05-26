#include "fire_interfaces/msg/fire_info.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "HCNetSDK.h"
#include "LinuxPlayM4.h" // Include for PlayM4 functions

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FireInfoNode : public LifecycleNode
{
public:
    FireInfoNode() : LifecycleNode("fireinfo_pub")
    {
        this->declare_parameter<int>("fire_info_timer_period_ms", 100);
        this->declare_parameter<std::string>("ip", "192.168.1.64");
        this->declare_parameter<std::string>("username", "admin");
        this->declare_parameter<std::string>("password", "");
        this->declare_parameter<bool>("enable_pub_picture", true); // Corrected typo

        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params)
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto &param : params)
                {
                    if (param.get_name() == "fire_info_timer_period_ms")
                    {
                        int new_period = param.as_int();
                        if (pub_fire_info_timer_)
                        {
                            pub_fire_info_timer_->cancel();
                            pub_fire_info_timer_ = this->create_wall_timer(
                                std::chrono::milliseconds(new_period),
                                std::bind(&FireInfoNode::pub_fire_info_callback, this));
                        }
                    }
                    else if (param.get_name() == "enable_pub_picture")
                    {
                        bool new_enable_status = param.as_bool();
                        // If the node is active, try to reconfigure the picture stream
                        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                        {
                            if (new_enable_status && lRealPlayHandle_ < 0) {
                                RCLCPP_INFO(this->get_logger(), "Parameter changed: Enabling picture publishing.");
                                start_image_stream();
                            } else if (!new_enable_status && lRealPlayHandle_ >= 0) {
                                RCLCPP_INFO(this->get_logger(), "Parameter changed: Disabling picture publishing.");
                                stop_image_stream();
                            }
                        }
                        enable_pub_picture_ = new_enable_status; // Update member variable
                    }
                }
                return result;
            });
    }

    // Static SDK Callbacks
    static void CALLBACK DecCBFun_s(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo, void *pUser, int /*nReserved2*/)
    {
        if (!pUser) return;
        FireInfoNode *node = static_cast<FireInfoNode *>(pUser);

        if (pFrameInfo->nType == T_YV12)
        {
            if (!node->image_pub_ || !node->image_pub_->is_activated()) return;

            int width = pFrameInfo->nWidth;
            int height = pFrameInfo->nHeight;
            cv::Mat yv12(height + height / 2, width, CV_8UC1, (unsigned char *)pBuf);
            cv::Mat bgr;
            cv::cvtColor(yv12, bgr, cv::COLOR_YUV2BGR_YV12);

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();
            if (node->image_clock_) {
                msg->header.stamp = node->image_clock_->now();
            } else {
                 msg->header.stamp = node->now(); // Fallback
            }
            node->image_pub_->publish(*msg);
        }
    }

    static void CALLBACK g_RealDataCallBack_V30_s(LONG /*lRealHandle*/, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *pUser)
    {
        if (!pUser) return;
        FireInfoNode *node = static_cast<FireInfoNode *>(pUser);

        switch (dwDataType)
        {
        case NET_DVR_SYSHEAD:
            if (node->playm4_port_ >= 0) break; // Already initialized
            if (!PlayM4_GetPort(&node->playm4_port_))
            {
                RCLCPP_ERROR(node->get_logger(), "PlayM4_GetPort failed");
                break;
            }
            if (dwBufSize > 0)
            {
                if (!PlayM4_SetStreamOpenMode(node->playm4_port_, STREAME_REALTIME))
                {
                    RCLCPP_ERROR(node->get_logger(), "PlayM4_SetStreamOpenMode failed");
                    break;
                }
                if (!PlayM4_OpenStream(node->playm4_port_, pBuffer, dwBufSize, 1024 * 1024))
                {
                    RCLCPP_ERROR(node->get_logger(), "PlayM4_OpenStream failed");
                    break;
                }
                // Pass 'node' (this pointer) as user data to DecCBFun_s
                PlayM4_SetDecCallBackExMend(node->playm4_port_, DecCBFun_s, reinterpret_cast<void*>(node), 0, NULL);
                if (!PlayM4_Play(node->playm4_port_, 0)) // Play on window handle 0 (no display)
                {
                    RCLCPP_ERROR(node->get_logger(), "PlayM4_Play failed");
                    break;
                }
                RCLCPP_INFO(node->get_logger(), "PlayM4 stream started successfully for port %d.", node->playm4_port_);
            }
            break;
        case NET_DVR_STREAMDATA:
        default:
            if (dwBufSize > 0 && node->playm4_port_ != -1)
            {
                if (!PlayM4_InputData(node->playm4_port_, pBuffer, dwBufSize))
                {
                    // RCLCPP_WARN(node->get_logger(), "PlayM4_InputData failed for port %d", node->playm4_port_);
                }
            }
            break;
        }
    }


    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        pub_ = this->create_publisher<fire_interfaces::msg::FireInfo>("hk_camera/fire_info", 5);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("hk_camera/image_raw", 10);
        image_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        camera_ip_ = this->get_parameter("ip").as_string();
        camera_username_ = this->get_parameter("username").as_string();
        camera_password_ = this->get_parameter("password").as_string();
        enable_pub_picture_ = this->get_parameter("enable_pub_picture").as_bool();

        RCLCPP_INFO(this->get_logger(), "Configuring FireInfoNode. IP: %s, User: %s, EnablePicture: %s",
                    camera_ip_.c_str(), camera_username_.c_str(), enable_pub_picture_ ? "true" : "false");

        if (NET_DVR_Init() == FALSE) {
            RCLCPP_ERROR(this->get_logger(), "NET_DVR_Init failed!");
            return CallbackReturn::FAILURE;
        }
        // NET_DVR_SetLogToFile(3, (char *)"./sdkLog", TRUE); // Optional: Enable SDK logging

        NET_DVR_USER_LOGIN_INFO loginInfo = {0};
        NET_DVR_DEVICEINFO_V40 deviceInfo = {0};

        strncpy(loginInfo.sDeviceAddress, camera_ip_.c_str(), sizeof(loginInfo.sDeviceAddress) - 1);
        loginInfo.sDeviceAddress[sizeof(loginInfo.sDeviceAddress) - 1] = '\0';
        strncpy(loginInfo.sUserName, camera_username_.c_str(), sizeof(loginInfo.sUserName) - 1);
        loginInfo.sUserName[sizeof(loginInfo.sUserName) - 1] = '\0';
        strncpy(loginInfo.sPassword, camera_password_.c_str(), sizeof(loginInfo.sPassword) - 1);
        loginInfo.sPassword[sizeof(loginInfo.sPassword) - 1] = '\0';
        loginInfo.wPort = 8000;
        loginInfo.bUseAsynLogin = false;

        lUserID_ = NET_DVR_Login_V40(&loginInfo, &deviceInfo);
        if (lUserID_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Login failed: %d. IP: %s, User: %s", NET_DVR_GetLastError(), camera_ip_.c_str(), camera_username_.c_str());
            NET_DVR_Cleanup();
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(this->get_logger(), "Login SUCCESS to IP: %s. UserID: %ld", camera_ip_.c_str(), lUserID_);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "Activating FireInfoNode.");
        pub_->on_activate();
        image_pub_->on_activate();

        int period = this->get_parameter("fire_info_timer_period_ms").as_int();
        pub_fire_info_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period),
            std::bind(&FireInfoNode::pub_fire_info_callback, this));

        if (enable_pub_picture_)
        {
            start_image_stream();
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating FireInfoNode.");
        pub_fire_info_timer_.reset();
        
        if (enable_pub_picture_ || lRealPlayHandle_ >=0) // Stop if it was enabled or handle is valid
        {
            stop_image_stream();
        }

        pub_->on_deactivate();
        image_pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up FireInfoNode.");
        // Ensure stream is stopped before logout
        if (lRealPlayHandle_ >= 0) {
            stop_image_stream(); // Should already be called in deactivate, but as a safeguard
        }
        if (lUserID_ >= 0)
        {
            NET_DVR_Logout(lUserID_);
            lUserID_ = -1;
        }
        NET_DVR_Cleanup();
        pub_.reset();
        image_pub_.reset();
        image_clock_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down FireInfoNode.");
        // Reuse on_cleanup for resource release
        return on_cleanup(state);
    }

private:
    void pub_fire_info_callback()
    {
        auto msg = fire_interfaces::msg::FireInfo();
        if (get_fire_info_sdk(msg)) // Renamed to avoid conflict
        {
            // Error already logged in get_fire_info_sdk
            return;
        }
        msg.header.stamp = this->now();
        if (pub_ && pub_->is_activated()) {
             pub_->publish(msg);
        }
    }

    int get_fire_info_sdk(fire_interfaces::msg::FireInfo &msg)
    {
        if (lUserID_ < 0) {
            RCLCPP_WARN(this->get_logger(), "Not logged in, cannot get fire info.");
            return -1;
        }
        int iRet;
        DWORD uiReturnLen;
        NET_DVR_THERMOMETRYRULE_TEMPERATURE_INFO struOutBuffer = {0};
        iRet = NET_DVR_GetDVRConfig(lUserID_, NET_DVR_GET_THERMOMETRYRULE_TEMPERATURE_INFO, 1, // Assuming channel 1 for thermal
                                    &struOutBuffer, sizeof(struOutBuffer), &uiReturnLen);
        if (!iRet)
        {
            RCLCPP_ERROR(this->get_logger(), "NET_DVR_GetDVRConfig for fire info failed, error: %d", NET_DVR_GetLastError());
            return -1;
        }
        if (struOutBuffer.byIsFreezedata) {
            RCLCPP_WARN(this->get_logger(), "Thermal data is frozen, skipping.");
            return -1; // Or handle as needed
        }
        msg.max = struOutBuffer.fMaxTemperature;
        msg.center_x = struOutBuffer.struHighestPoint.fX;
        msg.center_y = struOutBuffer.struHighestPoint.fY;
        return 0;
    }

    bool start_image_stream() {
        if (lUserID_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot start image stream: Not logged in.");
            return false;
        }
        if (lRealPlayHandle_ >= 0) {
            RCLCPP_WARN(this->get_logger(), "Image stream already started.");
            return true; // Already running
        }

        RCLCPP_INFO(this->get_logger(), "Starting image stream...");
        NET_DVR_PREVIEWINFO struPlayInfo = {0};
#if (defined(_WIN32) || defined(_WIN_WCE))
        struPlayInfo.hPlayWnd = NULL;
#elif defined(__linux__)
        struPlayInfo.hPlayWnd = 0; // No window needed for callback
#endif
        struPlayInfo.lChannel = 1; // Assuming channel 1 for visual stream, adjust if needed
        struPlayInfo.dwLinkMode = 0; // TCP
        struPlayInfo.bBlocked = 1;   // Blocking mode for stream start
        struPlayInfo.dwDisplayBufNum = 15; // Buffer number

        // Pass 'this' pointer as user data to g_RealDataCallBack_V30_s
        lRealPlayHandle_ = NET_DVR_RealPlay_V40(lUserID_, &struPlayInfo, g_RealDataCallBack_V30_s, reinterpret_cast<void*>(this));
        if (lRealPlayHandle_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "NET_DVR_RealPlay_V40 error: %d", NET_DVR_GetLastError());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Image stream started successfully. RealPlayHandle: %ld", lRealPlayHandle_);
        return true;
    }

    void stop_image_stream() {
        RCLCPP_INFO(this->get_logger(), "Stopping image stream...");
        if (playm4_port_ >= 0) {
            if (!PlayM4_Stop(playm4_port_)) {
                 RCLCPP_WARN(this->get_logger(), "PlayM4_Stop failed for port %d", playm4_port_);
            }
            if (!PlayM4_CloseStream(playm4_port_)) {
                 RCLCPP_WARN(this->get_logger(), "PlayM4_CloseStream failed for port %d", playm4_port_);
            }
            if (!PlayM4_FreePort(playm4_port_)) {
                 RCLCPP_WARN(this->get_logger(), "PlayM4_FreePort failed for port %d", playm4_port_);
            }
            playm4_port_ = -1;
             RCLCPP_INFO(this->get_logger(), "PlayM4 resources released for port.");
        }

        if (lRealPlayHandle_ >= 0)
        {
            if (!NET_DVR_StopRealPlay(lRealPlayHandle_)) {
                RCLCPP_WARN(this->get_logger(), "NET_DVR_StopRealPlay failed, error: %d", NET_DVR_GetLastError());
            }
            lRealPlayHandle_ = -1;
            RCLCPP_INFO(this->get_logger(), "NET_DVR_StopRealPlay called.");
        }
    }

    // Member variables
    rclcpp_lifecycle::LifecyclePublisher<fire_interfaces::msg::FireInfo>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr pub_fire_info_timer_;
    
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Clock::SharedPtr image_clock_;

    LONG lUserID_ = -1;
    LONG lRealPlayHandle_ = -1;
    int playm4_port_ = -1; // Changed from static global to member

    std::string camera_ip_;
    std::string camera_username_;
    std::string camera_password_;
    bool enable_pub_picture_ = true;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FireInfoNode>();
    
    // Use a MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    
    RCLCPP_INFO(node->get_logger(), "FireInfoNode spinning with MultiThreadedExecutor.");
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}