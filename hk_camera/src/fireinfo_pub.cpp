#include "fire_interfaces/msg/fire_info.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "HCNetSDK.h"

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FireInfoNode : public LifecycleNode
{
public:
    FireInfoNode() : LifecycleNode("fireinfo_pub")
    {
        this->declare_parameter<int>("fire_info_timer_period_ms", 100);
        this->declare_parameter<std::string>("ip", "192.168.1.64"); // Default IP
        this->declare_parameter<std::string>("username", "admin");  // Default username
        this->declare_parameter<std::string>("password", "");       // Default password
        this->declare_parameter<bool>("enbale_pub_picture", true);  // Declared, but not used yet

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
                                std::bind(&FireInfoNode::pub_fire_info, this));
                        }
                    }
                }
                return result;
            });
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        pub_ = this->create_publisher<fire_interfaces::msg::FireInfo>("hk_camera/fire_info", 5);

        // Retrieve parameters
        std::string camera_ip = this->get_parameter("ip").as_string();
        std::string camera_username = this->get_parameter("username").as_string();
        std::string camera_password = this->get_parameter("password").as_string();
        bool enable_pub_picture = this->get_parameter("enbale_pub_picture").as_bool(); // Retrieved but not used yet

        RCLCPP_INFO(this->get_logger(), "Attempting to login to camera at IP: %s with username: %s", camera_ip.c_str(), camera_username.c_str());

        NET_DVR_Init();
        NET_DVR_USER_LOGIN_INFO loginInfo = {0};
        NET_DVR_DEVICEINFO_V40 deviceInfo = {0};

        // Use strncpy for safety
        strncpy(loginInfo.sDeviceAddress, camera_ip.c_str(), sizeof(loginInfo.sDeviceAddress) - 1);
        loginInfo.sDeviceAddress[sizeof(loginInfo.sDeviceAddress) - 1] = '\0'; // Ensure null termination

        strncpy(loginInfo.sUserName, camera_username.c_str(), sizeof(loginInfo.sUserName) - 1);
        loginInfo.sUserName[sizeof(loginInfo.sUserName) - 1] = '\0'; // Ensure null termination

        strncpy(loginInfo.sPassword, camera_password.c_str(), sizeof(loginInfo.sPassword) - 1);
        loginInfo.sPassword[sizeof(loginInfo.sPassword) - 1] = '\0'; // Ensure null termination

        loginInfo.wPort = 8000; // Assuming default port, can be parameterized if needed
        loginInfo.bUseAsynLogin = false;

        lUserID_ = NET_DVR_Login_V40(&loginInfo, &deviceInfo);
        if (lUserID_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Login failed: %d. IP: %s, User: %s", NET_DVR_GetLastError(), camera_ip.c_str(), camera_username.c_str());
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(this->get_logger(), "Login SUCCESS to IP: %s", camera_ip.c_str());

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        pub_->on_activate();
        int period = this->get_parameter("fire_info_timer_period_ms").as_int();

        pub_fire_info_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period),
            std::bind(&FireInfoNode::pub_fire_info, this));
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        pub_fire_info_timer_.reset();
        pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        if (lUserID_ >= 0)
        {
            NET_DVR_Logout(lUserID_);
            lUserID_ = -1;
        }
        NET_DVR_Cleanup();
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        if (lUserID_ >= 0)
        {
            NET_DVR_Logout(lUserID_);
            lUserID_ = -1;
        }
        NET_DVR_Cleanup();
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    void pub_fire_info()
    {
        // NET_DVR_STDXMLConfig_REQUEST("GET /ISAPI/Thermal/channels/2/fireDetection", nullptr);
        auto msg = fire_interfaces::msg::FireInfo();
        // 这个接口大概要200ms
        if (get_fire_info(msg))
        {
            RCLCPP_ERROR(this->get_logger(), "Get temperature failed");
            return;
        }

        msg.header.stamp = this->now();
        pub_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Publishing fire info");
    }

    std::string NET_DVR_STDXMLConfig_REQUEST(const char *Url, const char *InBuffer)
    {
        NET_DVR_XML_CONFIG_INPUT struInputParam = {0};
        NET_DVR_XML_CONFIG_OUTPUT struOutputParam = {0};
        struInputParam.dwSize = sizeof(NET_DVR_XML_CONFIG_INPUT);
        struInputParam.lpRequestUrl = (void *)Url;
        if (struInputParam.lpRequestUrl)
            struInputParam.dwRequestUrlLen = strlen((char *)struInputParam.lpRequestUrl);
        struInputParam.lpInBuffer = (void *)InBuffer;
        if (struInputParam.lpInBuffer)
            struInputParam.dwInBufferSize = strlen((char *)struInputParam.lpInBuffer);

        char pOutBuffer[16 * 1024] = {0};
        char pStatusBuffer[1024] = {0};

        struOutputParam.dwSize = sizeof(NET_DVR_XML_CONFIG_OUTPUT);
        struOutputParam.lpOutBuffer = pOutBuffer;
        struOutputParam.dwOutBufferSize = sizeof(pOutBuffer);
        struOutputParam.lpStatusBuffer = pStatusBuffer;
        struOutputParam.dwStatusSize = sizeof(pStatusBuffer);
        if (NET_DVR_STDXMLConfig(lUserID_, &struInputParam, &struOutputParam))
        {
            RCLCPP_INFO(this->get_logger(), "NET_DVR_STDXMLConfig success\n");
            RCLCPP_INFO(this->get_logger(), "Output buffer: %s\n", (char *)struOutputParam.lpOutBuffer);
            RCLCPP_INFO(this->get_logger(), "Status buffer: %s\n", (char *)struOutputParam.lpStatusBuffer);
            return std::string(pOutBuffer, strnlen(pOutBuffer, sizeof(pOutBuffer)));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "NET_DVR_STDXMLConfig failed, %d\n", NET_DVR_GetLastError());
            return {};
        }
    }
    int get_fire_info(fire_interfaces::msg::FireInfo &msg)
    {
        int iRet;
        DWORD uiReturnLen;
        NET_DVR_THERMOMETRYRULE_TEMPERATURE_INFO struOutBuffer = {0};
        iRet = NET_DVR_GetDVRConfig(lUserID_, NET_DVR_GET_THERMOMETRYRULE_TEMPERATURE_INFO, 1,
                                    &struOutBuffer, sizeof(struOutBuffer), &uiReturnLen);
        if (!iRet)
        {
            RCLCPP_ERROR(this->get_logger(), "NET_DVR_GetDVRConfig failed, %d\n", NET_DVR_GetLastError());
            return -1;
        }
        else
        {
            // RCLCPP_DEBUG(this->get_logger(), "NET_DVR_GetDVRConfig success\n");
            RCLCPP_DEBUG(this->get_logger(), "Temperature freezed: %d\n", struOutBuffer.byIsFreezedata);
            if (struOutBuffer.byIsFreezedata)
                return -1;
            msg.max = struOutBuffer.fMaxTemperature;
            msg.center_x = struOutBuffer.struHighestPoint.fX;
            msg.center_y = struOutBuffer.struHighestPoint.fY;
            return 0;
        }
    }

    rclcpp_lifecycle::LifecyclePublisher<fire_interfaces::msg::FireInfo>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr pub_fire_info_timer_;
    int lUserID_ = -1; // Initialize lUserID_
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FireInfoNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}