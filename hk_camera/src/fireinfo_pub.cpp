#include "fire_interfaces/msg/fire_info.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h> // 添加TF广播器
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "HCNetSDK.h"
#include <cmath>
#include <deque>
#include <numeric>

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FireInfoNode : public LifecycleNode
{
public:
    FireInfoNode() : LifecycleNode("fireinfo_pub")
    {
        // 声明参数
        this->declare_parameter<int>("fire_info_timer_period_ms", 200);
        this->declare_parameter<std::string>("left_ip", "HM-TD2B28T-3-T120250109AACHEA3538378.local");
        this->declare_parameter<std::string>("right_ip", "HM-TD2B28T-3-T120250408AACHEA4127072.local");
        this->declare_parameter<std::string>("username", "admin");
        this->declare_parameter<std::string>("password", "ubuntu_define");
        this->declare_parameter<bool>("enbale_pub_picture", true);
        this->declare_parameter<double>("camera_distance", 0.294); // 相机间距，单位：米
        this->declare_parameter<bool>("enable_color", true);
        this->declare_parameter<bool>("enable_ir", true);

        // 相机参数 - 使用FOV角度而不是像素焦距
        this->declare_parameter<double>("horizontal_fov", 50.0);   // 水平FOV角度
        this->declare_parameter<double>("vertical_fov", 37.2);     // 垂直FOV角度
        this->declare_parameter<double>("principal_point_x", 0.5); // 主点x（比例）
        this->declare_parameter<double>("principal_point_y", 0.5); // 主点y（比例）

        // TF frame名称
        this->declare_parameter<std::string>("left_camera_frame", "left_hk_camera_optical_frame");
        this->declare_parameter<std::string>("right_camera_frame", "right_hk_camera_optical_frame");
        this->declare_parameter<std::string>("world_frame", "base_link");
        this->declare_parameter<std::string>("stereo_ir_camera_frame", "stereo_ir_camera");
        // 滤波参数
        this->declare_parameter<double>("kalman_process_noise", 0.001);
        this->declare_parameter<double>("kalman_measurement_noise", 0.01);
        this->declare_parameter<int>("moving_average_window", 5);
        this->declare_parameter<double>("outlier_threshold", 0.2); // 异常值阈值（米）

        // 添加火焰位置TF相关参数
        this->declare_parameter<std::string>("fire_frame_id", "fire_position");
        this->declare_parameter<bool>("publish_fire_tf", true);

        // 初始化TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
                    else if (param.get_name() == "camera_distance")
                    {
                        camera_distance_ = param.as_double();
                    }
                }
                return result;
            });

        // 初始化卡尔曼滤波器
        // double Q = this->get_parameter("kalman_process_noise").as_double();
        // double R = this->get_parameter("kalman_measurement_noise").as_double();

        // kalman_x_.Q = kalman_y_.Q = kalman_z_.Q = Q;
        // kalman_x_.R = kalman_y_.R = kalman_z_.R = R;
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        pub_ = this->create_publisher<fire_interfaces::msg::FireInfo>("hk_camera/fire_info", 5);

        // 获取参数
        std::string left_camera_ip = this->get_parameter("left_ip").as_string();
        std::string right_camera_ip = this->get_parameter("right_ip").as_string();
        std::string camera_username = this->get_parameter("username").as_string();
        std::string camera_password = this->get_parameter("password").as_string();
        camera_distance_ = this->get_parameter("camera_distance").as_double();

        RCLCPP_INFO(this->get_logger(), "相机间距: %.4f 米", camera_distance_);

        // 初始化SDK
        NET_DVR_Init();

        // 登录左相机
        if (!login_camera(left_camera_ip, camera_username, camera_password, lUserID_Left_, "左相机"))
        {
            return CallbackReturn::FAILURE;
        }

        // 登录右相机
        if (!login_camera(right_camera_ip, camera_username, camera_password, lUserID_Right_, "右相机"))
        {
            // 如果右相机登录失败，清理左相机
            if (lUserID_Left_ >= 0)
            {
                NET_DVR_Logout(lUserID_Left_);
                lUserID_Left_ = -1;
            }
            return CallbackReturn::FAILURE;
        }

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
        cleanup_cameras();
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        cleanup_cameras();
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    bool login_camera(const std::string &ip, const std::string &username,
                      const std::string &password, int &userID, const std::string &name)
    {
        RCLCPP_INFO(this->get_logger(), "正在登录%s，IP: %s，用户名: %s",
                    name.c_str(), ip.c_str(), username.c_str());

        NET_DVR_USER_LOGIN_INFO loginInfo = {0};
        NET_DVR_DEVICEINFO_V40 deviceInfo = {0};

        // 安全地复制字符串
        strncpy(loginInfo.sDeviceAddress, ip.c_str(), sizeof(loginInfo.sDeviceAddress) - 1);
        loginInfo.sDeviceAddress[sizeof(loginInfo.sDeviceAddress) - 1] = '\0';

        strncpy(loginInfo.sUserName, username.c_str(), sizeof(loginInfo.sUserName) - 1);
        loginInfo.sUserName[sizeof(loginInfo.sUserName) - 1] = '\0';

        strncpy(loginInfo.sPassword, password.c_str(), sizeof(loginInfo.sPassword) - 1);
        loginInfo.sPassword[sizeof(loginInfo.sPassword) - 1] = '\0';

        loginInfo.wPort = 8000;
        loginInfo.bUseAsynLogin = false;

        userID = NET_DVR_Login_V40(&loginInfo, &deviceInfo);
        if (userID < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "%s登录失败: %d，IP: %s",
                         name.c_str(), NET_DVR_GetLastError(), ip.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "%s登录成功，IP: %s，用户ID: %d",
                    name.c_str(), ip.c_str(), userID);
        return true;
    }

    void cleanup_cameras()
    {
        if (lUserID_Left_ >= 0)
        {
            NET_DVR_Logout(lUserID_Left_);
            RCLCPP_INFO(this->get_logger(), "左相机已注销");
            lUserID_Left_ = -1;
        }
        if (lUserID_Right_ >= 0)
        {
            NET_DVR_Logout(lUserID_Right_);
            RCLCPP_INFO(this->get_logger(), "右相机已注销");
            lUserID_Right_ = -1;
        }
        NET_DVR_Cleanup();
    }

    void pub_fire_info()
    {
        auto msg = fire_interfaces::msg::FireInfo();
        msg.header.stamp = this->now();
        msg.header.frame_id = "stereo_camera";

        bool left_valid = false, right_valid = false;

        // 获取左相机火灾信息
        if (lUserID_Left_ >= 0)
        {
            left_valid = get_fire_info_from_camera(lUserID_Left_, msg.left_max, msg.left_avg,
                                                   msg.left_center_x, msg.left_center_y, "左相机");
        }

        // 获取右相机火灾信息
        if (lUserID_Right_ >= 0)
        {
            right_valid = get_fire_info_from_camera(lUserID_Right_, msg.right_max, msg.right_avg,
                                                    msg.right_center_x, msg.right_center_y, "右相机");
        }

        // 如果两个相机都检测到火焰，进行温差检测
        if (left_valid && right_valid)
        {
            // 检测温差
            float temp_diff = std::abs(msg.left_max - msg.right_max);

            RCLCPP_INFO(this->get_logger(),
                        "双相机检测 - 左相机: (%.3f,%.3f) %.1f°C, 右相机: (%.3f,%.3f) %.1f°C, 温差: %.1f°C",
                        msg.left_center_x, msg.left_center_y, msg.left_max,
                        msg.right_center_x, msg.right_center_y, msg.right_max,
                        temp_diff);

            if (temp_diff <= 5.0)
            {
                // 温差在合理范围内，计算3D坐标
                calculate_3d_fire_position(msg.left_center_x, msg.left_center_y,
                                           msg.right_center_x, msg.right_center_y,
                                           msg.fire_position);

                // 发布消息
                pub_->publish(msg);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "左右相机温差过大 (%.1f°C > 5.0°C)，跳过3D计算", temp_diff);
                msg.fire_position.x = 0.0;
                msg.fire_position.y = 0.0;
                msg.fire_position.z = 0.0;
                pub_->publish(msg);
            }
        }
        else if (left_valid)
        {
            RCLCPP_INFO(this->get_logger(), "仅左相机检测到火焰: (%.3f,%.3f) %.1f°C",
                        msg.left_center_x, msg.left_center_y, msg.left_max);
            // 清零右相机和3D位置数据
            msg.right_max = 0.0;
            msg.right_avg = 0.0;
            msg.right_center_x = 0.0;
            msg.right_center_y = 0.0;
            msg.fire_position.x = 0.0;
            msg.fire_position.y = 0.0;
            msg.fire_position.z = 0.0;
            pub_->publish(msg);
        }
        else if (right_valid)
        {
            RCLCPP_INFO(this->get_logger(), "仅右相机检测到火焰: (%.3f,%.3f) %.1f°C",
                        msg.right_center_x, msg.right_center_y, msg.right_max);
            // 清零左相机和3D位置数据
            msg.left_max = 0.0;
            msg.left_avg = 0.0;
            msg.left_center_x = 0.0;
            msg.left_center_y = 0.0;
            msg.fire_position.x = 0.0;
            msg.fire_position.y = 0.0;
            msg.fire_position.z = 0.0;
            pub_->publish(msg);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "未检测到火焰");
            // 清零所有数据
            msg.left_max = 0.0;
            msg.left_avg = 0.0;
            msg.left_center_x = 0.0;
            msg.left_center_y = 0.0;
            msg.right_max = 0.0;
            msg.right_avg = 0.0;
            msg.right_center_x = 0.0;
            msg.right_center_y = 0.0;
            msg.fire_position.x = 0.0;
            msg.fire_position.y = 0.0;
            msg.fire_position.z = 0.0;
            pub_->publish(msg);
        }
    }

    bool get_fire_info_from_camera(int userID, float &max_temp, float &avg_temp,
                                   float &center_x, float &center_y, const std::string &camera_name)
    {
        DWORD uiReturnLen;
        NET_DVR_THERMOMETRYRULE_TEMPERATURE_INFO struOutBuffer = {0};

        int iRet = NET_DVR_GetDVRConfig(userID, NET_DVR_GET_THERMOMETRYRULE_TEMPERATURE_INFO, 1,
                                        &struOutBuffer, sizeof(struOutBuffer), &uiReturnLen);
        if (!iRet)
        {
            RCLCPP_DEBUG(this->get_logger(), "%s获取温度信息失败: %d",
                         camera_name.c_str(), NET_DVR_GetLastError());
            return false;
        }

        // 检查数据是否被冻结
        if (struOutBuffer.byIsFreezedata)
        {
            RCLCPP_DEBUG(this->get_logger(), "%s温度数据被冻结", camera_name.c_str());
            return false;
        }

        // 提取温度和坐标信息
        max_temp = struOutBuffer.fMaxTemperature;
        avg_temp = struOutBuffer.fAverageTemperature; // 如果SDK支持的话
        center_x = struOutBuffer.struHighestPoint.fX;
        center_y = struOutBuffer.struHighestPoint.fY;

        return true;
    }

    void calculate_3d_fire_position(float left_x_ratio, float left_y_ratio,
                                    float right_x_ratio, float right_y_ratio,
                                    geometry_msgs::msg::Point &fire_position)
    {
        try
        {
            // 获取相机参数
            double horizontal_fov_deg = this->get_parameter("horizontal_fov").as_double();
            double vertical_fov_deg = this->get_parameter("vertical_fov").as_double();

            // 输出输入参数用于调试
            RCLCPP_INFO(this->get_logger(),
                        "输入坐标 - 左相机: (%.3f, %.3f), 右相机: (%.3f, %.3f)",
                        left_x_ratio, left_y_ratio, right_x_ratio, right_y_ratio);

            // 直接从比例坐标计算角度
            // 比例坐标0.0对应-FOV/2，0.5对应0度，1.0对应+FOV/2
            double left_azimuth_deg = (left_x_ratio - 0.5) * horizontal_fov_deg;
            double left_elevation_deg = (0.5 - left_y_ratio) * vertical_fov_deg;

            double right_azimuth_deg = (right_x_ratio - 0.5) * horizontal_fov_deg;
            double right_elevation_deg = (0.5 - right_y_ratio) * vertical_fov_deg;

            RCLCPP_INFO(this->get_logger(),
                        "左相机角度: 方位角=%.2f°, 俯仰角=%.2f°",
                        left_azimuth_deg, left_elevation_deg);
            RCLCPP_INFO(this->get_logger(),
                        "右相机角度: 方位角=%.2f°, 俯仰角=%.2f°",
                        right_azimuth_deg, right_elevation_deg);

            // 检查角度是否合理
            if (std::abs(left_azimuth_deg) > horizontal_fov_deg / 2 ||
                std::abs(right_azimuth_deg) > horizontal_fov_deg / 2 ||
                std::abs(left_elevation_deg) > vertical_fov_deg / 2 ||
                std::abs(right_elevation_deg) > vertical_fov_deg / 2)
            {
                RCLCPP_WARN(this->get_logger(), "计算角度超出FOV范围，可能输入数据有误");
            }

            // 简化计算：使用基本三角测量，不依赖TF
            double baseline = camera_distance_;
            double angle_diff_deg = left_azimuth_deg - right_azimuth_deg; // 视差角（度）
            double angle_diff_rad = angle_diff_deg * M_PI / 180.0;

            RCLCPP_INFO(this->get_logger(),
                        "视差角: %.3f度 (%.3f弧度), 基线距离: %.4f米",
                        angle_diff_deg, angle_diff_rad, baseline);

            if (std::abs(angle_diff_rad) < 0.01)
            { // 角度差太小（约0.57度）
                RCLCPP_WARN(this->get_logger(), "视差角太小，无法准确计算深度");
                fire_position.x = 0.0;
                fire_position.y = 0.0;
                fire_position.z = 10.0; // 默认10米
                return;
            }

            // 使用简单三角测量计算距离
            // 对于小角度：distance ≈ baseline / angle_diff_rad
            // 对于大角度：使用更精确的公式
            double distance;
            if (std::abs(angle_diff_rad) < 0.1)
            { // 小角度近似
                distance = baseline / std::abs(angle_diff_rad);
            }
            else
            { // 大角度精确计算
                distance = baseline / (2.0 * tan(std::abs(angle_diff_rad) / 2.0));
            }

            // 使用平均角度计算方向
            double avg_azimuth_deg = (left_azimuth_deg + right_azimuth_deg) / 2.0;
            double avg_elevation_deg = (left_elevation_deg + right_elevation_deg) / 2.0;
            double avg_azimuth_rad = avg_azimuth_deg * M_PI / 180.0;
            double avg_elevation_rad = avg_elevation_deg * M_PI / 180.0;

            // 计算原始3D坐标（修正坐标系）
            geometry_msgs::msg::Point raw_position;
            
            // 修正坐标轴映射：
            // 原来: x=左右, y=上下, z=前后
            // 修正: x=前后, y=左右, z=上下
            raw_position.x = distance * cos(avg_azimuth_rad) * cos(avg_elevation_rad);  // 前方距离
            raw_position.y = distance * sin(avg_azimuth_rad) * cos(avg_elevation_rad);  // 左右偏移
            raw_position.z = distance * sin(avg_elevation_rad);                         // 上下偏移

            RCLCPP_INFO(this->get_logger(),
                        "distance:%lf 修正后3D位置: (%.3f, %.3f, %.3f) 米 [x=前后, y=左右, z=上下]",
                        distance, raw_position.x, raw_position.y, raw_position.z);

            // 异常值检测
            // if (!position_filter_.position_buffer_.empty())
            // {
            //     if (isOutlier(raw_position, position_filter_.filtered_position_))
            //     {
            //         RCLCPP_WARN(this->get_logger(), "检测到异常值，跳过本次测量");
            //         fire_position = position_filter_.filtered_position_;
            //         return;
            //     }
            // }

            // // 应用卡尔曼滤波
            // geometry_msgs::msg::Point kalman_filtered;
            // kalman_filtered.x = kalman_x_.update(raw_position.x);
            // kalman_filtered.y = kalman_y_.update(raw_position.y);
            // kalman_filtered.z = kalman_z_.update(raw_position.z);

            // // 添加到滑动窗口滤波器
            // position_filter_.addPosition(kalman_filtered);

            // // 获取滤波后的位置
            // geometry_msgs::msg::Point filtered_position = position_filter_.filtered_position_;

            // RCLCPP_DEBUG(this->get_logger(),
            //             "滤波后位置: (%.3f, %.3f, %.3f) 米",
            //             filtered_position.x, filtered_position.y, filtered_position.z);

            // 转换到world_frame
            geometry_msgs::msg::Point world_position;
            if (transform_to_world_frame(raw_position, world_position))
            {
                fire_position = world_position;

                RCLCPP_INFO(this->get_logger(),
                            "世界坐标系火源位置: (%.3f, %.3f, %.3f) 米 ",
                            fire_position.x, fire_position.y, fire_position.z);

                // 发布火焰位置的TF变换
                publish_fire_tf(fire_position);
            }
            else
            {
                // 如果TF转换失败，使用相机坐标系的结果
                // fire_position = filtered_position;
                RCLCPP_WARN(this->get_logger(), "TF转换失败，使用相机坐标系结果");

                // 也可以发布相机坐标系的火焰位置
                publish_fire_tf_camera_frame(fire_position);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "计算3D位置时发生错误: %s", e.what());
            fire_position.x = 0.0;
            fire_position.y = 0.0;
            fire_position.z = 1.0;
        }
    }
    bool transform_to_world_frame(const geometry_msgs::msg::Point &camera_position,
                                  geometry_msgs::msg::Point &world_position)
    {
        try
        {
            // 使用相机frame作为参考frame
            std::string camera_frame = this->get_parameter("stereo_ir_camera_frame").as_string();
            std::string world_frame = this->get_parameter("world_frame").as_string();

            RCLCPP_DEBUG(this->get_logger(), "尝试TF转换: %s -> %s",
                         camera_frame.c_str(), world_frame.c_str());

            // 创建在相机坐标系中的点
            geometry_msgs::msg::PointStamped camera_point;
            camera_point.header.frame_id = camera_frame;
            camera_point.header.stamp = this->now();
            camera_point.point = camera_position;

            // 转换到世界坐标系
            geometry_msgs::msg::PointStamped world_point;
            tf_buffer_->transform(camera_point, world_point, world_frame,
                                  tf2::durationFromSec(0.1));

            world_position = world_point.point;

            RCLCPP_INFO(this->get_logger(),
                        "TF转换成功: 相机坐标(%.3f, %.3f, %.3f) -> 世界坐标(%.3f, %.3f, %.3f)",
                        camera_position.x, camera_position.y, camera_position.z,
                        world_position.x, world_position.y, world_position.z);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF转换失败: %s", ex.what());
            return false;
        }
    }
    // 发布火焰位置TF变换（世界坐标系）
    void publish_fire_tf(const geometry_msgs::msg::Point &fire_position)
    {
        if (!this->get_parameter("publish_fire_tf").as_bool())
        {
            return;
        }

        try
        {
            std::string world_frame = this->get_parameter("world_frame").as_string();
            std::string fire_frame = this->get_parameter("fire_frame_id").as_string();

            geometry_msgs::msg::TransformStamped fire_transform;
            fire_transform.header.stamp = this->now();
            fire_transform.header.frame_id = world_frame;
            fire_transform.child_frame_id = fire_frame;

            // 设置位置
            fire_transform.transform.translation.x = fire_position.x;
            fire_transform.transform.translation.y = fire_position.y;
            fire_transform.transform.translation.z = fire_position.z;

            // 设置姿态（火焰没有特定朝向，使用单位四元数）
            fire_transform.transform.rotation.x = 0.0;
            fire_transform.transform.rotation.y = 0.0;
            fire_transform.transform.rotation.z = 0.0;
            fire_transform.transform.rotation.w = 1.0;

            // 发布TF变换
            tf_broadcaster_->sendTransform(fire_transform);

            RCLCPP_DEBUG(this->get_logger(),
                         "发布火焰位置TF: %s -> %s (%.3f, %.3f, %.3f)",
                         world_frame.c_str(), fire_frame.c_str(),
                         fire_position.x, fire_position.y, fire_position.z);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "发布火焰TF失败: %s", e.what());
        }
    }

    // 发布火焰位置TF变换（相机坐标系）
    void publish_fire_tf_camera_frame(const geometry_msgs::msg::Point &fire_position)
    {
        if (!this->get_parameter("publish_fire_tf").as_bool())
        {
            return;
        }

        try
        {
            std::string camera_frame = this->get_parameter("stereo_ir_camera_frame").as_string();
            std::string fire_frame = this->get_parameter("fire_frame_id").as_string();

            geometry_msgs::msg::TransformStamped fire_transform;
            fire_transform.header.stamp = this->now();
            fire_transform.header.frame_id = camera_frame;
            fire_transform.child_frame_id = fire_frame;

            // 设置位置
            fire_transform.transform.translation.x = fire_position.x;
            fire_transform.transform.translation.y = fire_position.y;
            fire_transform.transform.translation.z = fire_position.z;

            // 设置姿态
            fire_transform.transform.rotation.x = 0.0;
            fire_transform.transform.rotation.y = 0.0;
            fire_transform.transform.rotation.z = 0.0;
            fire_transform.transform.rotation.w = 1.0;

            // 发布TF变换
            tf_broadcaster_->sendTransform(fire_transform);

            RCLCPP_DEBUG(this->get_logger(),
                         "发布火焰位置TF(相机坐标系): %s -> %s (%.3f, %.3f, %.3f)",
                         camera_frame.c_str(), fire_frame.c_str(),
                         fire_position.x, fire_position.y, fire_position.z);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "发布火焰TF(相机坐标系)失败: %s", e.what());
        }
    }

    // 成员变量
    rclcpp_lifecycle::LifecyclePublisher<fire_interfaces::msg::FireInfo>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr pub_fire_info_timer_;

    int lUserID_Left_ = -1;  // 左相机用户ID
    int lUserID_Right_ = -1; // 右相机用户ID
    double camera_distance_; // 相机间距

    // TF相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // 添加TF广播器

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
#if 0
    // 保留现有的滤波相关代码（不删除，但卡尔曼滤波功能可以通过参数控制）
    FilteredPosition position_filter_;
    SimpleKalmanFilter kalman_x_, kalman_y_, kalman_z_;

    // 异常值检测
    bool isOutlier(const geometry_msgs::msg::Point &current_pos,
                   const geometry_msgs::msg::Point &last_pos)
    {
        if (position_filter_.position_buffer_.empty())
            return false;

        double distance = sqrt(
            pow(current_pos.x - last_pos.x, 2) +
            pow(current_pos.y - last_pos.y, 2) +
            pow(current_pos.z - last_pos.z, 2));

        // 增大异常值阈值，从0.2米改为1.0米
        // 并添加更多调试信息
        RCLCPP_DEBUG(this->get_logger(),
                     "异常值检测 - 当前位置: (%.3f, %.3f, %.3f), 上次位置: (%.3f, %.3f, %.3f), 距离变化: %.3f米",
                     current_pos.x, current_pos.y, current_pos.z,
                     last_pos.x, last_pos.y, last_pos.z, distance);

        bool is_outlier = distance > 1.0; // 增大阈值到1米

        if (is_outlier)
        {
            RCLCPP_WARN(this->get_logger(), "检测到异常值：位置变化 %.3f米 > 1.0米阈值", distance);
        }

        return is_outlier;
    }
#endif
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FireInfoNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}