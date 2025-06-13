#include "fire_interfaces/msg/fire_info.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "HCNetSDK.h"
#include <cmath>
#include <ctime>
#include <deque>
#include <numeric>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <future>

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// 相机数据结构
struct CameraData
{
    std::atomic<bool> valid{false};
    std::atomic<bool> read_requested{false};
    std::atomic<bool> read_completed{false};
    std::atomic<float> max_temp{0.0f};
    std::atomic<float> avg_temp{0.0f};
    std::atomic<float> center_x{0.0f};
    std::atomic<float> center_y{0.0f};
    std::chrono::steady_clock::time_point timestamp;
    std::mutex mutex;
    std::condition_variable cv_read_request;
    std::condition_variable cv_read_complete;

    void update(float max_t, float avg_t, float cx, float cy)
    {
        std::lock_guard<std::mutex> lock(mutex);
        max_temp.store(max_t);
        avg_temp.store(avg_t);
        center_x.store(cx);
        center_y.store(cy);
        timestamp = std::chrono::steady_clock::now();
        valid.store(true);
        read_completed.store(true);
        cv_read_complete.notify_one();
    }

    void invalidate()
    {
        std::lock_guard<std::mutex> lock(mutex);
        valid.store(false);
        read_completed.store(true);
        cv_read_complete.notify_one();
    }

    bool get_data(float &max_t, float &avg_t, float &cx, float &cy,
                  std::chrono::steady_clock::time_point &ts)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!valid.load())
            return false;

        max_t = max_temp.load();
        avg_t = avg_temp.load();
        cx = center_x.load();
        cy = center_y.load();
        ts = timestamp;
        return true;
    }

    void request_read()
    {
        std::lock_guard<std::mutex> lock(mutex);
        read_requested.store(true);
        read_completed.store(false);
        cv_read_request.notify_one();
    }

    bool wait_for_completion(std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex);
        return cv_read_complete.wait_for(lock, timeout, [this]
                                         { return read_completed.load(); });
    }
};

// 相机线程控制结构
struct CameraThreadControl
{
    std::atomic<bool> should_stop{false};
    std::atomic<bool> thread_ready{false};
    std::thread worker_thread;
    std::mutex mutex;
    std::condition_variable cv_ready;

    void wait_until_ready()
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv_ready.wait(lock, [this]
                      { return thread_ready.load(); });
    }

    void notify_ready()
    {
        std::lock_guard<std::mutex> lock(mutex);
        thread_ready.store(true);
        cv_ready.notify_one();
    }
};

// 火源位置数据结构
struct FirePositionData
{
    geometry_msgs::msg::Point position;
    std::chrono::steady_clock::time_point timestamp;
    double confidence;

    FirePositionData(const geometry_msgs::msg::Point &pos, double conf = 1.0)
        : position(pos), timestamp(std::chrono::steady_clock::now()), confidence(conf) {}
};

// 滑动窗口滤波器
class SlidingWindowFilter
{
public:
    SlidingWindowFilter(size_t window_size = 10, double outlier_threshold = 2.0)
        : window_size_(window_size), outlier_threshold_(outlier_threshold) {}

    geometry_msgs::msg::Point addAndFilter(const geometry_msgs::msg::Point &new_position, double confidence = 1.0)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        data_buffer_.emplace_back(new_position, confidence);

        if (data_buffer_.size() > window_size_)
        {
            data_buffer_.pop_front();
        }

        auto filtered_data = removeOutliers();
        return calculateWeightedAverage(filtered_data);
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_buffer_.clear();
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_buffer_.size();
    }

private:
    mutable std::mutex mutex_;
    std::deque<FirePositionData> data_buffer_;
    size_t window_size_;
    double outlier_threshold_;

    std::vector<FirePositionData> removeOutliers()
    {
        if (data_buffer_.size() < 3)
        {
            return std::vector<FirePositionData>(data_buffer_.begin(), data_buffer_.end());
        }

        auto mean_pos = calculateSimpleAverage();
        double std_dev = calculateStandardDeviation(mean_pos);

        std::vector<FirePositionData> filtered_data;
        for (const auto &data : data_buffer_)
        {
            double distance = calculateDistance(data.position, mean_pos);
            if (distance <= outlier_threshold_ * std_dev)
            {
                filtered_data.push_back(data);
            }
        }

        if (filtered_data.size() < 2)
        {
            filtered_data.clear();
            auto recent_count = std::min(size_t(3), data_buffer_.size());
            for (auto it = data_buffer_.rbegin(); it != data_buffer_.rbegin() + recent_count; ++it)
            {
                filtered_data.insert(filtered_data.begin(), *it);
            }
        }

        return filtered_data;
    }

    geometry_msgs::msg::Point calculateSimpleAverage()
    {
        geometry_msgs::msg::Point avg;
        avg.x = avg.y = avg.z = 0.0;

        for (const auto &data : data_buffer_)
        {
            avg.x += data.position.x;
            avg.y += data.position.y;
            avg.z += data.position.z;
        }

        double count = static_cast<double>(data_buffer_.size());
        avg.x /= count;
        avg.y /= count;
        avg.z /= count;

        return avg;
    }

    geometry_msgs::msg::Point calculateWeightedAverage(const std::vector<FirePositionData> &data)
    {
        if (data.empty())
        {
            return geometry_msgs::msg::Point();
        }

        geometry_msgs::msg::Point weighted_avg;
        weighted_avg.x = weighted_avg.y = weighted_avg.z = 0.0;
        double total_weight = 0.0;

        auto current_time = std::chrono::steady_clock::now();

        for (const auto &item : data)
        {
            auto age = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - item.timestamp).count();
            double time_weight = std::max(0.1, 1.0 - age / 5000.0);
            double weight = item.confidence * time_weight;

            weighted_avg.x += item.position.x * weight;
            weighted_avg.y += item.position.y * weight;
            weighted_avg.z += item.position.z * weight;
            total_weight += weight;
        }

        if (total_weight > 0.0)
        {
            weighted_avg.x /= total_weight;
            weighted_avg.y /= total_weight;
            weighted_avg.z /= total_weight;
        }

        return weighted_avg;
    }

    double calculateStandardDeviation(const geometry_msgs::msg::Point &mean)
    {
        if (data_buffer_.size() < 2)
            return 0.0;

        double sum_sq_diff = 0.0;
        for (const auto &data : data_buffer_)
        {
            double dist = calculateDistance(data.position, mean);
            sum_sq_diff += dist * dist;
        }

        return std::sqrt(sum_sq_diff / (data_buffer_.size() - 1));
    }

    double calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

// 简单卡尔曼滤波器
class SimpleKalmanFilter
{
public:
    SimpleKalmanFilter(double process_noise = 0.01, double measurement_noise = 0.1)
        : q_(process_noise), r_(measurement_noise), x_(0.0), p_(1.0), initialized_(false) {}

    double update(double measurement)
    {
        if (!initialized_)
        {
            x_ = measurement;
            initialized_ = true;
            return x_;
        }

        p_ += q_;
        double k = p_ / (p_ + r_);
        x_ += k * (measurement - x_);
        p_ *= (1.0 - k);

        return x_;
    }

    void reset()
    {
        initialized_ = false;
        x_ = 0.0;
        p_ = 1.0;
    }

private:
    double q_, r_, x_, p_;
    bool initialized_;
};

class FireInfoNode : public LifecycleNode
{
public:
    FireInfoNode() : LifecycleNode("fireinfo_pub"),
                     position_filter_(10),
                     kalman_x_(0.01, 0.1),
                     kalman_y_(0.01, 0.1),
                     kalman_z_(0.01, 0.1)
    {
        // 声明参数
        this->declare_parameter<int>("fire_info_timer_period_ms", 200);
        this->declare_parameter<int>("camera_read_timeout_ms", 500);
        this->declare_parameter<int>("max_sync_time_diff_ms", 50);
        this->declare_parameter<std::string>("left_ip", "HM-TD2B28T-3-T120250109AACHEA3538378.local");
        this->declare_parameter<std::string>("right_ip", "HM-TD2B28T-3-T120250408AACHEA4127072.local");
        this->declare_parameter<std::string>("username", "admin");
        this->declare_parameter<std::string>("password", "ubuntu_define");
        this->declare_parameter<bool>("enbale_pub_picture", true);
        this->declare_parameter<double>("camera_distance", 0.294);
        this->declare_parameter<bool>("enable_color", true);
        this->declare_parameter<bool>("enable_ir", true);

        // 相机参数
        this->declare_parameter<double>("horizontal_fov", 50.0);
        this->declare_parameter<double>("vertical_fov", 37.2);
        this->declare_parameter<double>("fire_tempture_threshold", 50.0);
        this->declare_parameter<double>("principal_point_x", 0.5);
        this->declare_parameter<double>("principal_point_y", 0.5);

        // TF frame名称
        this->declare_parameter<std::string>("left_camera_frame", "left_hk_camera_optical_frame");
        this->declare_parameter<std::string>("right_camera_frame", "right_hk_camera_optical_frame");
        this->declare_parameter<std::string>("world_frame", "base_link");
        this->declare_parameter<std::string>("stereo_ir_camera_frame", "stereo_ir_camera");
        this->declare_parameter<std::string>("fire_frame_id", "fire_position");
        this->declare_parameter<bool>("publish_fire_tf", true);

        // 滤波器参数
        this->declare_parameter<bool>("enable_position_filter", true);
        this->declare_parameter<int>("filter_window_size", 10);
        this->declare_parameter<double>("filter_outlier_threshold", 2.0);
        this->declare_parameter<double>("kalman_process_noise", 0.01);
        this->declare_parameter<double>("kalman_measurement_noise", 0.1);
        this->declare_parameter<double>("max_position_change_rate", 2.0);

        // 初始化TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
    }

    ~FireInfoNode()
    {
        stop_camera_threads();
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
        horizontal_fov_deg_ = this->get_parameter("horizontal_fov").as_double();
        vertical_fov_deg_ = this->get_parameter("vertical_fov").as_double();
        fire_tempture_threshold_ = this->get_parameter("fire_tempture_threshold").as_double();
        camera_read_timeout_ms_ = this->get_parameter("camera_read_timeout_ms").as_int();
        max_sync_time_diff_ms_ = this->get_parameter("max_sync_time_diff_ms").as_int();

        RCLCPP_INFO(this->get_logger(), "相机间距: %.4f 米 火源温度阈值：%.4f 读取超时：%d ms 最大同步差：%d ms",
                    camera_distance_, fire_tempture_threshold_, camera_read_timeout_ms_, max_sync_time_diff_ms_);

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

        start_camera_threads();

        int period = this->get_parameter("fire_info_timer_period_ms").as_int();
        pub_fire_info_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period),
            std::bind(&FireInfoNode::pub_fire_info, this));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        stop_camera_threads();
        pub_fire_info_timer_.reset();
        pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        stop_camera_threads();
        cleanup_cameras();
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        stop_camera_threads();
        cleanup_cameras();
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    void start_camera_threads()
    {
        if (lUserID_Left_ >= 0)
        {
            left_thread_control_.should_stop.store(false);
            left_thread_control_.thread_ready.store(false);
            left_thread_control_.worker_thread = std::thread(&FireInfoNode::left_camera_worker, this);
            left_thread_control_.wait_until_ready();
            RCLCPP_INFO(this->get_logger(), "左相机读取线程已启动并准备就绪");
        }

        if (lUserID_Right_ >= 0)
        {
            right_thread_control_.should_stop.store(false);
            right_thread_control_.thread_ready.store(false);
            right_thread_control_.worker_thread = std::thread(&FireInfoNode::right_camera_worker, this);
            right_thread_control_.wait_until_ready();
            RCLCPP_INFO(this->get_logger(), "右相机读取线程已启动并准备就绪");
        }
    }

    void stop_camera_threads()
    {
        if (left_thread_control_.worker_thread.joinable())
        {
            left_thread_control_.should_stop.store(true);
            left_camera_data_.cv_read_request.notify_all();
            left_thread_control_.worker_thread.join();
            RCLCPP_INFO(this->get_logger(), "左相机读取线程已停止");
        }

        if (right_thread_control_.worker_thread.joinable())
        {
            right_thread_control_.should_stop.store(true);
            right_camera_data_.cv_read_request.notify_all();
            right_thread_control_.worker_thread.join();
            RCLCPP_INFO(this->get_logger(), "右相机读取线程已停止");
        }
    }

    void left_camera_worker()
    {
        left_thread_control_.notify_ready();

        while (!left_thread_control_.should_stop.load())
        {
            try
            {
                std::unique_lock<std::mutex> lock(left_camera_data_.mutex);
                left_camera_data_.cv_read_request.wait(lock, [this]
                                                       { return left_camera_data_.read_requested.load() || left_thread_control_.should_stop.load(); });

                if (left_thread_control_.should_stop.load())
                {
                    break;
                }

                left_camera_data_.read_requested.store(false);
                lock.unlock();

                float max_temp, avg_temp, center_x, center_y;
                bool success = get_fire_info_from_camera(
                    lUserID_Left_, max_temp, avg_temp, center_x, center_y, "左相机");

                if (success)
                {
                    left_camera_data_.update(max_temp, avg_temp, center_x, center_y);
                    RCLCPP_DEBUG(this->get_logger(),
                                 "左相机数据更新: (%.3f,%.3f) %.1f°C",
                                 center_x, center_y, max_temp);
                }
                else
                {
                    left_camera_data_.invalidate();
                    RCLCPP_DEBUG(this->get_logger(), "左相机未检测到火源");
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "左相机线程异常: %s", e.what());
                left_camera_data_.invalidate();
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "左相机工作线程退出");
    }

    void right_camera_worker()
    {
        right_thread_control_.notify_ready();

        while (!right_thread_control_.should_stop.load())
        {
            try
            {
                std::unique_lock<std::mutex> lock(right_camera_data_.mutex);
                right_camera_data_.cv_read_request.wait(lock, [this]
                                                        { return right_camera_data_.read_requested.load() || right_thread_control_.should_stop.load(); });

                if (right_thread_control_.should_stop.load())
                {
                    break;
                }

                right_camera_data_.read_requested.store(false);
                lock.unlock();

                float max_temp, avg_temp, center_x, center_y;
                bool success = get_fire_info_from_camera(
                    lUserID_Right_, max_temp, avg_temp, center_x, center_y, "右相机");

                if (success)
                {
                    right_camera_data_.update(max_temp, avg_temp, center_x, center_y);
                    RCLCPP_DEBUG(this->get_logger(),
                                 "右相机数据更新: (%.3f,%.3f) %.1f°C",
                                 center_x, center_y, max_temp);
                }
                else
                {
                    right_camera_data_.invalidate();
                    RCLCPP_DEBUG(this->get_logger(), "右相机未检测到火源");
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "右相机线程异常: %s", e.what());
                right_camera_data_.invalidate();
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "右相机工作线程退出");
    }

    void pub_fire_info()
    {
        auto start_time = std::chrono::steady_clock::now();

        left_camera_data_.request_read();
        right_camera_data_.request_read();

        std::chrono::milliseconds timeout(camera_read_timeout_ms_);
        bool left_completed = left_camera_data_.wait_for_completion(timeout);
        bool right_completed = right_camera_data_.wait_for_completion(timeout);

        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        if (!left_completed || !right_completed)
        {
            RCLCPP_WARN(this->get_logger(),
                        "相机读取超时 - 左相机: %s, 右相机: %s, 耗时: %ld ms",
                        left_completed ? "完成" : "超时",
                        right_completed ? "完成" : "超时",
                        elapsed_ms);
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "两个相机都完成数据读取，耗时: %ld ms", elapsed_ms);

        float left_max, left_avg, left_x, left_y;
        float right_max, right_avg, right_x, right_y;
        std::chrono::steady_clock::time_point left_timestamp, right_timestamp;

        bool left_valid = left_camera_data_.get_data(left_max, left_avg, left_x, left_y, left_timestamp);
        bool right_valid = right_camera_data_.get_data(right_max, right_avg, right_x, right_y, right_timestamp);
        bool data_synchronized = true;
        if (left_valid && right_valid)
        {
            auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 left_timestamp > right_timestamp ? (left_timestamp - right_timestamp) : (right_timestamp - left_timestamp))
                                 .count();

            if (time_diff > max_sync_time_diff_ms_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "相机数据时间不同步，时间差: %ld ms > %d ms",
                            time_diff, max_sync_time_diff_ms_);
                data_synchronized = false;
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(),
                             "相机数据同步良好，时间差: %ld ms", time_diff);
            }
        }

        auto msg = fire_interfaces::msg::FireInfo();
        msg.header.stamp = this->now();
        msg.header.frame_id = "stereo_camera";

        if (left_valid && right_valid && data_synchronized)
        {
            msg.left_max = left_max;
            msg.left_avg = left_avg;
            msg.left_center_x = left_x;
            msg.left_center_y = left_y;
            msg.right_max = right_max;
            msg.right_avg = right_avg;
            msg.right_center_x = right_x;
            msg.right_center_y = right_y;

            float temp_diff = std::abs(left_max - right_max);

            RCLCPP_INFO(this->get_logger(),
                        "双相机同步检测 - 左相机: (%.3f,%.3f) %.1f°C, 右相机: (%.3f,%.3f) %.1f°C, 温差: %.1f°C, 总耗时: %ld ms",
                        left_x, left_y, left_max,
                        right_x, right_y, right_max,
                        temp_diff, elapsed_ms);

            if (temp_diff <= 5.0)
            {
                if (calculate_3d_fire_position(left_x, left_y, right_x, right_y,
                                               msg.fire_position, msg.fire_position_map) == 0)
                {
                    pub_->publish(msg);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "左右相机温差过大 (%.1f°C > 5.0°C)，跳过3D计算", temp_diff);
            }
        }
        else if (left_valid && right_valid && !data_synchronized)
        {
            RCLCPP_DEBUG(this->get_logger(), "等待相机数据同步...");
        }
        else if (left_valid || right_valid)
        {
            RCLCPP_DEBUG(this->get_logger(), "仅单个相机检测到火焰: 左=%s, 右=%s, 总耗时: %ld ms",
                         left_valid ? "是" : "否", right_valid ? "是" : "否", elapsed_ms);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "未检测到火焰，总耗时: %ld ms", elapsed_ms);
        }
    }

    bool login_camera(const std::string &ip, const std::string &username,
                      const std::string &password, int &userID, const std::string &name)
    {
        RCLCPP_INFO(this->get_logger(), "正在登录%s，IP: %s，用户名: %s",
                    name.c_str(), ip.c_str(), username.c_str());

        NET_DVR_USER_LOGIN_INFO loginInfo = {0};
        NET_DVR_DEVICEINFO_V40 deviceInfo = {0};

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

        if (struOutBuffer.byIsFreezedata)
        {
            RCLCPP_DEBUG(this->get_logger(), "%s温度数据被冻结", camera_name.c_str());
            return false;
        }

        max_temp = struOutBuffer.fMaxTemperature;
        if (max_temp <= fire_tempture_threshold_)
        {
            RCLCPP_DEBUG(this->get_logger(), "%s火源温度 %.2f°C 低于阈值 %.2f°C，跳过",
                         camera_name.c_str(), max_temp, fire_tempture_threshold_);
            return false;
        }
        avg_temp = struOutBuffer.fAverageTemperature;
        center_x = struOutBuffer.struHighestPoint.fX;
        center_y = struOutBuffer.struHighestPoint.fY;

        return true;
    }

    bool transform_to_world_frame(const geometry_msgs::msg::Point &camera_point,
                                  geometry_msgs::msg::Point &world_point)
    {
        try
        {
            std::string camera_frame = this->get_parameter("stereo_ir_camera_frame").as_string();
            std::string target_frame = "map";

            geometry_msgs::msg::PointStamped camera_point_stamped;
            camera_point_stamped.header.frame_id = camera_frame;
            camera_point_stamped.header.stamp = rclcpp::Time(0);
            camera_point_stamped.point = camera_point;

            geometry_msgs::msg::PointStamped world_point_stamped;
            world_point_stamped = tf_buffer_->transform(camera_point_stamped, target_frame);

            world_point = world_point_stamped.point;

            RCLCPP_DEBUG(this->get_logger(),
                         "TF转换成功: 相机坐标(%.3f, %.3f, %.3f) -> 世界坐标(%.3f, %.3f, %.3f)",
                         camera_point.x, camera_point.y, camera_point.z,
                         world_point.x, world_point.y, world_point.z);

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "TF转换失败: %s", ex.what());

            return fallback_coordinate_transform(camera_point, world_point);
        }
    }

    bool fallback_coordinate_transform(const geometry_msgs::msg::Point &camera_point,
                                       geometry_msgs::msg::Point &world_point)
    {
        world_point.x = camera_point.x;
        world_point.y = camera_point.y;
        world_point.z = camera_point.z;

        RCLCPP_DEBUG(this->get_logger(),
                     "使用备选坐标变换: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)",
                     camera_point.x, camera_point.y, camera_point.z,
                     world_point.x, world_point.y, world_point.z);

        return true;
    }

    void publish_fire_tf(const geometry_msgs::msg::Point &world_position)
    {
        if (!this->get_parameter("publish_fire_tf").as_bool())
        {
            return;
        }

        try
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->now();
            t.header.frame_id = "map";
            t.child_frame_id = this->get_parameter("fire_frame_id").as_string();

            t.transform.translation.x = world_position.x;
            t.transform.translation.y = world_position.y;
            t.transform.translation.z = world_position.z;

            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(t);

            RCLCPP_DEBUG(this->get_logger(),
                         "发布火焰位置TF: %s -> %s (%.3f, %.3f, %.3f)",
                         t.header.frame_id.c_str(), t.child_frame_id.c_str(),
                         world_position.x, world_position.y, world_position.z);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "发布火焰TF失败: %s", e.what());
        }
    }

    // 应用位置滤波器（针对世界坐标系）
    geometry_msgs::msg::Point applyPositionFilters(const geometry_msgs::msg::Point &world_position,
                                                   float left_temp, float right_temp)
    {
        if (!this->get_parameter("enable_position_filter").as_bool())
        {
            return world_position;
        }

        double confidence = calculateConfidence(world_position, left_temp, right_temp);

        if (isPositionChangeReasonable(world_position))
        {
            geometry_msgs::msg::Point kalman_filtered;
            kalman_filtered.x = kalman_x_.update(world_position.x);
            kalman_filtered.y = kalman_y_.update(world_position.y);
            kalman_filtered.z = kalman_z_.update(world_position.z);

            geometry_msgs::msg::Point final_filtered = position_filter_.addAndFilter(kalman_filtered, confidence);

            last_filtered_position_ = final_filtered;
            last_filter_time_ = std::chrono::steady_clock::now();

            return final_filtered;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "位置变化过大，使用上次滤波结果");
            return last_filtered_position_;
        }
    }

    double calculateConfidence(const geometry_msgs::msg::Point &position, float left_temp, float right_temp)
    {
        double confidence = 1.0;

        float temp_diff = std::abs(left_temp - right_temp);
        double temp_confidence = std::max(0.1, 1.0 - temp_diff / 10.0);

        double distance = std::sqrt(position.x * position.x + position.y * position.y + position.z * position.z);
        double distance_confidence = 1.0;
        if (distance < 0.5 || distance > 10.0)
        {
            distance_confidence = 0.3;
        }

        double stability_confidence = 1.0;
        if (position_filter_.size() > 3)
        {
            double change = calculateDistance(position, last_filtered_position_);
            if (change > 0.5)
            {
                stability_confidence = std::max(0.2, 1.0 - change / 2.0);
            }
        }

        confidence = temp_confidence * distance_confidence * stability_confidence;

        RCLCPP_DEBUG(this->get_logger(),
                     "置信度计算: 温度=%.2f, 距离=%.2f, 稳定性=%.2f, 总计=%.2f",
                     temp_confidence, distance_confidence, stability_confidence, confidence);

        return confidence;
    }

    bool isPositionChangeReasonable(const geometry_msgs::msg::Point &new_position)
    {
        if (last_filter_time_.time_since_epoch().count() == 0)
        {
            return true;
        }

        auto current_time = std::chrono::steady_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_filter_time_).count();

        if (time_diff <= 0)
            return true;

        double distance_change = calculateDistance(new_position, last_filtered_position_);
        double max_change_rate = this->get_parameter("max_position_change_rate").as_double();
        double max_allowed_change = max_change_rate * (time_diff / 1000.0);

        bool reasonable = distance_change <= max_allowed_change;

        if (!reasonable)
        {
            RCLCPP_WARN(this->get_logger(),
                        "位置变化过快: %.3f米 在 %ldms 内 (最大允许: %.3f米)",
                        distance_change, time_diff, max_allowed_change);
        }

        return reasonable;
    }

    double calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    int calculate_3d_fire_position(float left_x_ratio, float left_y_ratio,
                                   float right_x_ratio, float right_y_ratio,
                                   geometry_msgs::msg::Point &fire_position,
                                   geometry_msgs::msg::Point &fire_position_map)
    {
        static double last_distance;
        static time_t last_time = 0;
        try
        {
            RCLCPP_INFO(this->get_logger(),
                        "输入坐标 - 左相机: (%.3f, %.3f), 右相机: (%.3f, %.3f)",
                        left_x_ratio, left_y_ratio, right_x_ratio, right_y_ratio);

            float left_max = 0.0f, left_avg = 0.0f, left_x = 0.0f, left_y = 0.0f;
            float right_max = 0.0f, right_avg = 0.0f, right_x = 0.0f, right_y = 0.0f;
            std::chrono::steady_clock::time_point left_timestamp, right_timestamp;

            left_camera_data_.get_data(left_max, left_avg, left_x, left_y, left_timestamp);
            right_camera_data_.get_data(right_max, right_avg, right_x, right_y, right_timestamp);

            double left_azimuth_deg = (left_x_ratio - 0.5) * this->horizontal_fov_deg_;
            double left_elevation_deg = (0.5 - left_y_ratio) * this->vertical_fov_deg_;
            double right_azimuth_deg = (right_x_ratio - 0.5) * this->horizontal_fov_deg_;
            double right_elevation_deg = (0.5 - right_y_ratio) * this->vertical_fov_deg_;

            RCLCPP_INFO(this->get_logger(),
                        "左相机角度: 方位角=%.2f°, 俯仰角=%.2f°",
                        left_azimuth_deg, left_elevation_deg);
            RCLCPP_INFO(this->get_logger(),
                        "右相机角度: 方位角=%.2f°, 俯仰角=%.2f°",
                        right_azimuth_deg, right_elevation_deg);

            double baseline = camera_distance_;
            double angle_diff_deg = left_azimuth_deg - right_azimuth_deg;
            double angle_diff_rad = angle_diff_deg * M_PI / 180.0;

            RCLCPP_INFO(this->get_logger(),
                        "视差角: %.3f度 (%.3f弧度), 基线距离: %.4f米",
                        angle_diff_deg, angle_diff_rad, baseline);

            if (std::abs(angle_diff_rad) < 0.01)
            {
                RCLCPP_WARN(this->get_logger(), "视差角太小，无法准确计算深度");
                fire_position.x = 0.0;
                fire_position.y = 0.0;
                fire_position.z = 10.0;
                return 0;
            }

            double distance = baseline / 2.0 / tan(std::abs(angle_diff_rad) / 2.0);

            if (last_distance == 0)
            {
                last_distance = distance;
            }
            if (std::abs((distance - last_distance) / last_distance) > 0.3 && time(NULL) - last_time < 2)
            {
                RCLCPP_WARN(this->get_logger(), "距离变化过大 distance: %.3f, last_distance: %.3f", distance, last_distance);
                distance = last_distance * (distance > last_distance ? 1.3 : 0.7);
            }
            if (distance < 0.1)
            {
                RCLCPP_WARN(this->get_logger(), "计算得到的距离过小，跳过本次测量");
                return -1;
            }

            last_distance = distance;
            last_time = time(NULL);

            double avg_azimuth_deg = (left_azimuth_deg + right_azimuth_deg) / 2.0;
            double avg_elevation_deg = (left_elevation_deg + right_elevation_deg) / 2.0;
            double avg_azimuth_rad = avg_azimuth_deg * M_PI / 180.0;
            double avg_elevation_rad = avg_elevation_deg * M_PI / 180.0;

            geometry_msgs::msg::Point raw_position;
            raw_position.x = distance * cos(avg_azimuth_rad) * cos(avg_elevation_rad);
            raw_position.y = -distance * sin(avg_azimuth_rad) * cos(avg_elevation_rad);
            raw_position.z = distance * sin(avg_elevation_rad);

            RCLCPP_INFO(this->get_logger(),
                        "distance:%.3f 修正后3D位置: (%.3f, %.3f, %.3f) 米 [x=前后, y=左右, z=上下]",
                        distance, raw_position.x, raw_position.y, raw_position.z);

            fire_position = raw_position;

            geometry_msgs::msg::Point world_position;
            if (transform_to_world_frame(raw_position, world_position))
            {
                geometry_msgs::msg::Point filtered_world_position = applyPositionFilters(world_position, left_max, right_max);

                RCLCPP_INFO(this->get_logger(),
                            "滤波后位置(世界坐标系): (%.3f, %.3f, %.3f) 米",
                            filtered_world_position.x, filtered_world_position.y, filtered_world_position.z);

                fire_position_map = filtered_world_position;
                publish_fire_tf(filtered_world_position);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "TF转换失败，使用相机坐标系结果");
                fire_position_map = raw_position;
            }

            return 0;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "计算3D位置时发生错误: %s", e.what());
            return -1;
        }
    }

    // 成员变量声明
    rclcpp_lifecycle::LifecyclePublisher<fire_interfaces::msg::FireInfo>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr pub_fire_info_timer_;

    int lUserID_Left_ = -1;
    int lUserID_Right_ = -1;

    double camera_distance_;
    double horizontal_fov_deg_;
    double vertical_fov_deg_;
    double fire_tempture_threshold_;
    int camera_read_timeout_ms_;
    int max_sync_time_diff_ms_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    CameraThreadControl left_thread_control_;
    CameraThreadControl right_thread_control_;

    CameraData left_camera_data_;
    CameraData right_camera_data_;

    SlidingWindowFilter position_filter_;
    SimpleKalmanFilter kalman_x_, kalman_y_, kalman_z_;
    geometry_msgs::msg::Point last_filtered_position_;
    std::chrono::steady_clock::time_point last_filter_time_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FireInfoNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}