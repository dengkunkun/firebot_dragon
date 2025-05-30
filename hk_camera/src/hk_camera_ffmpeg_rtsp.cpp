#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/header.hpp>

extern "C"
{
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

class HKCameraRTSPNode : public rclcpp::Node
{
private:
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher color_image_pub_;
    image_transport::Publisher ir_image_pub_;
    
    // 添加CameraInfo支持
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_service_;
    
    // 相机标定信息
    sensor_msgs::msg::CameraInfo camera_info_;
    bool camera_info_loaded_ = false;

public:
    HKCameraRTSPNode() : Node("hk_camera_rtsp_node")
    {
        // 声明参数
        this->declare_parameter("ip", "HM-TD2B28T-3-T120250109AACHEA3538378.local");
        this->declare_parameter("username", "admin");
        this->declare_parameter("password", "ubuntu_define");
        this->declare_parameter("rtsp_port", 554);
        this->declare_parameter("color_stream_path", "/Streaming/Channels/1");
        this->declare_parameter("ir_stream_path", "/Streaming/Channels/2");
        this->declare_parameter("enable_color_stream", true);
        this->declare_parameter("enable_ir_stream", false);
        this->declare_parameter("frame_rate", 25.0);
        this->declare_parameter("timeout_ms", 5000);
        this->declare_parameter("jpeg_quality", 80);
        this->declare_parameter("png_compression", 3);

        // 获取参数
        ip_ = this->get_parameter("ip").as_string();
        username_ = this->get_parameter("username").as_string();
        password_ = this->get_parameter("password").as_string();
        rtsp_port_ = this->get_parameter("rtsp_port").as_int();
        color_stream_path_ = this->get_parameter("color_stream_path").as_string();
        ir_stream_path_ = this->get_parameter("ir_stream_path").as_string();
        enable_color_stream_ = this->get_parameter("enable_color_stream").as_bool();
        enable_ir_stream_ = this->get_parameter("enable_ir_stream").as_bool();
        frame_rate_ = this->get_parameter("frame_rate").as_double();
        timeout_ms_ = this->get_parameter("timeout_ms").as_int();
        jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
        png_compression_ = this->get_parameter("png_compression").as_int();

        // 构建RTSP URL
        color_rtsp_url_ = "rtsp://" + username_ + ":" + password_ + "@" + ip_ + ":" +
                          std::to_string(rtsp_port_) + color_stream_path_;
        ir_rtsp_url_ = "rtsp://" + username_ + ":" + password_ + "@" + ip_ + ":" +
                       std::to_string(rtsp_port_) + ir_stream_path_;

        RCLCPP_INFO(this->get_logger(), "Color stream URL: %s", color_rtsp_url_.c_str());
        RCLCPP_INFO(this->get_logger(), "IR stream URL: %s", ir_rtsp_url_.c_str());

        // 初始化默认相机信息
        initDefaultCameraInfo();
        
        // 初始化FFmpeg
        initFFmpeg();
    }

    // 添加初始化函数，在main中调用
    void initialize()
    {
        // 初始化image_transport（在shared_ptr创建后）
        it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());

        // 创建发布者 - 使用标准话题名
        if (enable_color_stream_)
        {
            color_image_pub_ = it_->advertise("image_raw", 1);
            
            // 创建CameraInfo发布者
            camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                "camera_info", 10);
                
            // 创建set_camera_info服务 - 添加camera前缀
            set_camera_info_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
                "camera/set_camera_info",  // 改为带camera前缀
                std::bind(&HKCameraRTSPNode::setCameraInfoCallback, this,
                         std::placeholders::_1, std::placeholders::_2));
        }
        
        if (enable_ir_stream_)
        {
            ir_image_pub_ = it_->advertise("ir/image_raw", 1);
        }

        // 启动视频流处理
        if (enable_color_stream_)
        {
            color_stream_thread_ = std::thread(&HKCameraRTSPNode::processColorStream, this);
        }
        if (enable_ir_stream_)
        {
            ir_stream_thread_ = std::thread(&HKCameraRTSPNode::processIRStream, this);
        }
    }

    ~HKCameraRTSPNode()
    {
        running_ = false;

        if (color_stream_thread_.joinable())
        {
            color_stream_thread_.join();
        }
        if (ir_stream_thread_.joinable())
        {
            ir_stream_thread_.join();
        }

        cleanup();
    }

private:
    void initDefaultCameraInfo()
    {
        camera_info_.header.frame_id = "camera_optical_frame";
        camera_info_.width = 1920;  // 将根据实际图像更新
        camera_info_.height = 1080; // 将根据实际图像更新
        camera_info_.distortion_model = "plumb_bob";
        
        // 默认内参矩阵（标定前的估计值）
        camera_info_.k = {
            1000.0, 0.0, 960.0,
            0.0, 1000.0, 540.0,
            0.0, 0.0, 1.0
        };
        
        // 默认畸变系数
        camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        
        // 矫正矩阵（单目相机为单位矩阵）
        camera_info_.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
        
        // 投影矩阵
        camera_info_.p = {
            1000.0, 0.0, 960.0, 0.0,
            0.0, 1000.0, 540.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        };
    }

    void setCameraInfoCallback(
        const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
        std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
    {
        camera_info_ = request->camera_info;
        camera_info_loaded_ = true;
        
        response->success = true;
        response->status_message = "Camera info set successfully";
        
        RCLCPP_INFO(this->get_logger(), "Camera calibration updated successfully");
        
        // 可选：保存标定结果到文件
        saveCameraInfoToFile();
    }

    void saveCameraInfoToFile()
    {
        // 这里可以实现保存到YAML文件的逻辑
        RCLCPP_INFO(this->get_logger(), "Camera calibration data updated");
    }

    void initFFmpeg()
    {
        // av_register_all(); // 在FFmpeg 4.0+中已弃用，不再需要
        avformat_network_init();
        av_log_set_level(AV_LOG_WARNING);
    }

    void cleanup()
    {
        if (color_sws_ctx_)
        {
            sws_freeContext(color_sws_ctx_);
            color_sws_ctx_ = nullptr;
        }
        if (ir_sws_ctx_)
        {
            sws_freeContext(ir_sws_ctx_);
            ir_sws_ctx_ = nullptr;
        }
        avformat_network_deinit();
    }

    bool openRTSPStream(const std::string &url, AVFormatContext *&format_ctx,
                        AVCodecContext *&codec_ctx, int &video_stream_index)
    {
        // 设置网络超时
        AVDictionary *opts = nullptr;
        av_dict_set(&opts, "rtsp_transport", "udp", 0);  // 改为udp
        av_dict_set(&opts, "buffer_size", "1024000", 0); // 增加缓冲区
        av_dict_set(&opts, "max_delay", "100000", 0);    // 减少延迟
        av_dict_set(&opts, "hwaccel", "auto", 0);

        // 打开输入流
        format_ctx = avformat_alloc_context();
        if (avformat_open_input(&format_ctx, url.c_str(), nullptr, &opts) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream: %s", url.c_str());
            av_dict_free(&opts);
            return false;
        }
        av_dict_free(&opts);

        // 获取流信息
        if (avformat_find_stream_info(format_ctx, nullptr) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to find stream info");
            avformat_close_input(&format_ctx);
            return false;
        }

        // 查找视频流
        video_stream_index = -1;
        for (unsigned int i = 0; i < format_ctx->nb_streams; i++)
        {
            if (format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
            {
                video_stream_index = i;
                break;
            }
        }

        if (video_stream_index == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "No video stream found");
            avformat_close_input(&format_ctx);
            return false;
        }

        // 获取解码器 - 修复const类型转换问题
        AVCodecParameters *codecpar = format_ctx->streams[video_stream_index]->codecpar;
        const AVCodec *codec = avcodec_find_decoder(codecpar->codec_id);
        if (!codec)
        {
            RCLCPP_ERROR(this->get_logger(), "Codec not found");
            avformat_close_input(&format_ctx);
            return false;
        }

        // 创建解码器上下文
        codec_ctx = avcodec_alloc_context3(codec);
        if (avcodec_parameters_to_context(codec_ctx, codecpar) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to copy codec parameters");
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&format_ctx);
            return false;
        }

        // 打开解码器
        if (avcodec_open2(codec_ctx, codec, nullptr) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open codec");
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&format_ctx);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully opened RTSP stream: %dx%d",
                    codec_ctx->width, codec_ctx->height);
        return true;
    }

    void processColorStream()
    {
        processStream(color_rtsp_url_, color_image_pub_, "color");
    }

    void processIRStream()
    {
        processStream(ir_rtsp_url_, ir_image_pub_, "ir");
    }

    void processStream(const std::string &url,
                      image_transport::Publisher &publisher,
                      const std::string &stream_name)
    {
        AVFormatContext *format_ctx = nullptr;
        AVCodecContext *codec_ctx = nullptr;
        int video_stream_index = -1;

        while (running_ && rclcpp::ok())
        {
            // 尝试打开流
            if (!openRTSPStream(url, format_ctx, codec_ctx, video_stream_index))
            {
                RCLCPP_WARN(this->get_logger(), "Retrying %s stream connection in 3 seconds...", stream_name.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(3));
                continue;
            }

            AVPacket *packet = av_packet_alloc();
            AVFrame *frame = av_frame_alloc();
            AVFrame *frame_bgr = av_frame_alloc();

            // 初始化转换上下文
            SwsContext *sws_ctx = sws_getContext(
                codec_ctx->width, codec_ctx->height, codec_ctx->pix_fmt,
                codec_ctx->width, codec_ctx->height, AV_PIX_FMT_BGR24,
                SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

            if (!sws_ctx)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize sws context for %s stream", stream_name.c_str());
                break;
            }

            // 预分配BGR帧缓冲区
            int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_BGR24, codec_ctx->width, codec_ctx->height, 1);
            uint8_t *buffer = (uint8_t *)av_malloc(num_bytes * sizeof(uint8_t));
            av_image_fill_arrays(frame_bgr->data, frame_bgr->linesize, buffer, AV_PIX_FMT_BGR24,
                                 codec_ctx->width, codec_ctx->height, 1);

            RCLCPP_INFO(this->get_logger(), "%s compressed stream started", stream_name.c_str());

            // 帧计数和时间统计
            int frame_count = 0;
            auto start_time = std::chrono::steady_clock::now();

            // 主处理循环
            while (running_ && rclcpp::ok())
            {
                if (av_read_frame(format_ctx, packet) >= 0)
                {
                    if (packet->stream_index == video_stream_index)
                    {
                        if (avcodec_send_packet(codec_ctx, packet) == 0)
                        {
                            while (avcodec_receive_frame(codec_ctx, frame) == 0)
                            {
                                // 转换像素格式
                                sws_scale(sws_ctx, frame->data, frame->linesize, 0, codec_ctx->height,
                                          frame_bgr->data, frame_bgr->linesize);

                                // 转换为OpenCV Mat
                                cv::Mat cv_image(codec_ctx->height, codec_ctx->width, CV_8UC3,
                                                 frame_bgr->data[0], frame_bgr->linesize[0]);

                                // 发布图像（image_transport会自动处理压缩）
                                publishImage(cv_image, publisher, stream_name);

                                frame_count++;

                                // 每50帧统计一次帧率
                                if (frame_count % 50 == 0)
                                {
                                    auto now = std::chrono::steady_clock::now();
                                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
                                    double fps = 50.0 / (duration.count() / 1000.0);
                                    RCLCPP_INFO(this->get_logger(), "%s compressed stream FPS: %.2f", stream_name.c_str(), fps);
                                    start_time = now;
                                }
                            }
                        }
                    }
                    av_packet_unref(packet);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "%s stream connection lost, reconnecting...", stream_name.c_str());
                    break;
                }
            }

            // 清理资源
            av_free(buffer);
            av_frame_free(&frame);
            av_frame_free(&frame_bgr);
            av_packet_free(&packet);
            sws_freeContext(sws_ctx);
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&format_ctx);

            if (running_ && rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Reconnecting %s stream in 2 seconds...", stream_name.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }
    }

    void publishImage(const cv::Mat &cv_image,
                     image_transport::Publisher &publisher,
                     const std::string &stream_name)
    {
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_optical_frame";
        msg->height = cv_image.rows;
        msg->width = cv_image.cols;
        msg->encoding = "bgr8";
        msg->is_bigendian = false;
        msg->step = cv_image.step;
        
        size_t data_size = cv_image.rows * cv_image.step;
        msg->data.resize(data_size);
        std::memcpy(msg->data.data(), cv_image.data, data_size);

        publisher.publish(*msg);
        
        // 同时发布CameraInfo（只对彩色相机）
        if (stream_name == "color" && camera_info_pub_)
        {
            camera_info_.header = msg->header;
            camera_info_.width = cv_image.cols;
            camera_info_.height = cv_image.rows;
            camera_info_pub_->publish(camera_info_);
        }
        
        // 添加统计信息
        static int frame_count = 0;
        static auto last_time = std::chrono::steady_clock::now();
        
        frame_count++;
        if (frame_count % 30 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time);
            double fps = 30000.0 / duration.count();
            RCLCPP_INFO(this->get_logger(), "%s: %.1f FPS, %dx%d", 
                       stream_name.c_str(), fps, cv_image.cols, cv_image.rows);
            last_time = now;
        }
    }

    // 参数
    std::string ip_;
    std::string username_;
    std::string password_;
    int rtsp_port_;
    std::string color_stream_path_;
    std::string ir_stream_path_;
    std::string color_rtsp_url_;
    std::string ir_rtsp_url_;
    bool enable_color_stream_;
    bool enable_ir_stream_;
    double frame_rate_;
    int timeout_ms_;
    int jpeg_quality_;      // JPEG压缩质量
    int png_compression_;   // PNG压缩级别

    // ROS发布者 - 改为压缩图像
    // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr color_image_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr ir_image_pub_;

    // FFmpeg相关
    SwsContext *color_sws_ctx_ = nullptr;
    SwsContext *ir_sws_ctx_ = nullptr;

    // 线程控制
    std::atomic<bool> running_{true};
    std::thread color_stream_thread_;
    std::thread ir_stream_thread_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<HKCameraRTSPNode>();
        
        // 在shared_ptr创建后初始化image_transport
        node->initialize();
        
        RCLCPP_INFO(node->get_logger(), "HK Camera RTSP Node started");
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hk_camera_rtsp"), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}