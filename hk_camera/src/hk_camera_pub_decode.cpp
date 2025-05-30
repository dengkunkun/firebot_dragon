#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "HCNetSDK.h"
// #include "LinuxPlayM4.h"  //仅windows下可用

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}
#include <arpa/inet.h>  // 用于 ntohs, ntohl
#include <netinet/in.h> // 网络字节序相关
#include <queue>
#include <mutex>
#include <thread>
#include <memory>

class VideoDecoder {
public:
    VideoDecoder() {
        // FFmpeg 4.0+ 版本不需要手动注册，自动初始化
        // 如果是旧版本，取消注释下面两行
        #if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 9, 100)
            av_register_all();
            avcodec_register_all();
        #endif
        
        // 查找H264解码器
        codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec_) {
            throw std::runtime_error("H264 decoder not found");
        }
        
        // 创建解码器上下文
        codec_ctx_ = avcodec_alloc_context3(codec_);
        if (!codec_ctx_) {
            throw std::runtime_error("Could not allocate video codec context");
        }
        
        // 打开解码器
        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
            throw std::runtime_error("Could not open codec");
        }
        
        // 分配帧
        frame_ = av_frame_alloc();
        if (!frame_) {
            throw std::runtime_error("Could not allocate video frame");
        }
        
        // 分配数据包
        packet_ = av_packet_alloc();
        if (!packet_) {
            throw std::runtime_error("Could not allocate packet");
        }
    }
    
    ~VideoDecoder() {
        if (sws_ctx_) {
            sws_freeContext(sws_ctx_);
        }
        if (frame_) {
            av_frame_free(&frame_);
        }
        if (packet_) {
            av_packet_free(&packet_);
        }
        if (codec_ctx_) {
            avcodec_free_context(&codec_ctx_);
        }
    }
    
    cv::Mat decode(const uint8_t* data, size_t size, uint32_t rtp_timestamp) {
        // 设置数据包
        packet_->data = const_cast<uint8_t*>(data);
        packet_->size = size;
        packet_->pts = rtp_timestamp;
        
        // 发送数据包到解码器
        int ret = avcodec_send_packet(codec_ctx_, packet_);
        if (ret < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("decoder"), "Error sending packet to decoder: %d", ret);
            return cv::Mat();
        }
        
        // 接收解码后的帧
        ret = avcodec_receive_frame(codec_ctx_, frame_);
        if (ret < 0) {
            if (ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
                RCLCPP_ERROR(rclcpp::get_logger("decoder"), "Error receiving frame from decoder: %d", ret);
            }
            return cv::Mat();
        }
        
        return convert_frame_to_mat();
    }
    
private:
    cv::Mat convert_frame_to_mat() {
        int width = codec_ctx_->width;
        int height = codec_ctx_->height;
        
        if (width <= 0 || height <= 0) {
            return cv::Mat();
        }
        
        // 初始化SwScale上下文（YUV到BGR转换）
        if (!sws_ctx_ || last_width_ != width || last_height_ != height) {
            if (sws_ctx_) {
                sws_freeContext(sws_ctx_);
            }
            
            sws_ctx_ = sws_getContext(
                width, height, codec_ctx_->pix_fmt,
                width, height, AV_PIX_FMT_BGR24,
                SWS_BILINEAR, nullptr, nullptr, nullptr
            );
            
            last_width_ = width;
            last_height_ = height;
        }
        
        if (!sws_ctx_) {
            RCLCPP_ERROR(rclcpp::get_logger("decoder"), "Could not initialize SwScale context");
            return cv::Mat();
        }
        
        // 创建BGR图像
        cv::Mat bgr_image(height, width, CV_8UC3);
        uint8_t* dst_data[4] = { bgr_image.data, nullptr, nullptr, nullptr };
        int dst_linesize[4] = { width * 3, 0, 0, 0 };
        
        // 转换YUV到BGR
        sws_scale(sws_ctx_,
                  frame_->data, frame_->linesize,
                  0, height,
                  dst_data, dst_linesize);
        
        return bgr_image;
    }
    
    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVPacket* packet_ = nullptr;
    SwsContext* sws_ctx_ = nullptr;
    int last_width_ = 0;
    int last_height_ = 0;
};

struct RTPHeader {
    uint8_t version : 2;
    uint8_t padding : 1;
    uint8_t extension : 1;
    uint8_t csrc_count : 4;
    uint8_t marker : 1;
    uint8_t payload_type : 7;
    uint16_t sequence_number;
    uint32_t timestamp;
    uint32_t ssrc;
} __attribute__((packed));

class RTPProcessor {
public:
    struct RTPPacket {
        RTPHeader header;
        std::vector<uint8_t> payload;
        uint32_t timestamp;
    };
    
    bool parse_rtp_packet(const uint8_t* data, size_t size, RTPPacket& packet) {
        if (size < sizeof(RTPHeader)) {
            return false;
        }
        
        // 解析RTP头
        const RTPHeader* header = reinterpret_cast<const RTPHeader*>(data);
        packet.header = *header;
        
        // 网络字节序转换
        packet.header.sequence_number = ntohs(header->sequence_number);
        packet.header.timestamp = ntohl(header->timestamp);
        packet.header.ssrc = ntohl(header->ssrc);
        
        // 提取负载
        size_t header_size = sizeof(RTPHeader) + header->csrc_count * 4;
        if (header->extension) {
            // 处理扩展头（如果需要）
            header_size += 4; // 基本扩展头大小
        }
        
        if (size <= header_size) {
            return false;
        }
        
        packet.payload.assign(data + header_size, data + size);
        packet.timestamp = packet.header.timestamp;
        
        return true;
    }
    
    std::vector<uint8_t> assemble_frame(const RTPPacket& packet) {
        // 简单的帧组装逻辑
        // 对于H264，需要处理FU-A分片等
        if (packet.header.marker) {
            // 帧结束标记
            current_frame_.insert(current_frame_.end(), 
                                packet.payload.begin(), 
                                packet.payload.end());
            
            std::vector<uint8_t> complete_frame = current_frame_;
            current_frame_.clear();
            return complete_frame;
        } else {
            // 继续组装帧
            current_frame_.insert(current_frame_.end(), 
                                packet.payload.begin(), 
                                packet.payload.end());
            return std::vector<uint8_t>(); // 返回空表示帧未完成
        }
    }
    
private:
    std::vector<uint8_t> current_frame_;
};


using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr g_pub;
static rclcpp::Clock::SharedPtr g_clock;
static std::unique_ptr<VideoDecoder> g_decoder;
static std::unique_ptr<RTPProcessor> g_rtp_processor;
static std::mutex g_decode_mutex;

static int lPort = -1;

// void CALLBACK DecCBFun(int nPort, char *pBuf, int nSize,
//                        FRAME_INFO *pFrameInfo,
//                        void *nReserved1, int /*nReserved2*/)
// {
//     if (pFrameInfo->nType == T_YV12)
//     {
//         // 解码后的视频数据为YV12格式，需转换为BGR
//         int width = pFrameInfo->nWidth;
//         int height = pFrameInfo->nHeight;
//         cv::Mat yv12(height + height / 2, width, CV_8UC1, (unsigned char *)pBuf);
//         cv::Mat bgr;
//         cv::cvtColor(yv12, bgr, cv::COLOR_YUV2BGR_YV12);

//         auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();
//         msg->header.stamp = g_clock->now();
//         g_pub->publish(*msg);
//     }
// }

// void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *dwUser)
// {
//     switch (dwDataType)
//     {
//     case NET_DVR_SYSHEAD:
//         if (lPort >= 0)
//             break;
//         if (!PlayM4_GetPort(&lPort))
//             break;
//         if (dwBufSize > 0)
//         {
//             if (!PlayM4_SetStreamOpenMode(lPort, STREAME_REALTIME))
//                 break;
//             if (!PlayM4_OpenStream(lPort, pBuffer, dwBufSize, 1024 * 1024))
//                 break;
//             PlayM4_SetDecCallBackExMend(lPort, DecCBFun, NULL, 0, NULL);
//             if (!PlayM4_Play(lPort, 0))
//                 break;
//         }
//         break;
//     case NET_DVR_STREAMDATA:
//     default:
//         if (dwBufSize > 0 && lPort != -1)
//         {
//             PlayM4_InputData(lPort, pBuffer, dwBufSize);
//         }
//         break;
//     }
// }
void fStdDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, DWORD dwUser)
{
    [[maybe_unused]] LONG unused_handle = lRealHandle; // 标记为可能未使用

    std::lock_guard<std::mutex> lock(g_decode_mutex);
    
    switch (dwDataType)
    {
    case NET_DVR_STD_VIDEODATA:
        {
            if (!g_decoder || !g_rtp_processor || !g_pub || !g_pub->is_activated()) {
                return;
            }
            
            try {
                // 解析RTP包
                RTPProcessor::RTPPacket rtp_packet;
                if (!g_rtp_processor->parse_rtp_packet(pBuffer, dwBufSize, rtp_packet)) {
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("decoder"), *g_clock, 1000,
                                        "Failed to parse RTP packet");
                    return;
                }
                
                // 组装完整帧
                auto frame_data = g_rtp_processor->assemble_frame(rtp_packet);
                if (frame_data.empty()) {
                    return; // 帧未完成，继续等待
                }
                
                // 解码帧
                cv::Mat decoded_image = g_decoder->decode(frame_data.data(), frame_data.size(), 
                                                        rtp_packet.timestamp);
                
                if (!decoded_image.empty()) {
                    // 创建ROS消息
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", decoded_image).toImageMsg();
                    
                    // 设置时间戳（这里简化处理，实际可能需要转换RTP时间戳）
                    msg->header.stamp = g_clock->now();
                    msg->header.frame_id = "camera_frame";
                    
                    // 发布图像
                    g_pub->publish(*msg);
                    
                    RCLCPP_DEBUG(rclcpp::get_logger("decoder"), 
                               "Published image: %dx%d, RTP timestamp: %u", 
                               decoded_image.cols, decoded_image.rows, rtp_packet.timestamp);
                }
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("decoder"), *g_clock, 1000,
                                     "Decode error: %s", e.what());
            }
        }
        break;
        
    case NET_DVR_STD_AUDIODATA:
        // 音频数据处理（如果需要）
        break;
        
    case NET_DVR_SYSHEAD:
        RCLCPP_INFO(rclcpp::get_logger("decoder"), "Received system header");
        break;
        
    case NET_DVR_STREAMDATA:
        // 复合流数据
        break;
        
    case NET_DVR_PRIVATE_DATA:
        // 私有数据
        break;
        
    default:
        // 其他数据类型（音频编码等）
        RCLCPP_DEBUG(rclcpp::get_logger("decoder"), "Received data type: 0x%02X", dwDataType);
        break;
    }
}
class HKCameraDecodeNode : public LifecycleNode
{
public:
    HKCameraDecodeNode() : LifecycleNode("hk_camera_pub_decode") {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        g_pub = this->create_publisher<sensor_msgs::msg::Image>("hk_camera/image_raw", 10);
        g_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        
        // 初始化解码器和RTP处理器
        try {
            g_decoder = std::make_unique<VideoDecoder>();
            g_rtp_processor = std::make_unique<RTPProcessor>();
            RCLCPP_INFO(this->get_logger(), "Video decoder and RTP processor initialized");
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize decoder: %s", e.what());
            return CallbackReturn::FAILURE;
        }

        // 海康SDK初始化
        NET_DVR_Init();
        NET_DVR_USER_LOGIN_INFO loginInfo = {0};
        NET_DVR_DEVICEINFO_V40 deviceInfo = {0};
        strcpy(loginInfo.sDeviceAddress, "HM-TD2B28T-3-T120250109AACHEA3538378.local");
        strcpy(loginInfo.sUserName, "admin");
        strcpy(loginInfo.sPassword, "ubuntu_define");
        loginInfo.wPort = 8000;
        loginInfo.bUseAsynLogin = false;

        lUserID_ = NET_DVR_Login_V40(&loginInfo, &deviceInfo);
        if (lUserID_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Login failed: %d", NET_DVR_GetLastError());
            return CallbackReturn::FAILURE;
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully logged in to camera");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        g_pub->on_activate();

        NET_DVR_PREVIEWINFO struPlayInfo = {0};
        struPlayInfo.hPlayWnd = 0;
        struPlayInfo.lChannel = 1;
        struPlayInfo.byPreviewMode = 0;
        struPlayInfo.dwLinkMode = 3;    // RTP方式
        struPlayInfo.bBlocked = 0;
        struPlayInfo.dwDisplayBufNum = 5;

        lRealPlayHandle_ = NET_DVR_RealPlay_V40(lUserID_, &struPlayInfo, NULL, NULL);
        if (lRealPlayHandle_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "NET_DVR_RealPlay_V40 error: %d", NET_DVR_GetLastError());
            return CallbackReturn::FAILURE;
        }
        
        // 设置标准数据回调
        if (!NET_DVR_SetStandardDataCallBack(lRealPlayHandle_, fStdDataCallBack, 0)) {
            RCLCPP_ERROR(this->get_logger(), "NET_DVR_SetStandardDataCallBack error: %d", NET_DVR_GetLastError());
            return CallbackReturn::FAILURE;
        }
        
        RCLCPP_INFO(this->get_logger(), "Camera streaming started with RTP decoding");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        if (lRealPlayHandle_ >= 0) {
            NET_DVR_StopRealPlay(lRealPlayHandle_);
            lRealPlayHandle_ = -1;
            RCLCPP_INFO(this->get_logger(), "Stopped camera streaming");
        }
        g_pub->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        // 清理解码器
        g_decoder.reset();
        g_rtp_processor.reset();
        
        if (lUserID_ >= 0) {
            NET_DVR_Logout(lUserID_);
            lUserID_ = -1;
        }
        NET_DVR_Cleanup();
        g_pub.reset();
        g_clock.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        return on_cleanup(rclcpp_lifecycle::State());
    }

private:
    LONG lUserID_ = -1;
    LONG lRealPlayHandle_ = -1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HKCameraDecodeNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}