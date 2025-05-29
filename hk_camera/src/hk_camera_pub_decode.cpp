#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "HCNetSDK.h"
// #include "LinuxPlayM4.h"  //仅windows下可用

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr g_pub;
static rclcpp::Clock::SharedPtr g_clock;

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
void fStdDataCallBack (LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer,DWORD dwBufSize,DWORD dwUser)
{

}
class HKCameraDecodeNode : public LifecycleNode
{
public:
    HKCameraDecodeNode() : LifecycleNode("hk_camera_pub_decode") {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        g_pub = this->create_publisher<sensor_msgs::msg::Image>("hk_camera/image_raw", 10);
        g_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        NET_DVR_Init();
        NET_DVR_USER_LOGIN_INFO loginInfo = {0};
        NET_DVR_DEVICEINFO_V40 deviceInfo = {0};
        strcpy(loginInfo.sDeviceAddress, "192.168.10.8");
        strcpy(loginInfo.sUserName, "admin");
        strcpy(loginInfo.sPassword, "ubuntu_define");
        loginInfo.wPort = 8000;
        loginInfo.bUseAsynLogin = false;

        lUserID_ = NET_DVR_Login_V40(&loginInfo, &deviceInfo);
        if (lUserID_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Login failed: %d", NET_DVR_GetLastError());
            return CallbackReturn::FAILURE;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        g_pub->on_activate();

        DWORD dwUser=0; 
        NET_DVR_PREVIEWINFO struPlayInfo = {0};
#if (defined(_WIN32) || defined(_WIN_WCE))
        struPlayInfo.hPlayWnd = NULL;
#elif defined(__linux__)
        struPlayInfo.hPlayWnd = 0;
#endif
        struPlayInfo.lChannel = 1;  //通道号
        struPlayInfo.byPreviewMode=0; //预览模式，0-正常预览，1-延迟预览
        struPlayInfo.dwLinkMode = 0; // 0：TCP方式,1：UDP方式,2：多播方式,3 - RTP方式，4-RTP/RTSP,5-RSTP/HTTP
        struPlayInfo.bBlocked = 0; //0-非阻塞取流, 1-阻塞取流,
        struPlayInfo.dwDisplayBufNum = 5;
        struPlayInfo.byProtoType; //应用层取流协议，0-私有协议，1-RTSP协议,2-SRTP码流加密

        lRealPlayHandle_ = NET_DVR_RealPlay_V40(lUserID_, &struPlayInfo, NULL, NULL);
        if (lRealPlayHandle_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "NET_DVR_RealPlay_V40 error: %d", NET_DVR_GetLastError());
            return CallbackReturn::FAILURE;
        }
        if(NET_DVR_SetStandardDataCallBack(lUserID_,fStdDataCallBack,dwUser)!=0){
            RCLCPP_ERROR(this->get_logger(), "NET_DVR_SetStandardDataCallBack error: %d", NET_DVR_GetLastError());
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        PlayM4_Stop(lPort);

        // 关闭流，回收源数据缓冲
        PlayM4_CloseStream(lPort);

        // 释放播放库端口号
        PlayM4_FreePort(lPort);

        if (lRealPlayHandle_ >= 0)
        {
            NET_DVR_StopRealPlay(lRealPlayHandle_);
            lRealPlayHandle_ = -1;
        }
        g_pub->on_deactivate();
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
        g_pub.reset();
        g_clock.reset();
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
        g_pub.reset();
        g_clock.reset();
        return CallbackReturn::SUCCESS;
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