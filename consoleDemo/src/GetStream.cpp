/*
 * Copyright(C) 2010,Hikvision Digital Technology Co., Ltd
 *
 * File   name��GetStream.cpp
 * Discription��
 * Version    ��1.0
 * Author     ��panyd
 * Create Date��2010_3_25
 * Modification History��
 */

#ifdef _WIN32
#include <windows.h>
#elif defined(__linux__) || defined(__APPLE__)
#include <unistd.h>
#endif

#include "HCNetSDK.h"
#include "public.h"
#include <stdio.h>
#include <time.h>
#include <string.h>

// void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer,DWORD dwBufSize,void* dwUser)
// {
//     printf("pyd---(private_v30)Get data,the size is %d,%d.\n", time(NULL), dwBufSize);
// }

void CALLBACK g_HikDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, DWORD dwUser)
{
    printf("pyd---(private)Get data,the size is %d.\n", dwBufSize);
}

void CALLBACK g_StdDataCallBack(int lRealHandle, unsigned int dwDataType, unsigned char *pBuffer, unsigned int dwBufSize, unsigned int dwUser)
{
    printf("pyd---(rtsp)Get data,the size is %d.\n", dwBufSize);
}

#include "LinuxPlayM4.h"
void CALLBACK DecCBFun(int nPort, char *pBuf, int nSize,
                       FRAME_INFO *pFrameInfo,
                       void* nReserved1, int /*nReserved2*/)
{

    //---------------------------------------
    // 获取解码后音频数据
    if (pFrameInfo->nType == T_AUDIO16)
    {
        printf("test:: get audio data !\n");
    }
    //---------------------------------------
    // 获取解码后视频数据
    else if (pFrameInfo->nType == T_YV12)
    {
        printf("%lu:: get video data nWidth:%d nHeight:%d  nStamp:%d nFrameRate:%d dwFrameNum:%u\n",
            time(NULL),pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->nStamp, pFrameInfo->nFrameRate, pFrameInfo->dwFrameNum);
    }
}

void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *dwUser)
{
    // HWND hWnd=GetConsoleWindowAPI();
    static int lPort = -1; // 播放库port号
    switch (dwDataType)
    {
    case NET_DVR_SYSHEAD: // 系统头
        if (lPort >= 0)
        {
            break; // 该通道取流之前已经获取到句柄，后续接口不需要再调用
        }

        if (!PlayM4_GetPort(&lPort)) // 获取播放库未使用的通道号
        {
            break;
        }
        // m_iPort = lPort; //第一次回调的是系统头，将获取的播放库port号赋值给全局port，下次回调数据时即使用此port号播放
        if (dwBufSize > 0)
        {
            if (!PlayM4_SetStreamOpenMode(lPort, STREAME_REALTIME)) // 设置实时流播放模式
            {
                break;
            }

            if (!PlayM4_OpenStream(lPort, pBuffer, dwBufSize, 1024 * 1024)) // 打开流接口
            {
                break;
            }
            PlayM4_SetDecCallBackExMend(lPort, DecCBFun, NULL, 0, NULL);
            // if (!PlayM4_Play(lPort, hWnd)) //播放开始
            if (!PlayM4_Play(lPort, 0))
            {
                break;
            }
        }
        break;
    case NET_DVR_STREAMDATA: // 码流数据
        if (dwBufSize > 0 && lPort != -1)
        {
            if (!PlayM4_InputData(lPort, pBuffer, dwBufSize))
            {
                break;
            }
        }
        break;
    default: // 其他数据
        if (dwBufSize > 0 && lPort != -1)
        {
            if (!PlayM4_InputData(lPort, pBuffer, dwBufSize))
            {
                break;
            }
        }
        break;
    }
}

/*******************************************************************
      Function:   Demo_GetStream
   Description:   preview(no "_V30")
     Parameter:   (IN)   none
        Return:   0--successful��-1--fail��
**********************************************************************/
int Demo_GetStream()
{

    NET_DVR_Init();
    long lUserID;
    // login
    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};
    struLoginInfo.bUseAsynLogin = false;

    struLoginInfo.wPort = 8000;
    memcpy(struLoginInfo.sDeviceAddress, "192.168.10.8", NET_DVR_DEV_ADDRESS_MAX_LEN);
    memcpy(struLoginInfo.sUserName, "admin", NAME_LEN);
    memcpy(struLoginInfo.sPassword, "ubuntu_define", NAME_LEN);
    lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);

    if (lUserID < 0)
    {
        printf("pyd1---Login error, %d\n", NET_DVR_GetLastError());
        return HPR_ERROR;
    }

    // Set callback function of getting stream.
    long lRealPlayHandle;
    NET_DVR_PREVIEWINFO struPlayInfo = {0};
#if (defined(_WIN32) || defined(_WIN_WCE))
    struPlayInfo.hPlayWnd = NULL;
#elif defined(__linux__)
    struPlayInfo.hPlayWnd = 0;
#endif
    struPlayInfo.lChannel = 1; // channel NO
    struPlayInfo.dwLinkMode = 0;
    struPlayInfo.bBlocked = 1;
    struPlayInfo.dwDisplayBufNum = 1;
    lRealPlayHandle = NET_DVR_RealPlay_V40(lUserID, &struPlayInfo, g_RealDataCallBack_V30, NULL);

    if (lRealPlayHandle < 0)
    {
        printf("pyd1---NET_DVR_RealPlay_V40 error\n");
        NET_DVR_Logout(lUserID);
        NET_DVR_Cleanup();
        return HPR_ERROR;
    }

    // Set callback function of getting stream.
    // int iRet;
    // iRet = NET_DVR_SetRealDataCallBack(lRealPlayHandle, g_HikDataCallBack, 0);
    // if (!iRet)
    // {
    //     printf("pyd1---NET_DVR_RealPlay_V40 error\n");
    //     NET_DVR_StopRealPlay(lRealPlayHandle);
    //     NET_DVR_Logout_V30(lUserID);
    //     NET_DVR_Cleanup();
    //     return HPR_ERROR;
    // }

#ifdef _WIN32
    Sleep(5000); // millisecond
#elif defined(__linux__)
    sleep(500); // second
#endif

    // stop
    NET_DVR_StopRealPlay(lRealPlayHandle);
    NET_DVR_Logout_V30(lUserID);
    NET_DVR_Cleanup();
    return HPR_OK;
}

void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
    char tempbuf[256] = {0};
    switch (dwType)
    {
    case EXCEPTION_RECONNECT: // Ԥ��ʱ����
        printf("pyd----------reconnect--------%d\n", time(NULL));
        break;
    default:
        break;
    }
};

/*******************************************************************
      Function:   Demo_GetStream_V30
   Description:   preview(_V30)
     Parameter:   (IN)   none
        Return:   0--successful��-1--fail��
**********************************************************************/
int Demo_GetStream_V30(LONG lUserID)
{
    // Set callback function of getting stream.
    long lRealPlayHandle;
    NET_DVR_PREVIEWINFO struPlayInfo = {0};
#if (defined(_WIN32) || defined(_WIN_WCE))
    struPlayInfo.hPlayWnd = NULL;
#elif defined(__linux__)
    struPlayInfo.hPlayWnd = 0;
#endif
    struPlayInfo.lChannel = 1; // channel NO
    struPlayInfo.dwLinkMode = 0;
    struPlayInfo.bBlocked = 1;
    struPlayInfo.dwDisplayBufNum = 1;
    lRealPlayHandle = NET_DVR_RealPlay_V40(lUserID, &struPlayInfo, NULL, NULL);
    // lRealPlayHandle = NET_DVR_RealPlay_V30(lUserID, &ClientInfo, NULL, NULL, 0);
    if (lRealPlayHandle < 0)
    {
        printf("pyd---NET_DVR_RealPlay_V40  error, %d\n", NET_DVR_GetLastError());

        return HPR_ERROR;
    }

    // Set rtsp callback function of getting stream.
    // NET_DVR_SetStandardDataCallBack(lRealPlayHandle, g_StdDataCallBack, 0);

#ifdef _WIN32
    Sleep(3000); // millisecond
#elif defined(__linux__) || defined(__APPLE__)
    sleep(5); // second
#endif

    // stop
    // NET_DVR_StopRealPlay(lRealPlayHandle);
    // NET_DVR_Logout(lUserID);
    // NET_DVR_Logout_V30(lUserID);
    // cleanup
    NET_DVR_Cleanup();

    return HPR_OK;
}

void Demo_SDK_Ability()
{
    NET_DVR_Init();
    NET_DVR_SDKABL struSDKABL = {0};
    if (NET_DVR_GetSDKAbility(&struSDKABL))
    {
        printf("SDK Max: %d\n", struSDKABL.dwMaxRealPlayNum);
        NET_DVR_Cleanup();
        return;
    }

    NET_DVR_Cleanup();
    return;
}

void Demo_SDK_Version()
{
    unsigned int uiVersion = NET_DVR_GetSDKBuildVersion();

    char strTemp[1024] = {0};
    sprintf(strTemp, "HCNetSDK V%d.%d.%d.%d\n",
            (0xff000000 & uiVersion) >> 24,
            (0x00ff0000 & uiVersion) >> 16,
            (0x0000ff00 & uiVersion) >> 8,
            (0x000000ff & uiVersion));
    printf(strTemp);
}
