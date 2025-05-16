/*
 * Copyright(C) 2010,Hikvision Digital Technology Co., Ltd
 *
 * File   name��consoleMain.cpp
 * Discription��
 * Version    ��1.0
 * Author     ��panyadong
 * Create Date��2010_3_25
 * Modification History��
 */

#ifndef __APPLE__

#include <stdio.h>
#include <iostream>
#include "GetStream.h"
#include "public.h"
#include "ConfigParams.h"
#include "Alarm.h"
#include "CapPicture.h"
#include "playback.h"
#include "Voice.h"
#include "tool.h"
#include <string.h>
using namespace std;

/*
BOOL NET_DVR_STDXMLConfig(  LONG                         lUserID,
  NET_DVR_XML_CONFIG_INPUT     *lpInputParam,
  NET_DVR_XML_CONFIG_OUTPUT    *lpOutputParam);

typedef struct tagNET_DVR_MIME_UNIT
{
    //格式如下
    //Content-Disposition: form-data; name="upload"; filename="C:\Users\test\Desktop\11.txt"
    //Content-Type: text/plain
    char szContentType[32];               //对应Content-Type
    char szName[MAX_FILE_PATH_LEN];       //对应name字段
    char szFilename[MAX_FILE_PATH_LEN];   //对应filename字段
    DWORD dwContentLen;                   //Content的长度，大小限制512K以内
    char* pContent;                       //数据指针
    BYTE bySelfRead;                 // 0-外界传入文件 1-内部读取数据（通过szFilename传递完整路径），下发数据时该字段生效，获取数据时无效
    BYTE byRes[15];
}NET_DVR_MIME_UNIT, *LPNET_DVR_MIME_UNIT;
*/

static void NET_DVR_STDXMLConfig_test(int lUserID)
{
    NET_DVR_XML_CONFIG_INPUT struInputParam = {0};
    NET_DVR_XML_CONFIG_OUTPUT struOutputParam = {0};
    struInputParam.dwSize = sizeof(NET_DVR_XML_CONFIG_INPUT);
    struInputParam.lpRequestUrl = (void *)"GET /ISAPI/Thermal/channels/2/fireDetection";
    struInputParam.dwRequestUrlLen = strlen((char *)struInputParam.lpRequestUrl);
//     struInputParam.lpInBuffer =(void*) R"(<?xml version="1.0" encoding="UTF-8"?>
// <ThermIntell xmlns="http://www.isapi.org/ver20/XMLSchema" version="2.0"><id>2</id>
//   <intellType>thermometryAndFireDetectionAndAIOpenPlatForm</intellType>
// </ThermIntell>)";
//     struInputParam.dwInBufferSize = strlen((char *)struInputParam.lpInBuffer);
    struInputParam.lpInBuffer = NULL;
    struInputParam.dwInBufferSize = 0;

    char pOutBuffer[16 * 1024] = {0};
    char pStatusBuffer[1024] = {0};
    // NET_DVR_MIME_UNIT struMIMEUnit[16] = {0};
    // for(int i = 0; i < 16; i++)
    // {
    //     struMIMEUnit[i].dwContentLen = 1024;
    //     struMIMEUnit[i].pContent = ( char*) malloc(struMIMEUnit[i].dwContentLen);
    //     if(struMIMEUnit[i].pContent == NULL)
    //     {
    //         printf("malloc failed\n");
    //         return;
    //     }
    //     struMIMEUnit[i].bySelfRead = 1;
    //     struMIMEUnit[i].szName[0] = '\0';
    //     struMIMEUnit[i].szFilename[0] = '\0';
    // }
    struOutputParam.dwSize = sizeof(NET_DVR_XML_CONFIG_OUTPUT);
    struOutputParam.lpOutBuffer = pOutBuffer;
    struOutputParam.dwOutBufferSize = sizeof(pOutBuffer);
    struOutputParam.lpStatusBuffer = pStatusBuffer;
    struOutputParam.dwStatusSize = sizeof(pStatusBuffer);
    // struOutputParam.lpDataBuffer =struMIMEUnit;
    // struOutputParam.byNumOfMultiPart=16;
    if (NET_DVR_STDXMLConfig(lUserID, &struInputParam, &struOutputParam))
    {
        printf("NET_DVR_STDXMLConfig success\n");
        printf("Output buffer: %s\n", (char *)struOutputParam.lpOutBuffer);
        printf("Status buffer: %s\n", (char *)struOutputParam.lpStatusBuffer);
    }
    else
    {
        printf("NET_DVR_STDXMLConfig failed, %d\n", NET_DVR_GetLastError());
    }
}
// 时间解析宏定义
#define GET_YEAR(_time_) (((_time_) >> 26) + 2000)
#define GET_MONTH(_time_) (((_time_) >> 22) & 15)
#define GET_DAY(_time_) (((_time_) >> 17) & 31)
#define GET_HOUR(_time_) (((_time_) >> 12) & 31)
#define GET_MINUTE(_time_) (((_time_) >> 6) & 63)
#define GET_SECOND(_time_) (((_time_) >> 0) & 63)

void CALLBACK cbMessageCallback(LONG lCommand, NET_DVR_ALARMER *pAlarmer, char *pAlarmInfo, DWORD dwBufLen, void *pUser)
{
    printf("Alarm callback: lCommand: %d, dwBufLen: %d\n", lCommand, dwBufLen);
    int i;

    switch (lCommand)
    {
    case COMM_FIREDETECTION_ALARM: // 火点检测报警
    {
        NET_DVR_FIREDETECTION_ALARM struFireDetection = {0};
        memcpy(&struFireDetection, pAlarmInfo, sizeof(NET_DVR_FIREDETECTION_ALARM));
        printf("火点检测报警: RelativeTime:%d, AbsTime:%d, PTZ{PanPos:%d, TiltPos:%d, ZoomPos:%d}, \
        PicDataLen:%d, DevInfo{DevIP:%s, Port:%d, Channel:%d, IvmsChannel:%d}, \
        FireMaxTemperature:%d, TargetDistance:%d, fireRectInfo{fX:%f,fY:%f,fWidth%f,fHeight%f}, \
        fireMaxTemperaturePoint{fX:%f,fY:%f}\n",
               struFireDetection.dwRelativeTime,
               struFireDetection.dwAbsTime, struFireDetection.wPanPos, struFireDetection.wTiltPos,
               struFireDetection.wZoomPos, struFireDetection.dwPicDataLen,
               struFireDetection.struDevInfo.struDevIP.sIpV4, struFireDetection.struDevInfo.wPort,
               struFireDetection.struDevInfo.byChannel, struFireDetection.struDevInfo.byIvmsChannel,
               struFireDetection.wFireMaxTemperature, struFireDetection.wTargetDistance,
               struFireDetection.struRect.fX, struFireDetection.struRect.fY, struFireDetection.struRect.fWidth,
               struFireDetection.struRect.fHeight, struFireDetection.struPoint.fX, struFireDetection.struPoint.fY);

        NET_DVR_TIME struAbsTime = {0};
        struAbsTime.dwYear = GET_YEAR(struFireDetection.dwAbsTime);
        struAbsTime.dwMonth = GET_MONTH(struFireDetection.dwAbsTime);
        struAbsTime.dwDay = GET_DAY(struFireDetection.dwAbsTime);
        struAbsTime.dwHour = GET_HOUR(struFireDetection.dwAbsTime);
        struAbsTime.dwMinute = GET_MINUTE(struFireDetection.dwAbsTime);
        struAbsTime.dwSecond = GET_SECOND(struFireDetection.dwAbsTime);

        // 保存报警抓拍图片
        if (struFireDetection.dwPicDataLen > 0 && struFireDetection.pBuffer != NULL)
        {
            char cFilename[256] = {0};
            HANDLE hFile;
            DWORD dwReturn;

            char chTime[128];
            sprintf(chTime, "%4.4d%2.2d%2.2d%2.2d%2.2d%2.2d", struAbsTime.dwYear, struAbsTime.dwMonth, struAbsTime.dwDay, struAbsTime.dwHour, struAbsTime.dwMinute, struAbsTime.dwSecond);

            sprintf(cFilename, "FireDetectionPic[%s][%s].jpg", struFireDetection.struDevInfo.struDevIP.sIpV4, chTime);

            // hFile = CreateFile((LPCWSTR)cFilename, GENERIC_WRITE, FILE_SHARE_READ, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
            // if (hFile == INVALID_HANDLE_VALUE)
            // {
            //     break;
            // }
            // WriteFile(hFile, struFireDetection.pBuffer, struFireDetection.dwPicDataLen, &dwReturn, NULL);
            // CloseHandle(hFile);
            // hFile = INVALID_HANDLE_VALUE;
        }
    }
    break;
    case COMM_THERMOMETRY_ALARM: // 温度预警或者温度报警
    {
        NET_DVR_THERMOMETRY_ALARM struThermometryAlarm = {0};
        memcpy(&struThermometryAlarm, pAlarmInfo, sizeof(NET_DVR_THERMOMETRY_ALARM));
        if (0 == struThermometryAlarm.byRuleCalibType)
        {
            printf("点测温: Channel:%d, RuleID:%d, ThermometryUnit:%d, PresetNo:%d, RuleTemperature:%.1f, CurrTemperature:%.1f, PTZ Info[Pan:%f, Tilt:%f, Zoom:%f], AlarmLevel:%d, AlarmType:%d, AlarmRule:%d, RuleCalibType:%d,  Point[x:%f, y:%f], PicLen:%d, ThermalPicLen:%d, ThermalInfoLen:%d", struThermometryAlarm.dwChannel, struThermometryAlarm.byRuleID, struThermometryAlarm.byThermometryUnit, struThermometryAlarm.wPresetNo, struThermometryAlarm.fRuleTemperature, struThermometryAlarm.fCurrTemperature, struThermometryAlarm.struPtzInfo.fPan, struThermometryAlarm.struPtzInfo.fTilt, struThermometryAlarm.struPtzInfo.fZoom, struThermometryAlarm.byAlarmLevel, struThermometryAlarm.byAlarmType, struThermometryAlarm.byAlarmRule, struThermometryAlarm.byRuleCalibType, struThermometryAlarm.struPoint.fX, struThermometryAlarm.struPoint.fY, struThermometryAlarm.dwPicLen, struThermometryAlarm.dwThermalPicLen, struThermometryAlarm.dwThermalInfoLen);
        }
        else if (1 == struThermometryAlarm.byRuleCalibType || 2 == struThermometryAlarm.byRuleCalibType)
        {
            int iPointNum = struThermometryAlarm.struRegion.dwPointNum;
            // for (int i = 0; i < iPointNum; i++)
            // {
            //     float fX = struThermometryAlarm.struRegion.struPos[i].fX;
            //     float fY = struThermometryAlarm.struRegion.struPos[i].fY;
            //     printf("测温区域坐标点:X%d:%f,Y%d:%f;", i + 1, fX, i + 1, fY);
            // }
            printf("线测温或者框测温: Channel:%d, RuleID:%d,HighestPoint[x:%f, y:%f],ThermometryUnit:%d, PresetNo:%d,\
RuleTemperature:%.1f, CurrTemperature:%.1f, PTZ Info[Pan:%f, Tilt:%f, Zoom:%f], AlarmLevel:%d, AlarmType:%d, \
AlarmRule:%d, RuleCalibType:%d, PicLen:%d, ThermalPicLen:%d, ThermalInfoLen:%d",
                   struThermometryAlarm.dwChannel, struThermometryAlarm.byRuleID,
                   struThermometryAlarm.struHighestPoint.fX, struThermometryAlarm.struHighestPoint.fY,
                   struThermometryAlarm.byThermometryUnit, struThermometryAlarm.wPresetNo,
                   struThermometryAlarm.fRuleTemperature, struThermometryAlarm.fCurrTemperature,
                   struThermometryAlarm.struPtzInfo.fPan, struThermometryAlarm.struPtzInfo.fTilt, struThermometryAlarm.struPtzInfo.fZoom, struThermometryAlarm.byAlarmLevel, struThermometryAlarm.byAlarmType,
                   struThermometryAlarm.byAlarmRule, struThermometryAlarm.byRuleCalibType, struThermometryAlarm.dwPicLen, struThermometryAlarm.dwThermalPicLen, struThermometryAlarm.dwThermalInfoLen);
        }
        break;
    }

    case COMM_THERMOMETRY_DIFF_ALARM: // 温差报警
    {
        NET_DVR_THERMOMETRY_DIFF_ALARM struThermometryDiffAlarm = {0};
        memcpy(&struThermometryDiffAlarm, pAlarmInfo, sizeof(NET_DVR_THERMOMETRY_DIFF_ALARM));
        if (0 == struThermometryDiffAlarm.byRuleCalibType)
        {
            printf("温差报警: Channel:%d, AlarmID1:%d, AlarmID2:%d, PresetNo:%d, RuleTemperatureDiff:%.1f, CurTemperatureDiff:%.1f, AlarmLevel:%d, AlarmType:%d, AlarmRule:%d, RuleCalibType:%d, Point1[x:%f, y:%f], point2[x:%f, y:%f], PTZ Info[Pan:%f, Tilt:%f, Zoom:%f], PicLen:%d, ThermalPicLen:%d, ThermalInfoLen:%d, ThermometryUnit:%d", struThermometryDiffAlarm.dwChannel, struThermometryDiffAlarm.byAlarmID1, struThermometryDiffAlarm.byAlarmID2, struThermometryDiffAlarm.wPresetNo, struThermometryDiffAlarm.fRuleTemperatureDiff, struThermometryDiffAlarm.fCurTemperatureDiff, struThermometryDiffAlarm.byAlarmLevel, struThermometryDiffAlarm.byAlarmType, struThermometryDiffAlarm.byAlarmRule, struThermometryDiffAlarm.byRuleCalibType, struThermometryDiffAlarm.struPoint[0].fX, struThermometryDiffAlarm.struPoint[0].fY, struThermometryDiffAlarm.struPoint[1].fX, struThermometryDiffAlarm.struPoint[1].fY, struThermometryDiffAlarm.struPtzInfo.fPan, struThermometryDiffAlarm.struPtzInfo.fTilt, struThermometryDiffAlarm.struPtzInfo.fZoom, struThermometryDiffAlarm.dwPicLen, struThermometryDiffAlarm.dwThermalPicLen, struThermometryDiffAlarm.dwThermalInfoLen, struThermometryDiffAlarm.byThermometryUnit);
        }
        else if (1 == struThermometryDiffAlarm.byRuleCalibType || 2 == struThermometryDiffAlarm.byRuleCalibType)
        {
            int i = 0;
            int iPointNum = struThermometryDiffAlarm.struRegion[0].dwPointNum;
            for (i = 0; i < iPointNum; i++)
            {
                float fX = struThermometryDiffAlarm.struRegion[0].struPos[i].fX;
                float fY = struThermometryDiffAlarm.struRegion[0].struPos[i].fY;
                printf("测温区域1坐标点: X%d:%f,Y%d:%f;", iPointNum + 1, fX, iPointNum + 1, fY);
            }
            iPointNum = struThermometryDiffAlarm.struRegion[1].dwPointNum;
            for (i = 0; i < iPointNum; i++)
            {
                float fX = struThermometryDiffAlarm.struRegion[1].struPos[i].fX;
                float fY = struThermometryDiffAlarm.struRegion[1].struPos[i].fY;
                printf("测温区域2坐标点: X%d:%f,Y%d:%f;", iPointNum + 1, fX, iPointNum + 1, fY);
            }

            printf("温差报警: Channel:%d, AlarmID1:%d, AlarmID2:%d, PresetNo:%d, RuleTemperatureDiff:%.1f, CurTemperatureDiff:%.1f, AlarmLevel:%d, AlarmType:%d, AlarmRule:%d, RuleCalibType:%d,  PTZ Info[Pan:%f, Tilt:%f, Zoom:%f], PicLen:%d, ThermalPicLen:%d, ThermalInfoLen:%d, ThermometryUnit:%d", struThermometryDiffAlarm.dwChannel, struThermometryDiffAlarm.byAlarmID1, struThermometryDiffAlarm.byAlarmID2, struThermometryDiffAlarm.wPresetNo, struThermometryDiffAlarm.fRuleTemperatureDiff, struThermometryDiffAlarm.fCurTemperatureDiff, struThermometryDiffAlarm.byAlarmLevel, struThermometryDiffAlarm.byAlarmType, struThermometryDiffAlarm.byAlarmRule, struThermometryDiffAlarm.byRuleCalibType, struThermometryDiffAlarm.struPtzInfo.fPan, struThermometryDiffAlarm.struPtzInfo.fTilt, struThermometryDiffAlarm.struPtzInfo.fZoom, struThermometryDiffAlarm.dwPicLen, struThermometryDiffAlarm.dwThermalPicLen, struThermometryDiffAlarm.dwThermalInfoLen, struThermometryDiffAlarm.byThermometryUnit);
        }
        break;
    }

    default:
        printf("其他报警，报警信息类型: %d\n", lCommand);
        break;
    }

    return;
}


// typedef struct tagNET_DVR_THERMOMETRY_UPLOAD
// {
//     DWORD       dwSize;
//     DWORD       dwRelativeTime;     // 相对时标
//     DWORD       dwAbsTime;            // 绝对时标
//     char        szRuleName[NAME_LEN/*32*/];//规则名称
//     BYTE        byRuleID;//规则ID号
//     BYTE        byRuleCalibType;//规则标定类型 0-点，1-框，2-线
//     WORD        wPresetNo; //预置点号
//     NET_DVR_POINT_THERM_CFG struPointThermCfg;
//     NET_DVR_LINEPOLYGON_THERM_CFG struLinePolygonThermCfg;
//     BYTE        byThermometryUnit;//测温单位: 0-摄氏度（℃），1-华氏度（℉），2-开尔文(K)
//     BYTE        byDataType;//数据状态类型:0-检测中，1-开始，2-结束
//     BYTE		byRes1;
//     /*
//     bit0-中心点测温：0-不支持，1-支持；
//     bit1-最高点测温：0-不支持，1-支持；
//     bit2-最低点测温：0-不支持，1-支持；
//     */
//     BYTE      bySpecialPointThermType;// 是否支持特殊点测温
//     float	   fCenterPointTemperature;//中心点温度,精确到小数点后一位(-40-1500),（浮点数+100）*10 （由bySpecialPointThermType判断是否支持中心点）
//     float	   fHighestPointTemperature;//最高点温度,精确到小数点后一位(-40-1500),（浮点数+100）*10（由bySpecialPointThermType判断是否支持最高点）
//     float	   fLowestPointTemperature;//最低点温度,精确到小数点后一位(-40-1500),（浮点数+100）*10（由bySpecialPointThermType判断是否支持最低点）
//     NET_VCA_POINT struHighestPoint;//线、框测温最高温度位置坐标（当规则标定类型为线、框的时候生效）
//     NET_VCA_POINT struLowestPoint;//线、框测温最低温度位置坐标（当规则标定类型为线、框的时候生效）
// 	BYTE       byIsFreezedata;//是否数据冻结 0-否 1-是
//     BYTE       byFaceSnapThermometryEnabled;//人脸抓拍测温使能 1-开启 0-关闭
//     BYTE        byRes2[2];
//     DWORD       dwChan; //通道号，查询条件中通道号为0xffffffff时该字段生效
//     NET_VCA_RECT         struFaceRect;      //人脸子图区域
//     DWORD       dwTimestamp;//DSP时间戳
//     BYTE          byRes[68];
// }NET_DVR_THERMOMETRY_UPLOAD, *LPNET_DVR_THERMOMETRY_UPLOAD;

void CALLBACK GetThermInfoCallback(DWORD dwType, void* lpBuffer, DWORD dwBufLen, void* pUserData)
{
    printf("GetThermInfoCallback dwType[%d] dwBufLen[%d]\n", dwType, dwBufLen);
    if (dwType == NET_SDK_CALLBACK_TYPE_DATA)
    {
        NET_DVR_THERMOMETRY_UPLOAD struThermometry = { 0 };
        memcpy(&struThermometry, lpBuffer, sizeof(NET_DVR_THERMOMETRY_UPLOAD));

        NET_DVR_TIME struAbsTime = { 0 };
        struAbsTime.dwYear = GET_YEAR(struThermometry.dwAbsTime);
        struAbsTime.dwMonth = GET_MONTH(struThermometry.dwAbsTime);
        struAbsTime.dwDay = GET_DAY(struThermometry.dwAbsTime);
        struAbsTime.dwHour = GET_HOUR(struThermometry.dwAbsTime);
        struAbsTime.dwMinute = GET_MINUTE(struThermometry.dwAbsTime);
        struAbsTime.dwSecond = GET_SECOND(struThermometry.dwAbsTime);

        printf("实时测温结果:byRuleID[%d]wPresetNo[%d]byRuleCalibType[%d]byThermometryUnit[%d]byDataType[%d]dwAbsTime[%4.4d%2.2d%2.2d%2.2d%2.2d%2.2d]\n",
            struThermometry.byRuleID, struThermometry.wPresetNo, struThermometry.byRuleCalibType, struThermometry.byThermometryUnit,
            struThermometry.byDataType, struAbsTime.dwYear, struAbsTime.dwMonth, struAbsTime.dwDay, struAbsTime.dwHour, struAbsTime.dwMinute, struAbsTime.dwSecond);
        
        //点测温
        if (struThermometry.byRuleCalibType == 0)
        {
            printf("点测温信息:fTemperature[%f]\n", struThermometry.struPointThermCfg.fTemperature);
        }

        //框/线测温
        if ((struThermometry.byRuleCalibType == 1) || (struThermometry.byRuleCalibType == 2))
        {
            printf("框/线测温信息:fMaxTemperature[%f]fMinTemperature[%f]fAverageTemperature[%f]fTemperatureDiff[%f]\n",
                struThermometry.struLinePolygonThermCfg.fMaxTemperature, struThermometry.struLinePolygonThermCfg.fMinTemperature,
                struThermometry.struLinePolygonThermCfg.fAverageTemperature, struThermometry.struLinePolygonThermCfg.fTemperatureDiff);
        }
        LPNET_VCA_POLYGON pPolygon = &struThermometry.struLinePolygonThermCfg.struRegion;
        // for(int i=0; i<pPolygon->dwPointNum; i++)
        // {
        //     printf("框/线测温坐标点[%d]:X[%f] Y[%f]\n", i, pPolygon->struPos[i].fX, pPolygon->struPos[i].fY);
        // }
        printf("最高温度坐标点坐标: X[%f] Y[%f]\n", struThermometry.struHighestPoint.fX, struThermometry.struHighestPoint.fY);
        printf("最低温度坐标点坐标: X[%f] Y[%f]\n", struThermometry.struLowestPoint.fX, struThermometry.struLowestPoint.fY);
    }
    else if (dwType == NET_SDK_CALLBACK_TYPE_STATUS)
    {
        DWORD dwStatus = *(DWORD*)lpBuffer;
        if (dwStatus == NET_SDK_CALLBACK_STATUS_SUCCESS)
        {
            printf("dwStatus:NET_SDK_CALLBACK_STATUS_SUCCESS\n");
        }
        else if (dwStatus == NET_SDK_CALLBACK_STATUS_FAILED)
        {
            DWORD dwErrCode = *(DWORD*)((char *)lpBuffer + 4);
            printf("NET_DVR_GET_MANUALTHERM_INFO failed, Error code %d\n", dwErrCode);
        }
    }
}


int main()
{
    NET_DVR_Init();
    Demo_SDK_Version();
    NET_DVR_SetLogToFile(3, "./sdkLog");
    char cUserChoose = 'r';
    LONG lHandle = -1;

    // Login device
    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};
    struLoginInfo.bUseAsynLogin = false;

    struLoginInfo.wPort = 8000;
    memcpy(struLoginInfo.sDeviceAddress, "192.168.10.8", NET_DVR_DEV_ADDRESS_MAX_LEN);
    memcpy(struLoginInfo.sUserName, "admin", NAME_LEN);
    memcpy(struLoginInfo.sPassword, "ubuntu_define", NAME_LEN);

    int lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);

    if (lUserID < 0)
    {
        printf("pyd---Login error, %d\n", NET_DVR_GetLastError());
        printf("Press any key to quit...\n");
        cin >> cUserChoose;

        NET_DVR_Cleanup();
        return HPR_ERROR;
    }
#ifdef REALTIME_THERMOMETRY
     //启动实时温度检测
     NET_DVR_REALTIME_THERMOMETRY_COND struThermCond = { 0 };
     struThermCond.dwSize = sizeof(struThermCond);
     struThermCond.byRuleID = 0;       //规则ID，0代表获取全部规则，具体规则ID从1开始
     struThermCond.dwChan = 2; //从1开始，0xffffffff代表获取全部通道
     struThermCond.byMode=1;
    lHandle = NET_DVR_StartRemoteConfig(lUserID, NET_DVR_GET_REALTIME_THERMOMETRY, &struThermCond, sizeof(struThermCond), GetThermInfoCallback, NULL);
     if (lHandle < 0)
     {
         printf("NET_DVR_GET_REALTIME_THERMOMETRY failed, error code: %d\n", NET_DVR_GetLastError());
     }
     else
     {
         printf("NET_DVR_GET_REALTIME_THERMOMETRY is successful!");
     }
#endif
    //报警布防
    NET_DVR_SetDVRMessageCallBack_V50(0, cbMessageCallback, NULL);
    NET_DVR_SETUPALARM_PARAM struAlarmParam = {0};
    struAlarmParam.dwSize = sizeof(struAlarmParam);
    // 火点检测不需要设置其他报警布防参数，不支持

    lHandle = NET_DVR_SetupAlarmChan_V41(lUserID, &struAlarmParam);
    if (lHandle < 0)
    {
        printf("NET_DVR_SetupAlarmChan_V41 error, %d\n", NET_DVR_GetLastError());
        NET_DVR_Logout(lUserID);
        NET_DVR_Cleanup();
        return HPR_ERROR;
    }

    while ('q' != cUserChoose)
    {
        printf("\n");
        printf("Input 1, Test GetStream\n");
        printf("      2, Test Configure params\n");
        printf("      3, Test Alarm\n");
        printf("      4, Test Capture Picture\n");
        printf("      5, Test play back\n");
        printf("      6, Test Voice\n");
        printf("      7, Test SDK ability\n");
        printf("      8, Test tool interface\n");
        printf("      9, Test SDK XML\n");
        /*
        printf("      7, Test Matrix decode\n");
        printf("      8, Test PTZ\n");
        printf("      9, Test Format\n");
        printf("      0, Test Update\n");
        printf("      a, Test Serial trans\n");
        printf("      b, Test Configure Params\n");
        printf("      c, Test VCA && IVMS\n");
        */
        printf("      q, Quit.\n");
        printf("Input:");

        cin >> cUserChoose;
        switch (cUserChoose)
        {
        case '1':
            // Demo_GetStream_V30(lUserID); //Get stream.
            Demo_GetStream();
            break;
        case '2':
            Demo_SetIPCNet(lUserID);     // Setting params.
            Demo_IPParaCfg_V40(lUserID); // IP params.
            break;
        case '3':
            Demo_Alarm(); // Alarm & listen.
            break;
        case '4':
            Demo_Capture();
            break;
        case '5':
            Demo_PlayBack((int)lUserID); // record & playback
            break;
        case '6':
            Demo_Voice();
            break;
        case '7':
            Demo_SDK_Ability();
            break;
        case '8':
            Demo_DVRIPByResolveSvr();
            break;
        case '9':
            NET_DVR_STDXMLConfig_test(lUserID);
            break;
        default:
            break;
        }
    }

    // logout
    NET_DVR_Logout_V30(lUserID);
    NET_DVR_Cleanup();
    return 0;
}

#endif
