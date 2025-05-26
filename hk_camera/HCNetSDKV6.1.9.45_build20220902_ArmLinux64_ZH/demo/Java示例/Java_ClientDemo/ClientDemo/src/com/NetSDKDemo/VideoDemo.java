package com.NetSDKDemo;

import com.sun.jna.Pointer;
import com.sun.jna.ptr.ByteByReference;
import com.sun.jna.ptr.IntByReference;
import com.sun.scenario.effect.impl.sw.sse.SSEBlend_SRC_OUTPeer;

import java.io.*;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Timer;

import static com.NetSDKDemo.ClientDemo.hCNetSDK;

/**
 * 视频取流预览，下载，抓图mok
 * @create 2022-03-30-9:48
 */
public class VideoDemo {
    Timer downloadtimer;//下载用定时器
    Timer Playbacktimer;//回放用定时器
    static FRealDataCallBack fRealDataCallBack;//预览回调函数实现
    static fPlayEScallback fPlayescallback; //裸码流回调函数
    static playDataCallBack playBackCallBack; //回放码流回调
    static fStdDataCallBack fStdDataCallback; //标准码流回调
    static int Count = 0;
    int m_lLoadHandle;
    int iPlayBack; //回放句柄
    static File file;
    static File esRealDatafile;
    static boolean palybackFlay = false;
    static FileOutputStream outputStream;
    static IntByReference m_lPort = new IntByReference(-1);
    static FileOutputStream EstremDataoutputStream = null;
    static int fileLenth = 0;

    /**
     * @param userID     设备登录句柄
     * @param iChannelNo 通道号
     * @return 取流预览句柄
     */
    public static int getRealStreamData(int userID, int iChannelNo) {
        if (userID == -1) {
            System.out.println("请先注册");
            return -1;
        }
        HCNetSDK.NET_DVR_PREVIEWINFO previewInfo = new HCNetSDK.NET_DVR_PREVIEWINFO();
        previewInfo.read();
//        previewInfo.hPlayWnd = 0;  //窗口句柄，从回调取流不显示一般设置为空
        previewInfo.lChannel = iChannelNo;  //通道号
        previewInfo.dwStreamType = 0; //0-主码流，1-子码流，2-三码流，3-虚拟码流，以此类推
        previewInfo.dwLinkMode = 0; //连接方式：0- TCP方式，1- UDP方式，2- 多播方式，3- RTP方式，4- RTP/RTSP，5- RTP/HTTP，6- HRUDP（可靠传输） ，7- RTSP/HTTPS，8- NPQ
        previewInfo.bBlocked = 1;  //0- 非阻塞取流，1- 阻塞取流
        previewInfo.byProtoType = 0; //应用层取流协议：0- 私有协议，1- RTSP协议
        previewInfo.write();
        //回调函数定义必须是全局的
        if (fRealDataCallBack == null) {
            fRealDataCallBack = new FRealDataCallBack();
        }
        //开启预览
        int Handle = hCNetSDK.NET_DVR_RealPlay_V40(userID, previewInfo, fRealDataCallBack, null);
        if (Handle == -1) {
            int iErr = hCNetSDK.NET_DVR_GetLastError();
            System.err.println("取流失败" + iErr);
            return -1;
        }
        System.out.println("取流成功");
        return Handle;
    }

    /**
     * 停止取流
     *
     * @param PlayHandle 预览句柄
     */
    public static void stopRealStreamData(int PlayHandle) {
        if (PlayHandle == -1) {
            System.out.println("实时取流未开启，请先开启实时取流");
            return;
        }
        if (!hCNetSDK.NET_DVR_StopRealPlay(PlayHandle)) {
            System.err.println("停止取流失败，err:" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }
        System.out.println("停止取流成功");
        return;
    }


    /**
     * 获取实时裸码流回调数据
     *
     * @param userID     登录句柄
     * @param iChannelNo 通道号参数
     */
    public static int getESRealStreamData(int userID, int iChannelNo) {
        if (userID == -1) {
            System.out.println("请先注册");
            return -1;
        }
        String resultFileName = System.getProperty("user.dir") + "\\Download\\ESRealStreamData.data";
        esRealDatafile =new File(resultFileName);
        if (!esRealDatafile.exists()) {
            try {
                esRealDatafile.createNewFile();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        try {
            EstremDataoutputStream  = new FileOutputStream(esRealDatafile);
        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        HCNetSDK.NET_DVR_PREVIEWINFO previewInfo = new HCNetSDK.NET_DVR_PREVIEWINFO();
        previewInfo.read();
//        previewInfo.hPlayWnd = null;  //窗口句柄，从回调取流不显示一般设置为空
        previewInfo.lChannel = iChannelNo;  //通道号
        previewInfo.dwStreamType = 0; //0-主码流，1-子码流，2-三码流，3-虚拟码流，以此类推
        previewInfo.dwLinkMode = 0; //连接方式：0- TCP方式，1- UDP方式，2- 多播方式，3- RTP方式，4- RTP/RTSP，5- RTP/HTTP，6- HRUDP（可靠传输） ，7- RTSP/HTTPS，8- NPQ
        previewInfo.bBlocked = 1;  //0- 非阻塞取流，1- 阻塞取流
        previewInfo.byProtoType = 0; //应用层取流协议：0- 私有协议，1- RTSP协议
        previewInfo.write();
        //开启预览
        int Handle = hCNetSDK.NET_DVR_RealPlay_V40(userID, previewInfo, null, null);
        if (Handle == -1) {
            int iErr = hCNetSDK.NET_DVR_GetLastError();
            System.err.println("取流失败" + iErr);
            return -1;
        }
        System.out.println("取流成功");

        //设置裸码流回调函数
        if (fPlayescallback == null) {
            fPlayescallback = new fPlayEScallback();
        }
        if (!hCNetSDK.NET_DVR_SetESRealPlayCallBack(Handle, fPlayescallback, null)) {
            System.err.println("设置裸码流回调失败，错误码：" + hCNetSDK.NET_DVR_GetLastError());
        }


        //设置RTP回调函数
//        if (fStdDataCallback==null)
//        {
//            fStdDataCallback=new fStdDataCallBack();
//        }
//        boolean setRtpcallback=hCNetSDK.NET_DVR_SetStandardDataCallBack(lPlay,fStdDataCallback,0);
//        if (setRtpcallback==false)
//        {
//            System.out.println("设置RTP回调失败，错误码："+hCNetSDK.NET_DVR_GetLastError());
//        }


        /**
         * 预览一段时间；如果要一直取流预览，需要保证程序一直运行
         */
/*        try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}


        String path = ".\\savevideo.mp4";
        Boolean bSaveVideo = hCNetSDK.NET_DVR_SaveRealData_V30(lPlay,0x2,path);
        if (bSaveVideo == false) {
            int iErr = hCNetSDK.NET_DVR_GetLastError();
            System.out.println("NET_DVR_SaveRealData_V30 failed" + iErr);
            return;
        }
        System.out.println("NET_DVR_SaveRealData_V30 suss");

        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        Boolean bStopSaveVideo = hCNetSDK.NET_DVR_StopSaveRealData(lPlay);
        if (bStopSaveVideo == false) {
            int iErr = hCNetSDK.NET_DVR_GetLastError();
            System.out.println("NET_DVR_StopSaveRealData failed" + iErr);
            return;
        }
            System.out.println("NET_DVR_StopSaveRealData suss");


        if (lPlay>=0) {
            if (hCNetSDK.NET_DVR_StopRealPlay(lPlay))
            {
                System.out.println("停止预览成功");
                return;
            }
        }*/
        return Handle;
    }

    /**
     * 开始录像，开启录像需要先开启实时预览
     * @param PlayHandle
     */
    public static void startSaveRealData(int PlayHandle)
    {
        if (PlayHandle == -1) {
            System.out.println("实时取流未开启，请先开启实时取流");
            return;
        }
        String path =  System.getProperty("user.dir") + "\\Download\\SaveRealData.mp4";
        byte[] byPath = new byte[0];
        try {
            byPath = path.getBytes("GBK");
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }
        HCNetSDK.BYTE_ARRAY byArray = new HCNetSDK.BYTE_ARRAY(byPath.length);
        System.arraycopy(byPath, 0, byArray.byValue, 0, byPath.length);
        byArray.write();
        Boolean bSaveVideo = hCNetSDK.NET_DVR_SaveRealData_V30(PlayHandle,0x2,byArray.getPointer());
        if (bSaveVideo == false) {
            int iErr = hCNetSDK.NET_DVR_GetLastError();
            System.err.println("NET_DVR_SaveRealData_V30 failed" + iErr);
            return;
        }
        System.out.println("NET_DVR_SaveRealData_V30 suss");
    }

    /**
     * 停止录像
     * @param PlayHandle
     */
    public static void stopSaveRealData(int PlayHandle)
    {
        if (PlayHandle == -1) {
            System.out.println("实时取流未开启，请先开启实时取流");
            return;
        }
        if ( !hCNetSDK.NET_DVR_StopSaveRealData(PlayHandle)) {
            int iErr = hCNetSDK.NET_DVR_GetLastError();
            System.out.println("NET_DVR_StopSaveRealData failed" + iErr);
            return;
        }
        System.out.println("NET_DVR_StopSaveRealData suss");

    }


    //播放库抓图
    /*public static void getPicbyPlayCtrl(){

        //取流成功后，延时一段时间保证播放库解码开始
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        IntByReference pWidth = new IntByReference(0);
        IntByReference pHieght = new IntByReference(0);
        boolean bFlag = playControl.PlayM4_GetPictureSize(m_lPort.getValue(), pWidth, pHieght);
        if (!bFlag) {
            System.out.println("获取失败：" + playControl.PlayM4_GetLastError(m_lPort.getValue()));
        }
        System.out.println(pWidth.getValue());
        System.out.println(pHieght.getValue());
        IntByReference RealPicSize = new IntByReference(0);
        int picsize = pWidth.getValue() * pHieght.getValue() * 5;
        HCNetSDK.BYTE_ARRAY picByte = new HCNetSDK.BYTE_ARRAY(picsize);
        picByte.write();
        Pointer pByte = picByte.getPointer();
        boolean b_GetPic = playControl.PlayM4_GetJPEG(m_lPort.getValue(), pByte, picsize, RealPicSize);
        if (!b_GetPic) {
            System.out.println("抓图失败：" + playControl.PlayM4_GetLastError(m_lPort.getValue()));
        }
        picByte.read();
        SimpleDateFormat sf = new SimpleDateFormat("yyyyMMddHHmmss");
        String newName = sf.format(new Date());
        FileOutputStream fout = null;
        try {

            fout = new FileOutputStream(System.getProperty("user.dir") + "//pic//" + newName + ".jpg");
            //将字节写入文件
            long offset = 0;
            ByteBuffer buffers = pByte.getByteBuffer(offset, RealPicSize.getValue());
            byte[] bytes = new byte[RealPicSize.getValue()];
            buffers.rewind();
            buffers.get(bytes);
            fout.write(bytes);
            fout.close();
        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        System.out.println("抓图成功!");

    }*/

    /**
     * 按时间回放录像
     * @param userID 设备登录句柄
     * @param lChannel 通道号
     */
    public void playBackBytime(int userID,int lChannel)
    {
        file = new File(System.getProperty("user.dir") + "\\Download\\Videodatabytime.mp4");  //保存回调函数的码流数据
        if (!file.exists()) {
            try {
                file.createNewFile();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        try {
            outputStream = new FileOutputStream(file);
        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        HCNetSDK.NET_DVR_VOD_PARA net_dvr_vod_para = new HCNetSDK.NET_DVR_VOD_PARA();
        net_dvr_vod_para.dwSize=net_dvr_vod_para.size();
        net_dvr_vod_para.struIDInfo.dwChannel=lChannel; //通道号
        //开始时间
        net_dvr_vod_para.struBeginTime.dwYear=2024;
        net_dvr_vod_para.struBeginTime.dwMonth=8;
        net_dvr_vod_para.struBeginTime.dwDay=5;
        net_dvr_vod_para.struBeginTime.dwHour=10;
        net_dvr_vod_para.struBeginTime.dwMinute=00;
        net_dvr_vod_para.struBeginTime.dwSecond=00;
        //停止时间
        net_dvr_vod_para.struEndTime.dwYear=2024;
        net_dvr_vod_para.struEndTime.dwMonth=8;
        net_dvr_vod_para.struEndTime.dwDay=5;
        net_dvr_vod_para.struEndTime.dwHour=10;
        net_dvr_vod_para.struEndTime.dwMinute=05;
        net_dvr_vod_para.struEndTime.dwSecond=00;
        net_dvr_vod_para.hWnd=null; // 回放的窗口句柄，若置为空，SDK仍能收到码流数据，但不解码显示
        net_dvr_vod_para.write();
        int iPlayBack=hCNetSDK.NET_DVR_PlayBackByTime_V40(userID,net_dvr_vod_para);
        if (iPlayBack<=-1)
        {
            System.out.println("按时间回放失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            palybackFlay = true;
            return;
        }
        //开启取流
       IntByReference intP=new IntByReference(54962*1024);
        IntByReference intInlen1=new IntByReference(0);
        boolean bCrtl=hCNetSDK.NET_DVR_PlayBackControl_V40(iPlayBack, HCNetSDK.NET_DVR_PLAYSTART, Pointer.NULL, 0, Pointer.NULL,intInlen1);
        if (bCrtl==false)
        {
            System.out.println("NET_DVR_PLAYSTART失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }
        //切换倒放
//        IntByReference intInlen2=new IntByReference(0);
//        boolean bCrtl2=hCNetSDK.NET_DVR_PlayBackControl_V40(iPlayBack, HCNetSDK.NET_DVR_PLAY_REVERSE, Pointer.NULL, 0, Pointer.NULL,intInlen2);
//        if (bCrtl2==false)
//        {
//            System.out.println("NET_DVR_PLAY_REVERSE失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
//            return;
//        }

        if (playBackCallBack==null)
        {
            playBackCallBack = new playDataCallBack();
        }
        boolean bRet=hCNetSDK.NET_DVR_SetPlayDataCallBack_V40(iPlayBack,playBackCallBack,Pointer.NULL);
        //开始计时器
        Playbacktimer = new Timer();//新建定时器
        Playbacktimer.schedule(new PlaybackTask(), 0, 5000);//0秒后开始响应函数
        return;
    }


    /**
     * 按文件回放录像
     * @param userID 用户ID
     * @param lChannel 通道号
     */
    public static void playBackByfile(int userID,int lChannel)
    {
        file = new File(System.getProperty("user.dir") + "\\Download\\Videodatabyfile.mp4");  //保存回调函数的码流数据

        if (!file.exists()) {
            try {
                file.createNewFile();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        try {
            outputStream = new FileOutputStream(file);
        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        String strFileName="";
        HCNetSDK.NET_DVR_FILECOND_V40 struFileCond = new HCNetSDK.NET_DVR_FILECOND_V40();
        struFileCond.read();
        struFileCond.lChannel= lChannel; //通道号 NVR设备路数小于32路的起始通道号从33开始，依次增加
        struFileCond.byFindType=0;  //录象文件类型 0=定时录像
        //起始时间
        struFileCond.struStartTime.dwYear=2023;
        struFileCond.struStartTime.dwMonth=9;
        struFileCond.struStartTime.dwDay=8;
        struFileCond.struStartTime.dwHour=8;
        struFileCond.struStartTime.dwMinute=00;
        struFileCond.struStartTime.dwSecond=00;
        //停止时间
        struFileCond.struStopTime.dwYear=2023;
        struFileCond.struStopTime.dwMonth=9;
        struFileCond.struStopTime.dwDay=8;
        struFileCond.struStopTime.dwHour=10;
        struFileCond.struStopTime.dwMinute=30;
        struFileCond.struStopTime.dwSecond=00;
        struFileCond.write();
        int  FindFileHandle=hCNetSDK.NET_DVR_FindFile_V40(userID,struFileCond);
        if (FindFileHandle<=-1)
        {
            System.out.println("查找建立失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
        }

        while (true)
        {
            HCNetSDK.NET_DVR_FINDDATA_V40 struFindData = new HCNetSDK.NET_DVR_FINDDATA_V40();

            long State=hCNetSDK.NET_DVR_FindNextFile_V40(FindFileHandle,struFindData);
            if (State<=-1)
            {

                System.out.println("查找失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
                return;

            }
            else if (State==1000)  //获取文件信息成功
            {
                struFindData.read();
                try {
                    strFileName=new String(struFindData.sFileName,"UTF-8").trim();
                    System.out.println("文件名称："+strFileName);
                    System.out.println("文件大小:"+struFindData.dwFileSize);
                } catch (UnsupportedEncodingException e) {
                    e.printStackTrace();
                }
                System.out.println("获取文件成功");
                break;
            }
            else if (State==1001) //未查找到文件
            {
                System.out.println("未查找到文件");
                break;
            }
            else if (State==1002) //正在查找请等待
            {
                System.out.println("正在查找，请等待");
                continue;

            }

            else if (State==1003) //没有更多的文件，查找结束
            {
                System.out.println("没有更多的文件，查找结束");
                break;

            }
            else if (State==1004) //查找文件时异常
            {

                System.out.println("没有更多的文件，查找结束");
                break;

            }
            else if (State==1005) //查找文件超时
            {

                System.out.println("没有更多的文件，查找结束");
                break;

            }

        }
        boolean b_CloseHandle=hCNetSDK.NET_DVR_FindClose_V30(FindFileHandle);
        if (!b_CloseHandle) {
            System.out.println("关闭失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }

        int lPlayByFileHandle=hCNetSDK.NET_DVR_PlayBackByName(userID,strFileName,null);
        if (lPlayByFileHandle<=-1)
        {
            System.out.println("按文件回放失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }
        IntByReference intP1=new IntByReference(0);
        IntByReference intInlen=new IntByReference(0);
        boolean b_PlayBackStart=hCNetSDK.NET_DVR_PlayBackControl_V40(lPlayByFileHandle,HCNetSDK.NET_DVR_PLAYSTART,intP1.getPointer(),4,Pointer.NULL,intInlen);
        if (!b_PlayBackStart) {
            System.out.println("开始播放失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }
        if (playBackCallBack==null)
        {
            playBackCallBack=new playDataCallBack();
        }
        boolean bRet=hCNetSDK.NET_DVR_SetPlayDataCallBack_V40(lPlayByFileHandle,playBackCallBack,Pointer.NULL);
        while (true)
        {
            int Pos=hCNetSDK.NET_DVR_GetDownloadPos(lPlayByFileHandle);
            if (Pos!=100)
            {
                System.out.println("回放进度:"+Pos);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
                continue;
            }
            else {
                break;
            }
        }
        boolean b_Stop=hCNetSDK.NET_DVR_StopPlayBack(lPlayByFileHandle);
        if (!b_Stop) {
            System.out.println("停止回放失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }
        System.out.println("回放成功");
        return;
    }





    //按文件下载录像(设置转成3GP格式)
    public  void downloadRecordByFile(int userID, int iChannelNo)
    {
        String strFileName="";
        HCNetSDK.NET_DVR_FILECOND_V40 struFileCond = new HCNetSDK.NET_DVR_FILECOND_V40();
        struFileCond.read();
        struFileCond.lChannel= iChannelNo; //通道号 NVR设备路数小于32路的起始通道号从33开始，依次增加
        struFileCond.dwFileType = 0Xff;
        struFileCond.byFindType  = 0;
        //起始时间
        struFileCond.struStartTime.dwYear=2022;
        struFileCond.struStartTime.dwMonth=11;
        struFileCond.struStartTime.dwDay=19;
        struFileCond.struStartTime.dwHour=12;
        struFileCond.struStartTime.dwMinute=00;
        struFileCond.struStartTime.dwSecond=00;
        //停止时间
        struFileCond.struStopTime.dwYear=2022;
        struFileCond.struStopTime.dwMonth=11;
        struFileCond.struStopTime.dwDay=19;
        struFileCond.struStopTime.dwHour=15;
        struFileCond.struStopTime.dwMinute=00;
        struFileCond.struStopTime.dwSecond=00;
        struFileCond.write();
        int  FindFileHandle=hCNetSDK.NET_DVR_FindFile_V40(userID,struFileCond);
        if (FindFileHandle<=-1)
        {
            System.out.println("查找建立失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
        }

        while (true)
        {
            HCNetSDK.NET_DVR_FINDDATA_V40 struFindData = new HCNetSDK.NET_DVR_FINDDATA_V40();

            long State=hCNetSDK.NET_DVR_FindNextFile_V40(FindFileHandle,struFindData);
            if (State<=-1)
            {

                System.out.println("查找失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
                return;

            }

            else if (State==1000)  //获取文件信息成功
            {
                struFindData.read();
                try {
                    strFileName=new String(struFindData.sFileName,"UTF-8").trim();
                    System.out.println("文件名称："+strFileName);
                    System.out.println("文件大小:"+struFindData.dwFileSize);
                } catch (UnsupportedEncodingException e) {
                    e.printStackTrace();
                }
                System.out.println("获取文件成功");
                break;

            }
            else if (State==1001) //未查找到文件
            {
                System.out.println("未查找到文件");
                break;

            }
            else if (State==1002) //正在查找请等待
            {
                System.out.println("正在查找，请等待");
                continue;

            }

            else if (State==1003) //没有更多的文件，查找结束
            {
                System.out.println("没有更多的文件，查找结束");
                break;

            }
            else if (State==1004) //查找文件时异常
            {

                System.out.println("没有更多的文件，查找结束");
                break;

            }
            else if (State==1005) //查找文件超时
            {

                System.out.println("没有更多的文件，查找结束");
                break;

            }

        }
        boolean b_CloseHandle=hCNetSDK.NET_DVR_FindClose_V30(FindFileHandle);
        if (!b_CloseHandle) {
            System.out.println("关闭失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }

        //按文件下载录像
        String SaveDir=".\\Download\\test.mp4";

        int FileName=hCNetSDK.NET_DVR_GetFileByName(userID,strFileName,SaveDir.getBytes());
        if (FileName<=-1)
        {
            System.out.println("下载录像失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }
        //转码3GP命令
        IntByReference intP=new IntByReference(5);
        IntByReference intInlen1=new IntByReference(0);
        boolean b_PlayBack=hCNetSDK.NET_DVR_PlayBackControl_V40(FileName,32,intP.getPointer(),4,Pointer.NULL,intInlen1);
        if (!b_PlayBack) {
            System.out.println("转封装失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }

        IntByReference intP1=new IntByReference(0);
        IntByReference intInlen=new IntByReference(0);
        boolean b_PlayBackStart=hCNetSDK.NET_DVR_PlayBackControl_V40(FileName,HCNetSDK.NET_DVR_PLAYSTART,intP1.getPointer(),4,Pointer.NULL,intInlen);
        if (!b_PlayBackStart) {
            System.out.println("开始播放失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }

/*        IntByReference intP=new IntByReference(30*1024);
        IntByReference intInlen1=new IntByReference(0);
        boolean b_PlayBack=hCNetSDK.NET_DVR_PlayBackControl_V40(FileName,24,intP.getPointer(),4, Pointer.NULL,intInlen1);
        if (!b_PlayBack) {
            System.out.println("设置下载速度失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }*/
        while (true)
        {
            int Pos=hCNetSDK.NET_DVR_GetDownloadPos(FileName);
            if (Pos!=100)
            {
                System.out.println("下载进度:"+Pos);
                
                try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} 
                continue;
            }
            else {
                break;
            }
        }
        boolean b_Stop=hCNetSDK.NET_DVR_StopGetFile(FileName);
        if (!b_Stop) {
            System.out.println("停止下载失败，错误码为" + hCNetSDK.NET_DVR_GetLastError());
            return;
        }
        System.out.println("下载成功");
        return;
    }

    //按时间下载录像(不支持转码3GP格式)
    public  void dowmloadRecordByTime(int userID,int iChannelNo)
    {
        HCNetSDK.NET_DVR_PLAYCOND net_dvr_playcond = new HCNetSDK.NET_DVR_PLAYCOND();
        net_dvr_playcond.read();
        net_dvr_playcond.dwChannel=iChannelNo; //通道号 NVR设备路数小于32路的起始通道号从33开始，依次增加
        //开始时间
        net_dvr_playcond.struStartTime.dwYear=2023;
        net_dvr_playcond.struStartTime.dwMonth=9;
        net_dvr_playcond.struStartTime.dwDay=14;
        net_dvr_playcond.struStartTime.dwHour=12;
        net_dvr_playcond.struStartTime.dwMinute=45;
        net_dvr_playcond.struStartTime.dwSecond=11;
        //停止时间
        net_dvr_playcond.struStopTime.dwYear=2023;
        net_dvr_playcond.struStopTime.dwMonth=9;
        net_dvr_playcond.struStopTime.dwDay=14;
        net_dvr_playcond.struStopTime.dwHour=12;
        net_dvr_playcond.struStopTime.dwMinute=45;
        net_dvr_playcond.struStopTime.dwSecond=19;
        net_dvr_playcond.write();
        String sFileName = ".\\Download\\" + System.currentTimeMillis() + ".mp4";
        System.out.println(sFileName);
        m_lLoadHandle = hCNetSDK.NET_DVR_GetFileByTime_V40(userID,sFileName,net_dvr_playcond);
        if (m_lLoadHandle >= 0)
        {
            hCNetSDK.NET_DVR_PlayBackControl(m_lLoadHandle, HCNetSDK.NET_DVR_PLAYSTART, 0, null);
/*            IntByReference intP=new IntByReference(5*8*1024);
            IntByReference intInlen=new IntByReference(0);
            boolean b_PlayBack=ClientDemo.hCNetSDK.NET_DVR_PlayBackControl_V40(m_lLoadHandle,24,intP.getPointer(),4,Pointer.NULL,intInlen);
            if (!b_PlayBack) {
                System.out.println("设置下载速度失败，错误码为" + ClientDemo.hCNetSDK.NET_DVR_GetLastError());
                return;
            }*/
            Date nowTime = new Date(System.currentTimeMillis());
            SimpleDateFormat sdFormatter = new SimpleDateFormat("yyyy-MM-dd-hh-mm-ss");
            System.out.println("开始下载时间："+sdFormatter.format(nowTime));
            downloadtimer = new Timer();//新建定时器
            downloadtimer.schedule(new downloadTask(), 0, 5000);//0秒后开始响应函数
        } else
        {
            System.out.println("按时间下载失败");
            System.out.println("laste error " + hCNetSDK.NET_DVR_GetLastError());
            return;
        }
    }

    /*************************************************
     类:      DownloadTask
     类描述:  下载定时器响应函数
     *************************************************/
     class downloadTask extends java.util.TimerTask
    {
        //定时器函数
        @Override
        public void run()
        {
            IntByReference nPos = new IntByReference(0);
           hCNetSDK.NET_DVR_PlayBackControl(m_lLoadHandle, HCNetSDK.NET_DVR_PLAYGETPOS, 0, nPos);
            if (nPos.getValue() > 100)
            {
                m_lLoadHandle=-1;
                downloadtimer.cancel();
                System.out.println( "由于网络原因或DVR忙,下载异常终止!");
            }
            if (nPos.getValue() == 100)
            {
                Date nowTime = new Date(System.currentTimeMillis());
                SimpleDateFormat sdFormatter = new SimpleDateFormat("yyyy-MM-dd-hh-mm-ss");
                System.out.println("结束下载时间："+sdFormatter.format(nowTime));
                m_lLoadHandle=-1;
                System.out.println("按时间下载结束!");
                // 任务完成后，取消定时器
                // 取消任务，防止再次执行
                this.cancel();
                // 取消定时器
                if (downloadtimer != null) {
                    downloadtimer.cancel();
                }
            }
        }
    }

    /**
     * 按时间回放进度获取定时器
     */
    class PlaybackTask extends java.util.TimerTask
    {
        //定时器函数
        @Override
        public void run()
        {
            System.out.println("定时器触发");
            IntByReference nPos = new IntByReference(0);

                System.out.println("iPlayBack "+iPlayBack);
                boolean bret=hCNetSDK.NET_DVR_PlayBackControl(iPlayBack, HCNetSDK.NET_DVR_PLAYGETPOS, 0, nPos);
                if (bret)
                {
                    System.out.println("回放进度" + nPos.getValue());
                }
                else
                {
                    System.out.println("获取回放进度失败");
                }

                if (nPos.getValue() > 100)
                {

                    hCNetSDK.NET_DVR_StopPlayBack(iPlayBack);
                    if (outputStream != null) {
                        try {
                            outputStream.close();
                        } catch (IOException e) {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                        }
                    }
                    palybackFlay = true;
                    System.out.println("由于网络原因或DVR忙,回放异常终止!");
                    return;
                }
                if (nPos.getValue() == 100)
                {
                    hCNetSDK.NET_DVR_StopPlayBack(iPlayBack);
                    if (outputStream != null) {
                        try {
                            outputStream.close();
                        } catch (IOException e) {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                        }
                    }
                    palybackFlay = true;
                    System.out.println("按时间回放结束");
                    // 任务完成后，取消定时器
                    // 取消任务，防止再次执行
                    this.cancel();
                    // 取消定时器
                    if (Playbacktimer != null) {
                            Playbacktimer.cancel();
                        }
                    }
                    return;

                }
            }

    //获取IP通道
    public static void getIPChannelInfo(int userID) {
        IntByReference ibrBytesReturned = new IntByReference(0);//获取IP接入配置参数
        HCNetSDK.NET_DVR_IPPARACFG_V40 m_strIpparaCfg = new HCNetSDK.NET_DVR_IPPARACFG_V40();
        m_strIpparaCfg.write();
        //lpIpParaConfig 接收数据的缓冲指针
        Pointer lpIpParaConfig = m_strIpparaCfg.getPointer();
        boolean bRet = hCNetSDK.NET_DVR_GetDVRConfig(userID, HCNetSDK.NET_DVR_GET_IPPARACFG_V40, 0, lpIpParaConfig, m_strIpparaCfg.size(), ibrBytesReturned);
        m_strIpparaCfg.read();
        System.out.println("起始数字通道号：" + m_strIpparaCfg.dwStartDChan);

        for (int iChannum = 0; iChannum < m_strIpparaCfg.dwDChanNum; iChannum++) {
            int channum = iChannum + m_strIpparaCfg.dwStartDChan;
            m_strIpparaCfg.struStreamMode[iChannum].read();
            if (m_strIpparaCfg.struStreamMode[iChannum].byGetStreamType == 0) {
                m_strIpparaCfg.struStreamMode[iChannum].uGetStream.setType(HCNetSDK.NET_DVR_IPCHANINFO.class);
                m_strIpparaCfg.struStreamMode[iChannum].uGetStream.struChanInfo.read();
                if (m_strIpparaCfg.struStreamMode[iChannum].uGetStream.struChanInfo.byEnable == 1) {
                    System.out.println("IP通道" + channum + "在线");
                } else {
                    System.out.println("IP通道" + channum + "不在线");
                }
            }
        }
    }

    static class fPlayEScallback implements  HCNetSDK.FPlayESCallBack {
        public void invoke(int lPreviewHandle, HCNetSDK.NET_DVR_PACKET_INFO_EX pstruPackInfo, Pointer pUser) {
            pstruPackInfo.read();
            //将设备发送过来的回放码流数据写入文件
            //保存I帧和P帧数据
            if (pstruPackInfo.dwPacketType == 1 || pstruPackInfo.dwPacketType == 3)
            {
                System.out.println("dwPacketType：" + pstruPackInfo.dwPacketType + "：wWidth:" + pstruPackInfo.wWidth + "：wHeight:" + pstruPackInfo.wHeight +
                        "：包长度：" + pstruPackInfo.dwPacketSize + "：帧号：" + pstruPackInfo.dwFrameNum);
                long offset = 0;
                ByteBuffer buffers = pstruPackInfo.pPacketBuffer.getByteBuffer(offset, pstruPackInfo.dwPacketSize);
                byte[] bytes = new byte[pstruPackInfo.dwPacketSize];
                buffers.rewind();
                buffers.get(bytes);
                try {
                    EstremDataoutputStream.write(bytes);
                } catch (IOException e) {
                    e.printStackTrace();
                }
                return;
            }

        }
    }

    static  class playDataCallBack implements HCNetSDK.FPlayDataCallBack{
        public void invoke(int lPlayHandle, int dwDataType, Pointer pBuffer, int dwBufSize, int dwUser)
        {
            System.out.println("回放码流回调...");
            //将设备发送过来的回放码流数据写入文件
            long offset = 0;
            ByteBuffer buffers = pBuffer.getByteBuffer(offset, dwBufSize);
            byte[] bytes = new byte[dwBufSize];
            buffers.rewind();
            buffers.get(bytes);
            try {
                outputStream.write(bytes);
            } catch (IOException e) {
                e.printStackTrace();
            }
            //播放库解码
/*            switch (dwDataType) {
                case HCNetSDK.NET_DVR_SYSHEAD: //系统头
                    if (!playControl.PlayM4_GetPort(m_lPort)) //获取播放库未使用的通道号
                    {
                        break;
                    }
                    if (dwBufSize > 0) {
                        if (!playControl.PlayM4_SetStreamOpenMode(m_lPort.getValue(), PlayCtrl.STREAME_REALTIME))  //设置实时流播放模式
                        {
                            break;
                        }
                        if (!playControl.PlayM4_OpenStream(m_lPort.getValue(), pBuffer, dwBufSize, 1024 * 1024)) //打开流接口
                        {
                            break;
                        }
                        if (!playControl.PlayM4_Play(m_lPort.getValue(), null)) //播放开始
                        {
                            break;
                        }

                    }
                case HCNetSDK.NET_DVR_STREAMDATA:   //码流数据
                    if ((dwBufSize > 0) && (m_lPort.getValue() != -1)) {
                        if (!playControl.PlayM4_InputData(m_lPort.getValue(), pBuffer, dwBufSize))  //输入流数据
                        {
                            break;
                        }
                    }
            }*/

        }

    }

    static  class fStdDataCallBack implements HCNetSDK.FStdDataCallBack{
        public void invoke(int lPlayHandle, int dwDataType, Pointer pBuffer, int dwBufSize, int dwUser)
        {
            System.out.println("RTP码流回调...");
        }

    }

    static class FRealDataCallBack implements HCNetSDK.FRealDataCallBack_V30 {
        //预览回调
        public void invoke(int lRealHandle, int dwDataType, Pointer pBuffer, int dwBufSize, Pointer pUser) {
            if (Count == 100) {//降低打印频率
                System.out.println("码流数据回调...dwBufSize="+dwBufSize);
                Count = 0;
            }
            Count++;
            //本Demo演示码流数据送人播放库解码流程,如果仅需保存实时码流数据需要自行实现，
            /*
            switch (dwDataType) {
                case HCNetSDK.NET_DVR_SYSHEAD: //系统头
                    if (!playControl.PlayM4_GetPort(m_lPort)) //获取播放库未使用的通道号
                    {
                        break;
                    }
                    if (dwBufSize > 0) {
                        if (!playControl.PlayM4_SetStreamOpenMode(m_lPort.getValue(), PlayCtrl.STREAME_REALTIME))  //设置实时流播放模式
                        {
                            break;
                        }
                        if (!playControl.PlayM4_OpenStream(m_lPort.getValue(), pBuffer, dwBufSize, 1024 * 1024)) //打开流接口
                        {
                            break;
                        }
                        if (!playControl.PlayM4_Play(m_lPort.getValue(), null)) //播放开始
                        {
                            break;
                        }

                    }
                case HCNetSDK.NET_DVR_STREAMDATA:   //码流数据
                    if ((dwBufSize > 0) && (m_lPort.getValue() != -1)) {
                        if (!playControl.PlayM4_InputData(m_lPort.getValue(), pBuffer, dwBufSize))  //输入流数据
                        {
                            break;
                        }
                    }
            }*/
        }
    }














}
