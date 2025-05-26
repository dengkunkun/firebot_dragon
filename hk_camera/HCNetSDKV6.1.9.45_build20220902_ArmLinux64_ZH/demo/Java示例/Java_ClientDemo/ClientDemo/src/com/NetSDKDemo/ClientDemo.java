package com.NetSDKDemo;

import Common.PropertiesUtil;
import Common.osSelect;
import com.sun.jna.Native;
import com.sun.jna.Pointer;

import java.io.IOException;
import java.util.Scanner;
import java.util.Timer;

/**
 * @create 2020-12-24-17:55
 */
public class ClientDemo {

    int iErr = 0;
    static HCNetSDK hCNetSDK = null;
    static int lUserID = -1;//用户句柄
    static int lDChannel;  //预览通道号
    static int lPlayHandle = -1;  //预览句柄
    static boolean bSaveHandle = false;
    Timer Playbacktimer;//回放用定时器
    static FExceptionCallBack_Imp fExceptionCallBack;
    static int FlowHandle;
    static PropertiesUtil propertiesUtil;
    static {
        try {
            propertiesUtil = new PropertiesUtil(System.getProperty("user.dir") + "/config.properties");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    static class FExceptionCallBack_Imp implements HCNetSDK.FExceptionCallBack {
        public void invoke(int dwType, int lUserID, int lHandle, Pointer pUser) {
            System.out.println("异常事件类型:"+dwType);
            return;
        }
    }
    /**
     * 动态库加载
     *
     * @return
     */
    private static boolean createSDKInstance() {
        if (hCNetSDK == null) {
            synchronized (HCNetSDK.class) {
                String strDllPath = "";
                try {
                    if (osSelect.isWindows())
                        //win系统加载库路径
                        strDllPath = System.getProperty("user.dir") + "\\lib\\HCNetSDK.dll";

                    else if (osSelect.isLinux())
                        //Linux系统加载库路径
                        strDllPath = System.getProperty("user.dir") + "/lib/libhcnetsdk.so";
                    hCNetSDK = (HCNetSDK) Native.loadLibrary(strDllPath, HCNetSDK.class);
                } catch (Exception ex) {
                    System.out.println("loadLibrary: " + strDllPath + " Error: " + ex.getMessage());
                    return false;
                }
            }
        }
        return true;
    }


    public static void main(String[] args) throws InterruptedException {

        if (hCNetSDK == null) {
            if (!createSDKInstance()) {
                System.out.println("Load SDK fail");
                return;
            }
        }
        //linux系统建议调用以下接口加载组件库
        if (osSelect.isLinux()) {
            HCNetSDK.BYTE_ARRAY ptrByteArray1 = new HCNetSDK.BYTE_ARRAY(256);
            HCNetSDK.BYTE_ARRAY ptrByteArray2 = new HCNetSDK.BYTE_ARRAY(256);
            //这里是库的绝对路径，请根据实际情况修改，注意改路径必须有访问权限
            String strPath1 = System.getProperty("user.dir") + "/lib/libcrypto.so.1.1";
            String strPath2 = System.getProperty("user.dir") + "/lib/libssl.so.1.1";
            System.arraycopy(strPath1.getBytes(), 0, ptrByteArray1.byValue, 0, strPath1.length());
            ptrByteArray1.write();
            hCNetSDK.NET_DVR_SetSDKInitCfg(HCNetSDK.NET_SDK_INIT_CFG_LIBEAY_PATH, ptrByteArray1.getPointer());
            System.arraycopy(strPath2.getBytes(), 0, ptrByteArray2.byValue, 0, strPath2.length());
            ptrByteArray2.write();
            hCNetSDK.NET_DVR_SetSDKInitCfg(HCNetSDK.NET_SDK_INIT_CFG_SSLEAY_PATH, ptrByteArray2.getPointer());
            String strPathCom = System.getProperty("user.dir") + "/lib/";
            HCNetSDK.NET_DVR_LOCAL_SDK_PATH struComPath = new HCNetSDK.NET_DVR_LOCAL_SDK_PATH();
            System.arraycopy(strPathCom.getBytes(), 0, struComPath.sPath, 0, strPathCom.length());
            struComPath.write();
            hCNetSDK.NET_DVR_SetSDKInitCfg(HCNetSDK.NET_SDK_INIT_CFG_SDK_PATH, struComPath.getPointer());
        }
        //SDK初始化，一个程序只需要调用一次
        boolean initSuc = hCNetSDK.NET_DVR_Init();
        //异常消息回调
        if(fExceptionCallBack == null)
        {
            fExceptionCallBack = new FExceptionCallBack_Imp();
        }
        Pointer pUser = null;
        if (!hCNetSDK.NET_DVR_SetExceptionCallBack_V30(0, 0, fExceptionCallBack, pUser)) {
            return ;
        }
        System.out.println("设置异常消息回调成功");
        //启动SDK写日志
        hCNetSDK.NET_DVR_SetLogToFile(3, "./sdkLog", false);
        
        //设备登录
        lUserID=loginDevice(propertiesUtil.readValue("ip"), (short) 8000, propertiesUtil.readValue("name"), propertiesUtil.readValue("pass"));

        for (boolean exit = false; !exit; ) {
            System.out.println("请输入您想要执行的demo实例! （退出请输入yes）");
            Scanner input = new Scanner(System.in);
            String str = input.next();
            // 转换为标准输入
            str = str.toLowerCase();
            if (str.equals("yes")) {
                // 退出程序
                exit = true;
                break;
            }
            switch (str) {
                case "1":
                {
                    System.out.println("\n[Module]实时取流示例代码");
                    lPlayHandle=VideoDemo.getRealStreamData(lUserID,1);
                    /**
                     * 实时取流开启成功后，调用播放库抓图，延时几秒保证取流进入回调进行解码
                     */
//                    Thread.sleep(3000);
                    break;
                }
                case "2":
                {
                    System.out.println("\n[Module]停止实时取流示例代码");
                    VideoDemo.stopRealStreamData(lPlayHandle);
                    break;
                }
                case "3":
                {
                    System.out.println("\n[Module]实时获取裸码流示例代码");
                    lPlayHandle=VideoDemo.getESRealStreamData(lUserID,1);
                    break;
                }
                case "4":
                {
                    System.out.println("\n[Module]按时间回放示例代码");
                    new VideoDemo().playBackBytime(lUserID,33);
                    break;
                }
                case "5":
                {
                    System.out.println("\n[Module]按文件回放录像示例代码");
                    VideoDemo.playBackByfile(lUserID,33);
                    break;
                }
                case "6":
                {
                    System.out.println("\n[Module]按时间下载录像示例代码");
                    new VideoDemo().dowmloadRecordByTime(lUserID,33);
                    break;
                }
                case "7":
                {
                    System.out.println("\n[Module]按文件下载录像示例代码");
                    new VideoDemo().downloadRecordByFile(lUserID,33);
                    break;
                }
                case "8":
                {
                    //开启录像提前开启预览
                    System.out.println("\n[Module]开启录像示例代码");
                    VideoDemo.startSaveRealData(lPlayHandle);
                    break;
                }

                case "9":
                {
                    System.out.println("\n[Module]关闭录像示例代码");
                    VideoDemo.stopSaveRealData(lPlayHandle);
                    break;
                }
                default:
                {
                    System.out.println("\n未知的指令操作!请重新输入!\n");
                }
            }
        }

        //退出程序时调用，每一台设备分别注销
        if (hCNetSDK.NET_DVR_Logout(lUserID)) {
            System.out.println("注销成功");
        }

        //SDK反初始化，释放资源，只需要退出时调用一次
        hCNetSDK.NET_DVR_Cleanup();
        return;
    }

    /**
     * 登录设备，支持 V40 和 V30 版本，功能一致。
     *
     * @param ip      设备IP地址
     * @param port    SDK端口，默认为设备的8000端口
     * @param user    设备用户名
     * @param psw     设备密码
     * @return 登录成功返回用户ID，失败返回-1
     */
    public static int loginDevice(String ip, short port, String user, String psw) {
        // 创建设备登录信息和设备信息对象
        HCNetSDK.NET_DVR_USER_LOGIN_INFO loginInfo = new HCNetSDK.NET_DVR_USER_LOGIN_INFO();
        HCNetSDK.NET_DVR_DEVICEINFO_V40 deviceInfo = new HCNetSDK.NET_DVR_DEVICEINFO_V40();

        // 设置设备IP地址
        byte[] deviceAddress = new byte[HCNetSDK.NET_DVR_DEV_ADDRESS_MAX_LEN];
        byte[] ipBytes = ip.getBytes();
        System.arraycopy(ipBytes, 0, deviceAddress, 0, Math.min(ipBytes.length, deviceAddress.length));
        loginInfo.sDeviceAddress = deviceAddress;

        // 设置用户名和密码
        byte[] userName = new byte[HCNetSDK.NET_DVR_LOGIN_USERNAME_MAX_LEN];
        byte[] password = psw.getBytes();
        System.arraycopy(user.getBytes(), 0, userName, 0, Math.min(user.length(), userName.length));
        System.arraycopy(password, 0, loginInfo.sPassword, 0, Math.min(password.length, loginInfo.sPassword.length));
        loginInfo.sUserName = userName;

        // 设置端口和登录模式
        loginInfo.wPort = port;
        loginInfo.bUseAsynLogin = false; // 同步登录
        loginInfo.byLoginMode = 0; // 使用SDK私有协议

        // 执行登录操作
        int userID = hCNetSDK.NET_DVR_Login_V40(loginInfo, deviceInfo);
        if (userID == -1) {
            System.err.println("登录失败，错误码为: " + hCNetSDK.NET_DVR_GetLastError());
        } else {
            System.out.println(ip + " 设备登录成功！");
            // 处理通道号逻辑
            int startDChan = deviceInfo.struDeviceV30.byStartDChan;
            System.out.println("预览起始通道号: " + startDChan);
        }
        return userID; // 返回登录结果
    }

}



