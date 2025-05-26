package alarm;

import NetSDKDemo.HCNetSDK;
import com.sun.jna.Pointer;


/**
 * @author jiangxin
 * @create 2022-08-15-17:26
 */
public class FMSGCallBack implements HCNetSDK.FMSGCallBack {
    //报警信息回调函数
    public void invoke(int lCommand, HCNetSDK.NET_DVR_ALARMER pAlarmer, Pointer pAlarmInfo, int dwBufLen, Pointer pUser) {
        AlarmDataParse.alarmDataHandle(lCommand, pAlarmer, pAlarmInfo, dwBufLen, pUser);
        return;
    }
}
