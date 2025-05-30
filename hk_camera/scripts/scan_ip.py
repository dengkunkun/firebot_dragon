#!/usr/bin/env python3
from zeroconf import Zeroconf, ServiceBrowser, ServiceListener
import re
import socket
import time

# 匹配以 HM-TD2B28T-3- 开头的主机名
CAMERA_PREFIX = "HM-TD2B28T-3-"

class CameraListener(ServiceListener):
    def __init__(self):
        self.found = {}

    def remove_service(self, zeroconf, type, name):
        pass  # 可选：处理设备下线

    def add_service(self, zeroconf, type, name):
        info = zeroconf.get_service_info(type, name)
        if info and info.server.startswith(CAMERA_PREFIX):
            addresses = [socket.inet_ntoa(addr) for addr in info.addresses]
            print(f"发现海康相机: {info.server}  IP: {addresses}")
            self.found[info.server] = addresses

    def update_service(self, zeroconf, type, name):
        pass  # 可选：处理设备信息更新

def main():
    zeroconf = Zeroconf()
    listener = CameraListener()
    # "_http._tcp.local." 是常见的服务类型，实际可根据相机支持的服务类型调整
    browser = ServiceBrowser(zeroconf, "_http._tcp.local.", listener)
    print("正在扫描局域网中的海康相机（10秒后自动结束）...")
    time.sleep(10)
    zeroconf.close()
    print("\n扫描结果：")
    for name, ips in listener.found.items():
        print(f"{name} -> {ips}")

if __name__ == "__main__":
    main()