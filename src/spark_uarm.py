#!/usr/bin/env python2
# -*- encoding: UTF-8 -*-
import rospy
import geometry_msgs.msg
from utils import y_to_z, euler_from_quaternion_y_up
import signal
import sys
import socket
import threading
import time
import json

class SparkUarm():
    def __init__(self):
        rospy.init_node("spark_uarm_node", anonymous=False)
        rospy.Subscriber('/vrpn_client_node/spark_uarm/pose', geometry_msgs.msg.PoseStamped, self.sparkUarmCallback)
        self.rate = rospy.Rate(10)  # 10hz
        
        self.spark_uarm_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # UDP服务端设置
        self.UDP_IP = "192.168.1.73"
        self.UDP_PORT = 9091
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind((self.UDP_IP, self.UDP_PORT))
        self.client_addresses = set()  # 存储连接的客户端地址
        
        # 启动UDP服务线程
        self.udp_thread = threading.Thread(target=self.udp_server_thread, daemon=True)
        self.udp_thread.start()
        
        print(f"Spark UArm UDP服务器启动: {self.UDP_IP}:{self.UDP_PORT}")

    def udp_server_thread(self):
        """UDP服务器线程，接收客户端连接"""
        while True:
            try:
                data, client_address = self.server_socket.recvfrom(1024)
                if client_address not in self.client_addresses:
                    self.client_addresses.add(client_address)
                    print(f"新客户端连接: {client_address}")
            except Exception as e:
                print(f"UDP服务器错误: {e}")
                break

    def broadcast_position(self):
        """向所有连接的客户端广播位置信息"""
        if self.client_addresses:
            position_data = json.dumps(self.spark_uarm_pose)
            for client_addr in list(self.client_addresses):
                try:
                    self.server_socket.sendto(position_data.encode(), client_addr)
                except Exception as e:
                    print(f"发送位置数据失败 {client_addr}: {e}")
                    self.client_addresses.discard(client_addr)

    def sparkUarmCallback(self, data):
        # 获取位置信息
        self.spark_uarm_pose['x'], self.spark_uarm_pose['y'] = y_to_z(data)
        
        # 获取方向信息
        x = data.pose.orientation.x
        y = data.pose.orientation.y
        z = data.pose.orientation.z
        w = data.pose.orientation.w
        _, _, yaw = euler_from_quaternion_y_up(x, y, z, w)
        self.spark_uarm_pose['theta'] = yaw
        
        # 打印当前位置
        print("[Spark UArm] x:{:.2f} y:{:.2f} theta:{:.2f}".format(
            self.spark_uarm_pose['x'], 
            self.spark_uarm_pose['y'], 
            self.spark_uarm_pose['theta']
        ))
        
        # 广播位置信息
        self.broadcast_position()

    def signal_handler(self, sig, frame):
        print("Signal detected: ", sig, ". Stopping...")
        self.server_socket.close()
        sys.exit(0)

    def run(self):
        print("Spark UArm position monitor started...")
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    try:
        spark_uarm = SparkUarm()
        spark_uarm.run()
    except rospy.ROSInterruptException:
        print("Program interrupted")
