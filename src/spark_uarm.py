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
        self.UDP_IP = "192.168.1.73"
        self.UDP_PORT = 9091
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind((self.UDP_IP, self.UDP_PORT))
        self.client_addresses = set() 
        self.udp_thread = threading.Thread(target=self.udp_server_thread, daemon=True)
        self.udp_thread.start()

    def udp_server_thread(self):
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
        if self.client_addresses:
            position_data = json.dumps(self.spark_uarm_pose)
            for client_addr in list(self.client_addresses):
                try:
                    self.server_socket.sendto(position_data.encode(), client_addr)
                except Exception as e:
                    self.client_addresses.discard(client_addr)

    def sparkUarmCallback(self, data):
        self.spark_uarm_pose['x'], self.spark_uarm_pose['y'] = y_to_z(data)
        x = data.pose.orientation.x
        y = data.pose.orientation.y
        z = data.pose.orientation.z
        w = data.pose.orientation.w
        _, _, yaw = euler_from_quaternion_y_up(x, y, z, w)
        self.spark_uarm_pose['theta'] = yaw
        print("[Spark UArm] x:{:.2f} y:{:.2f} theta:{:.2f}".format(
            self.spark_uarm_pose['x'], 
            self.spark_uarm_pose['y'], 
            self.spark_uarm_pose['theta']
        ))
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
