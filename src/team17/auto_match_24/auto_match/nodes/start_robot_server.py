#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Control Server Launcher
This script is used to launch the robot control server in the ROS Python 3.8 environment.
"""
import os
import sys
import argparse
import rospy

# Add the parent directory to the Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

def main():
    """Main function to parse arguments and start the server"""
    # 初始化 ROS 节点
    rospy.init_node('robot_control_server_launcher', anonymous=True)
    
    # 从命令行获取参数
    parser = argparse.ArgumentParser(description='Start the Robot Control Server')
    parser.add_argument('--host', type=str, default=None,
                        help='Host address to bind the server (default: from ROS param or localhost)')
    parser.add_argument('--port', type=int, default=None,
                        help='Port to bind the server (default: from ROS param or 9999)')
    args = parser.parse_args()
    
    # 从 ROS 参数服务器获取参数，如果命令行有指定则使用命令行参数覆盖
    host = args.host if args.host is not None else rospy.get_param('~host', 'localhost')
    port = args.port if args.port is not None else rospy.get_param('~port', 9999)
    
    try:
        # 导入服务器类
        from robot_control_server import RobotControlServer
        
        # 创建并启动服务器
        server = RobotControlServer(host=host, port=port)
        rospy.loginfo(f"Starting Robot Control Server on {host}:{port}")
        print(f"Starting Robot Control Server on {host}:{port}")
        server.start_server()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException received, shutting down...")
        print("ROSInterruptException received, shutting down...")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received, shutting down...")
        print("Keyboard interrupt received, shutting down...")
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")
        print(f"Error: {str(e)}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 