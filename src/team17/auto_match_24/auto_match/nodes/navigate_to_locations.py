#!/usr/bin/env python3

import rospy
import argparse
import sys
import os
from std_msgs.msg import String

# 尝试不同的导入方式来确保能找到 RobotNavigator 类
try:
    # 设置脚本所在目录到 Python 路径中
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(script_dir)
    from robot_navigation import RobotNavigator
except ImportError as e:
    rospy.logerr(f"导入错误: {e}")
    rospy.logerr("尝试备用导入方式...")
    try:
        # 添加完整的路径尝试
        sys.path.append(os.path.join(os.environ['HOME'], 'spark_noetic_team17', 'Robot_manipulation'))
        from robot_navigation import RobotNavigator
    except ImportError as e2:
        rospy.logerr(f"备用导入方式也失败: {e2}")
        rospy.logerr("注意: 请确保 robot_navigation.py 文件位于正确的路径下")
        # 创建一个简单版本的RobotNavigator以便能继续运行
        class RobotNavigator:
            def __init__(self):
                """简化版本的RobotNavigator"""
                rospy.logwarn("使用备用的RobotNavigator类，功能受限")
                self.goto_local_pub = rospy.Publisher("mark_nav", String, queue_size=1)
            
            def goto_local(self, name):
                """简化版本的导航功能"""
                self.goto_local_pub.publish(f"go {name}")
                return True

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='机器人导航测试工具')
    parser.add_argument('--location', '-l', type=str, help='导航目标位置名称')
    parser.add_argument('--interactive', '-i', action='store_true', help='交互模式')
    parser.add_argument('--cycle', '-c', action='store_true', help='循环模式，在两个位置之间循环导航')
    return parser.parse_args()

def main():
    """
    主程序：测试机器人导航功能
    """
    # 初始化节点
    rospy.init_node('location_navigator', anonymous=True)
    rospy.loginfo("导航节点初始化...")
    
    try:
        # 解析命令行参数
        args = parse_arguments()
        
        # 创建导航器实例
        rospy.loginfo("尝试创建RobotNavigator实例...")
        navigator = RobotNavigator()
        
        # 创建导航命令发布者
        nav_cmd_pub = rospy.Publisher("navigation_command", String, queue_size=1)
        
        # 预定义的位置
        location_1 = "sorting_area"  # 整理区
        location_2 = "placement_area"  # 放置区
        
        # 等待Publisher连接
        rospy.sleep(1.0)
        
        if args.interactive:
            # 交互模式
            rospy.loginfo("=== 交互式导航模式 ===")
            rospy.loginfo("可用命令:")
            rospy.loginfo("  1: 导航到整理区 (sorting_area)")
            rospy.loginfo("  2: 导航到放置区 (placement_area)")
            rospy.loginfo("  q: 退出")
            
            while not rospy.is_shutdown():
                try:
                    cmd = input("\n请输入命令(1/2/q): ").strip()
                    if cmd == '1':
                        rospy.loginfo(f"导航到 {location_1}")
                        nav_cmd_pub.publish(f"goto {location_1}")
                    elif cmd == '2':
                        rospy.loginfo(f"导航到 {location_2}")
                        nav_cmd_pub.publish(f"goto {location_2}")
                    elif cmd.lower() == 'q':
                        rospy.loginfo("退出导航程序")
                        break
                    else:
                        rospy.logwarn(f"未知命令: {cmd}")
                except Exception as e:
                    rospy.logerr(f"输入错误: {e}")
                
        elif args.cycle:
            # 循环模式：在两个位置之间循环导航
            rospy.loginfo("=== 循环导航模式 ===")
            rospy.loginfo(f"将在 {location_1} 和 {location_2} 之间循环导航")
            rospy.loginfo("按 Ctrl+C 停止程序")
            
            while not rospy.is_shutdown():
                # 导航到第一个位置
                rospy.loginfo(f"正在导航到位置: {location_1}")
                nav_cmd_pub.publish(f"goto {location_1}")
                rospy.sleep(65.0)  # 等待导航完成的大致时间
                
                if rospy.is_shutdown():
                    break
                    
                # 导航到第二个位置
                rospy.loginfo(f"正在导航到位置: {location_2}")
                nav_cmd_pub.publish(f"goto {location_2}")
                rospy.sleep(65.0)  # 等待导航完成的大致时间
                
        elif args.location:
            # 单次导航到指定位置
            rospy.loginfo(f"单次导航到位置: {args.location}")
            nav_cmd_pub.publish(f"goto {args.location}")
            rospy.sleep(1.0)  # 确保消息发送
            
        else:
            # 无参数，打印使用说明
            rospy.loginfo("=== 机器人导航测试程序 ===")
            rospy.loginfo("使用说明:")
            rospy.loginfo("  --location (-l): 指定导航目标位置")
            rospy.loginfo("  --interactive (-i): 交互模式")
            rospy.loginfo("  --cycle (-c): 循环模式")
            rospy.loginfo("\n示例:")
            rospy.loginfo("  rosrun auto_match navigate_to_locations.py -l sorting_area")
            rospy.loginfo("  rosrun auto_match navigate_to_locations.py -i")
            rospy.loginfo("  rosrun auto_match navigate_to_locations.py -c")
            
            rospy.loginfo("\n等待 ROS 消息...")
            rospy.spin()
    
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"主程序发生错误: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序初始化错误: {e}")
        import traceback
        rospy.logerr(traceback.format_exc()) 