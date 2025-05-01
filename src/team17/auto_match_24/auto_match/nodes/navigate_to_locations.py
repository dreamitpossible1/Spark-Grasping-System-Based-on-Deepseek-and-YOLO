#!/usr/bin/python3

import rospy
from robot_navigation import RobotNavigator

def main():
    """
    主程序：在两个预先标记的位置之间导航
    """
    # 初始化节点
    rospy.init_node('location_navigator', anonymous=True)
    
    # 创建导航器实例
    navigator = RobotNavigator()
    
    # 假设有两个标记的位置："location_1" 和 "location_2"
    # 您需要根据实际标记的位置名称替换这些值
    location_1 = "sorting_area"  # 例如：整理区
    location_2 = "placement_area"  # 例如：放置区
    
    try:
        # 循环在两个位置之间来回导航
        while not rospy.is_shutdown():
            # 导航到第一个位置
            rospy.loginfo(f"正在导航到位置: {location_1}")
            result = navigator.goto_local(location_1)
            
            if result:
                rospy.loginfo(f"成功到达位置: {location_1}")
                # 在位置1停留一段时间
                rospy.sleep(5.0)
            else:
                rospy.logwarn(f"导航到位置 {location_1} 失败")
            
            # 检查是否应该继续
            if rospy.is_shutdown():
                break
                
            # 导航到第二个位置
            rospy.loginfo(f"正在导航到位置: {location_2}")
            result = navigator.goto_local(location_2)
            
            if result:
                rospy.loginfo(f"成功到达位置: {location_2}")
                # 在位置2停留一段时间
                rospy.sleep(5.0)
            else:
                rospy.logwarn(f"导航到位置 {location_2} 失败")
    
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 