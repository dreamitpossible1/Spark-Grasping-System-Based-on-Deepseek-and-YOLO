#!/usr/bin/python3

import os
import threading
import math
import rospy
import rospkg
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from swiftpro.msg import position, status


class SwiftProInterface:
    def __init__(self):
        # 创建控制机械臂的topic发布者
        self.arm_position_pub = rospy.Publisher(
            "position_write_topic", position, queue_size=1)   # 机械臂运动位置发布者
        self.arm_pump_pub = rospy.Publisher(
            "pump_topic", status, queue_size=1)               # 机械臂气泵状态发布者
        self.arm_status_pub = rospy.Publisher(
            "swiftpro_status_topic", status, queue_size=1)    # 机械臂开关状态发布者

    def set_pose(self, x, y, z, speed=1000):
        '''
        发布机械臂运动位置
        '''
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        pos.speed = speed
        # rospy.loginfo(f"set pose {x},{y},{z}")
        self.arm_position_pub.publish(pos)

    def set_pump(self, enable: bool):
        '''
        吸取或释放，设定机械臂气泵状态
        '''
        rospy.loginfo(f"设定机械臂气泵状态为：{enable}")
        if enable:
            self.arm_pump_pub.publish(status(1))
        else:
            self.arm_pump_pub.publish(status(0))

    def set_status(self, lock: bool):
        '''
        设定机械臂开关状态
        '''
        rospy.loginfo(f"set arm status {lock}")
        if lock:
            self.arm_status_pub.publish(status(1))
        else:
            self.arm_status_pub.publish(status(0))


class CamAction:
    def __init__(self):
        pass

    def detector(self):
        '''
        获取需要抓取的物品在显示屏上的坐标位置
        @return: 需要抓取的物品列表cube_list

        cube_list[i]:代表第几个物体
        cube_list[i][0]:代表第i个物体的ID信息;               cube_list[i][1]:代表第i个物体的位置信息
        cube_list[i][1][0]:代表第i个物体的x方向上的位置;     cube_list[i][1][1]:代表第i个物体的y方向上的位置
        '''
        obj_dist = {}
        cube_list = []
        obj_array = None

        try:
            obj_array = rospy.wait_for_message(
                "/objects", Detection2DArray, timeout=5)
        except Exception:
            rospy.logerr("can't get object message")
            cube_list.clear()
            return cube_list
        
        # 提取
        for obj in obj_array.detections:
            obj_dist[obj.results[0].id] = [obj.bbox.center.x, obj.bbox.center.y]

        # 筛选出需要的物品 cube_list中的key代表识别物体的ID，value代表位置信息
        for key, value in obj_dist.items():
            cube_list.append([key, value])

        return cube_list
    
    def check_if_grasp(self, x, y, timeout=3, confidence=0.5, scope=30):
        """
        检查是否成功抓取了物体
        """
        stat = False
        total = int(timeout * 4)
        count = 0
        for _ in range(total):
            cube_list = self.detector()
            for pice in cube_list:
                if x - scope < pice[1][0] < x + scope and y - scope < pice[1][1] < y + scope:
                    count += 1
                    break
            rospy.sleep(0.25)
        if count / total > confidence:
            stat = False
            rospy.loginfo("grasp failed")
        else:
            stat = True
            rospy.loginfo("grasp success")
        return stat


class FixRotate:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def step_rotate_pro(self, rad, time=0.5):
        twist = Twist()
        twist.angular.z = rad
        self.cmd_pub.publish(twist)
        rospy.sleep(time)
        twist.angular.z = 0
        self.cmd_pub.publish(twist)


class ArmAction:
    def __init__(self):
        self.cam = CamAction()

        # 获取第一层标定文件数据
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
            f.close()
        del filename
        arr = s.split()
        del s
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]
        arr.clear()
        del arr

        # 获取第二层标定文件数据
        filename = os.environ['HOME'] + "/thefile2.txt"
        with open(filename, 'r') as f:
            s = f.read()
            f.close()
        del filename
        arr = s.split()
        del s
        self.x2_kb = [float(arr[0]), float(arr[1])]
        self.y2_kb = [float(arr[2]), float(arr[3])]
        arr.clear()
        del arr

        # 创建机械臂控制接口的对象
        self.interface = SwiftProInterface()

        self.height, self.width = 480, 640
        self.center_x = self.width // 2  # 图像底边中心的x坐标
        self.bottom_y = self.height  # 图像底边的y坐标

        self.time = {46: 0, 88: 0, 85: 0}  # 堆叠次数记录
        self.block_height = 100  # 方块高度
        self.is_in = False
        self.testing = False
        self.complete = {46: False, 88: False, 85: False}
        self.fix_rotate = FixRotate()
        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)
        self.reset_pub = rospy.Publisher("armreset_pro", position, queue_size=1)
        self.lidar_sub = rospy.Subscriber(
            "/scan", LaserScan, self.check_scan_stat, queue_size=1, buff_size=2**24)
        
        # 加载排除区域的图像
        try:
            self.exclude = np.array(cv2.imread(os.path.join(
                rospkg.RosPack().get_path('auto_match'), 'config', 'tmp.png'), 0))
            self.disallowed = np.array(cv2.imread(os.path.join(
                rospkg.RosPack().get_path('auto_match'), 'config', 'disallowed_area.png'), 0))
        except:
            rospy.logwarn("无法加载排除区域图像，将使用空白图像")
            self.exclude = np.ones((480, 640), dtype=np.uint8) * 255
            self.disallowed = np.ones((480, 640), dtype=np.uint8) * 255

    def grasp(self):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        x = 0
        y = 0
        z = 0
        cube_list = []
        # 寻找物品
        cube_list_tmp = self.cam.detector()
        if len(cube_list_tmp) == 0:
            return 0
        
        for pice in cube_list_tmp:
            xp = pice[1][0]
            yp = pice[1][1]
            # 检查是否在排除区域
            if 0 <= int(yp) < self.exclude.shape[0] and 0 <= int(xp) < self.exclude.shape[1]:
                if self.exclude[int(yp), int(xp)] >= 128:
                    cube_list.append(pice)
            else:
                # 如果坐标超出边界，直接添加
                cube_list.append(pice)

        if len(cube_list) == 0:
            return 0
            
        closest_x = cube_list[0][1][0]
        closest_y = cube_list[0][1][1]
        id = cube_list[0][0]
        for pice in cube_list:
            xp = pice[1][0]
            yp = pice[1][1]
            if yp > closest_y:
                closest_x = xp
                closest_y = yp
                id = pice[0]
                
        rospy.loginfo(f"物品位置在: {closest_x}, {closest_y}")
        # 获取机械臂目标位置
        x = self.x_kb[0] * closest_y + self.x_kb[1]
        y = self.y_kb[0] * closest_x + self.y_kb[1]
        z = -50.0
        rospy.loginfo(f"arm: {x}, {y}, {z}")
        # 机械臂移动到目标位置上方
        self.interface.set_pose(x, y, z + 20)
        rospy.sleep(0.5)
        self.interface.set_pose(x, y, z)
        # 打开气泵，进行吸取
        self.interface.set_pump(True)
        rospy.sleep(1.0)
        # 抬起目标方块
        rospy.loginfo(f"把物品抬起来")
        self.interface.set_pose(x, y, z + 120)
        rospy.sleep(1.0)
        rospy.loginfo(f"摆到旁边")
        self.arm_default_pose()
        rospy.sleep(0.75)
        if not self.cam.check_if_grasp(closest_x, closest_y, timeout=1.5, confidence=0.3):
            self.reset_pub.publish(position(10, 150, 160, 0))
            return 1
        self.grasp_status_pub.publish(String("0"))
        if self.time[id] < 3:
            self.time[id] += 1
        return id

    def drop(self, item):
        """
        根据物体ID和已堆叠次数选择合适的放置函数
        """
        self.complete[item] = False
        if self.time[item] == 3:
            if self.drop_step_three(item):
                return
        if self.time[item] == 2:
            if self.drop_step_two(item):
                return
        if self.time[item] == 1:
            self.drop_step_one(item)

    def drop_step_one(self, _):
        '''
        放置方块在第一层
        '''
        x = 230
        y = 0
        z = 175
        self.interface.set_pose(200, 110, 175)
        rospy.sleep(0.75)
        self.interface.set_pose(x, y, z)
        rospy.sleep(1.25)
        z = -125 + self.block_height
        self.interface.set_pose(x, y, z)
        rospy.sleep(1.5)
        z = z - 25
        self.interface.set_pose(x, y, z)
        rospy.sleep(0.5)
        # 关闭气泵
        self.interface.set_pump(False)
        rospy.sleep(1.0)
        # 向上移动一点
        z = z + 25
        self.interface.set_pose(x, y, z)
        rospy.sleep(0.5)
        self.interface.set_pose(200, 110, 175, 250)
        rospy.sleep(0.75)
        self.arm_default_pose()
        self.grasp_status_pub.publish(String("0"))
        return True
    
    def drop_step_two(self, item):
        '''
        放置方块在第二层，需要先找到底下的方块
        '''
        found = False
        count = 0
        while not found and count < 10:
            count += 1
            time = 10
            cube_list_tmp = []
            cube_list = []
            cube_list_tmp = self.cam.detector()
            for i in range(time):
                rospy.logwarn(f"list is empty, check time: {i}")
                rospy.sleep(0.5)
                for pice in cube_list_tmp:
                    xp = pice[1][0]
                    yp = pice[1][1]
                    if 0 <= int(yp) < self.disallowed.shape[0] and 0 <= int(xp) < self.disallowed.shape[1]:
                        if self.disallowed[int(yp), int(xp)] > 128:
                            cube_list.append(pice)
                    else:
                        cube_list.append(pice)
                if len(cube_list):
                    break
                else:
                    cube_list_tmp = self.cam.detector()

            if len(cube_list) == 0:
                self.time[item] = 1
                return False

            closest_x = cube_list[0][1][0]
            closest_y = cube_list[0][1][1]
            id = cube_list[0][0]

            for pice in cube_list:
                xp = pice[1][0]
                yp = pice[1][1]
                if yp < closest_y:
                    closest_x = xp
                    closest_y = yp
                    id = pice[0]
            
            if closest_x < 100:
                self.fix_rotate.step_rotate_pro(-0.8)
                rospy.sleep(0.75)
            elif closest_x > 540:
                self.fix_rotate.step_rotate_pro(0.8)
                rospy.sleep(0.75)
            else:
                found = True

        if id == item and 40 <= closest_y:
            x = self.x_kb[0] * closest_y + self.x_kb[1]
            y = self.y_kb[0] * closest_x + self.y_kb[1]
            z = 175
            self.interface.set_pose(200, 110, 175)
            rospy.sleep(0.75)
            self.interface.set_pose(x, y, z)
            rospy.sleep(1.25)
            z = -125 + self.block_height * 2
            self.interface.set_pose(x, y, z)
            rospy.sleep(1.5)
            z = z - 25
            self.interface.set_pose(x, y, z)
            rospy.sleep(0.5)
            # 关闭气泵
            self.interface.set_pump(False)
            rospy.sleep(1.0)
            # 向上移动一点
            z = z + 25
            self.interface.set_pose(x, y, z)
            rospy.sleep(0.5)
            self.interface.set_pose(200, 110, 175, 250)
            rospy.sleep(0.75)
            self.arm_default_pose()
            self.grasp_status_pub.publish(String("0"))
            return True
        else:
            self.time[item] = 1
            return False
        
    def drop_step_three(self, item, fantastic=False):
        '''
        放置方块在第三层，需要先找到底下的方块
        '''
        if fantastic:
            rospy.logwarn("fantastic mode start!!!")
            rospy.sleep(3)
        self.testing = True
        while self.testing:
            rospy.logwarn("check is in")
            rospy.sleep(0.3)

        if self.is_in:
            found = False
            count = 0
            while not found and count < 10:
                count += 1
                time = 10
                cube_list_tmp = []
                cube_list = []
                cube_list_tmp = self.cam.detector()
                for i in range(time):
                    rospy.logwarn(f"list is empty, check time: {i}")
                    rospy.sleep(0.5)
                    for pice in cube_list_tmp:
                        xp = pice[1][0]
                        yp = pice[1][1]
                        if 0 <= int(yp) < self.disallowed.shape[0] and 0 <= int(xp) < self.disallowed.shape[1]:
                            if self.disallowed[int(yp), int(xp)] > 128:
                                cube_list.append(pice)
                        else:
                            cube_list.append(pice)
                    if len(cube_list):
                        break
                    else:
                        cube_list_tmp = self.cam.detector()

                if len(cube_list) == 0:
                    self.time[item] = 2
                    return False
                
                closest_x = cube_list[0][1][0]
                closest_y = cube_list[0][1][1]
                id = cube_list[0][0]

                for pice in cube_list:
                    xp = pice[1][0]
                    yp = pice[1][1]
                    if yp < closest_y:
                        closest_x = xp
                        closest_y = yp
                        id = pice[0]

                if closest_x < 40:
                    self.fix_rotate.step_rotate_pro(-0.35)
                    rospy.sleep(0.75)
                elif closest_x > 600:
                    self.fix_rotate.step_rotate_pro(0.35)
                    rospy.sleep(0.75)
                else:
                    found = True

            if id == item:
                x = self.x2_kb[0] * closest_y + self.x2_kb[1]
                y = self.y2_kb[0] * closest_x + self.y2_kb[1]
                z = -125 + self.block_height * 3
                self.interface.set_pose(200, 110, 175)
                rospy.sleep(0.75)
                self.interface.set_pose(x, y, z)
                rospy.sleep(1.25)
                z = z - 25
                self.interface.set_pose(x, y, z)
                rospy.sleep(0.5)
                # 关闭气泵
                self.interface.set_pump(False)
                rospy.sleep(1.0)
                # 向上移动一点
                z = z + 25
                self.interface.set_pose(x, y, z)
                rospy.sleep(0.5)
                self.interface.set_pose(200, 110, 175, 250)
                rospy.sleep(0.75)
                self.arm_default_pose()
                self.grasp_status_pub.publish(String("0"))
                self.complete[item] = True
                return True
            else:
                self.time[item] = 2
                return False
        else:
            self.time[item] = 2
            return False
        
    def check_scan_stat(self, data):
        """
        检查激光雷达数据，判断是否有物体在前方
        """
        tested = False
        rospy.sleep(3.0)
        for _ in range(10):
            for distance in data.ranges:
                tested_once = False
                if distance == 0.0:
                    continue
                if distance < 0.3:
                    tested_once = True
                    break
            if tested_once:
                tested = True
            rospy.sleep(0.3)
            break

        self.is_in = tested
        self.testing = False

    def arm_home(self, block=False):
        '''
        收起机械臂(无物品)
        '''
        self.interface.set_pose(130, 0, 35)
        if block:
            rospy.sleep(1.0)

    def arm_default_pose(self):
        '''
        移动机械臂到摄像头看不到的地方，以方便识别与抓取
        '''
        self.interface.set_pose(10, 170, 160, 100)
        rospy.sleep(1.0)


class GraspStack:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('grasp_stack_node', anonymous=True)
        
        # 实例化Arm操作类
        self.arm = ArmAction()
        
        # 订阅控制命令
        self.cmd_sub = rospy.Subscriber("grasp_cmd", String, self.cmd_callback)
        
        # 其他初始化
        self.item_type = 0
        rospy.loginfo("Grasp and stack node initialized")
        
    def cmd_callback(self, msg):
        """
        接收命令并执行相应操作
        命令格式:
            - "grasp": 执行抓取操作
            - "drop": 执行放置操作
            - "reset": 重置机械臂到默认位置
        """
        cmd = msg.data
        if cmd == "grasp":
            # 执行抓取
            self.item_type = self.arm.grasp()
            rospy.loginfo(f"Grasped item type: {self.item_type}")
            
        elif cmd == "drop":
            # 执行放置
            if self.item_type > 0:
                self.arm.drop(self.item_type)
                rospy.loginfo(f"Dropped item type: {self.item_type}")
                self.item_type = 0
            else:
                rospy.logwarn("No item to drop")
                
        elif cmd == "reset":
            # 重置机械臂
            self.arm.arm_default_pose()
            rospy.loginfo("Arm reset to default pose")
            
        else:
            rospy.logwarn(f"Unknown command: {cmd}")

    def run(self):
        """
        主运行循环
        """
        rospy.loginfo("Grasp and Stack node is running...")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = GraspStack()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Grasp and Stack node terminated.") 