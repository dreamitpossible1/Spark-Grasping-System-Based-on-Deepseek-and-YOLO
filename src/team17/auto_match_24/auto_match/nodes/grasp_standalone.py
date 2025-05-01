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
        rospy.loginfo("初始化SwiftProInterface...")
        self.arm_position_pub = rospy.Publisher(
            "position_write_topic", position, queue_size=1)   # 机械臂运动位置发布者
        self.arm_pump_pub = rospy.Publisher(
            "pump_topic", status, queue_size=1)               # 机械臂气泵状态发布者
        self.arm_status_pub = rospy.Publisher(
            "swiftpro_status_topic", status, queue_size=1)    # 机械臂开关状态发布者
        rospy.loginfo("SwiftProInterface初始化完成")

    def set_pose(self, x, y, z, speed=800):
        '''
        发布机械臂运动位置
        '''
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        pos.speed = speed
        rospy.loginfo(f"设置机械臂位置: x={x}, y={y}, z={z}, speed={speed}")
        self.arm_position_pub.publish(pos)
        if abs(z) > 100:  # 如果高度较大，可能是大幅度移动
            rospy.loginfo("执行大幅度移动，请稍等...")

    def set_pump(self, enable: bool):
        '''
        吸取或释放，设定机械臂气泵状态
        '''
        if enable:
            rospy.loginfo("启动气泵 (吸取)")
            # 确保吸取命令被执行
            for i in range(3):  # 增加到3次
                self.arm_pump_pub.publish(status(1))
                rospy.sleep(0.5)  # 增加等待时间
            rospy.loginfo("已多次发送吸取命令")
        else:
            rospy.loginfo("关闭气泵 (释放)")
            # 确保释放命令被执行，多次发送释放命令
            for i in range(4):  # 增加到4次以确保可靠性
                self.arm_pump_pub.publish(status(0))
                rospy.sleep(0.5)  # 增加等待时间
            rospy.loginfo("已多次发送释放命令")

    def set_status(self, lock: bool):
        '''
        设定机械臂开关状态
        '''
        if lock:
            rospy.loginfo("锁定机械臂")
            self.arm_status_pub.publish(status(1))
        else:
            rospy.loginfo("解锁机械臂")
            self.arm_status_pub.publish(status(0))


class CamAction:
    def __init__(self):
        # 添加对检测控制的发布者
        self.detection_control_pub = rospy.Publisher("/grasp_cmd", String, queue_size=1)
        pass

    def detector(self):
        '''
        获取需要抓取的物品在显示屏上的坐标位置
        @return: 需要抓取的物品列表cube_list

        cube_list[i]:代表第几个物体
        cube_list[i][0]:代表第i个物体的ID信息;               cube_list[i][1]:代表第i个物体的位置信息
        cube_list[i][1][0]:代表第i个物体的x方向上的位置;     cube_list[i][1][1]:代表第i个物体的y方向上的位置
        '''
        rospy.loginfo("开始调用detector()方法获取物体信息")
        obj_dist = {}
        cube_list = []
        obj_array = None

        try:
            rospy.loginfo("等待/objects话题消息...")
            obj_array = rospy.wait_for_message(
                "/objects", Detection2DArray, timeout=5)
            rospy.loginfo(f"成功获取/objects消息，包含 {len(obj_array.detections)} 个检测结果")
        except Exception as e:
            rospy.logerr(f"无法获取object消息: {str(e)}")
            cube_list.clear()
            return cube_list
        
        # 提取
        rospy.loginfo("开始处理检测结果...")
        for i, obj in enumerate(obj_array.detections):
            obj_id = obj.results[0].id
            center_x = obj.bbox.center.x
            center_y = obj.bbox.center.y
            score = obj.results[0].score
            rospy.loginfo(f"检测到物体 #{i+1}: ID={obj_id}, 位置=({center_x}, {center_y}), 置信度={score}")
            
            # 修改此处，将每个检测到的物体直接添加到cube_list中
            # 而不是使用obj_dist字典（这样会导致相同ID的物体被覆盖）
            cube_list.append([obj_id, [center_x, center_y]])
            rospy.loginfo(f"直接添加到目标列表: ID={obj_id}, 位置=({center_x}, {center_y})")
            
        # 不再需要这个循环，因为我们已经在上面直接添加了
        # for key, value in obj_dist.items():
        #    cube_list.append([key, value])
        #    rospy.loginfo(f"添加到目标列表: ID={key}, 位置=({value[0]}, {value[1]})")

        rospy.loginfo(f"detector()返回结果: 找到 {len(cube_list)} 个目标物体")
        return cube_list
    
    def check_if_grasp(self, x, y, timeout=3, confidence=0.5, scope=30):
        """
        检查是否成功抓取了物体
        """
        # 先恢复物料检测更新以获取最新状态
        rospy.loginfo("临时恢复物料检测更新以检查抓取状态...")
        self.detection_control_pub.publish(String("resume"))
        rospy.sleep(1.0)  # 给检测系统一些时间更新
        
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
        
        # 检查完成后再次暂停物料检测更新
        rospy.loginfo("检查完成，重新暂停物料检测更新...")
        self.detection_control_pub.publish(String("pause"))
        
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

        # 不使用排除区域，直接设置为全白图像（所有区域都允许）
        self.exclude = np.ones((480, 640), dtype=np.uint8) * 255
        self.disallowed = np.ones((480, 640), dtype=np.uint8) * 255
        rospy.loginfo("已设置所有区域为可用区域，无排除区域")

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
        
        # 添加任务完成信号发布者
        self.task_complete_pub = rospy.Publisher("/grasp_cmd", String, queue_size=1)
        
    def grasp(self):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        rospy.loginfo("====== 开始抓取流程 ======")
        x = 0
        y = 0
        z = 0
        cube_list = []
        # 寻找物品
        rospy.loginfo("调用detector获取物体...")
        cube_list_tmp = self.cam.detector()
        rospy.loginfo(f"detector返回 {len(cube_list_tmp)} 个物体")
        
        # 打印所有检测到的物体信息
        for i, item in enumerate(cube_list_tmp):
            rospy.loginfo(f"检测到物体 #{i+1}: ID={item[0]}, 位置=({item[1][0]}, {item[1][1]})")
        
        if len(cube_list_tmp) == 0:
            rospy.logwarn("未检测到任何物体，退出抓取流程")
            return 0
        
        # 筛选出所有bowl (ID=45)物体
        bowl_list = [item for item in cube_list_tmp if item[0] == 45]
        rospy.loginfo(f"筛选得到 {len(bowl_list)} 个bowl物体")
        
        # 如果没有bowl，则使用原始列表
        if len(bowl_list) == 0:
            rospy.logwarn("未检测到bowl物体，使用原始物体列表")
        else:
            # 如果有bowl，则只使用bowl列表
            cube_list_tmp = bowl_list
            rospy.loginfo("使用筛选后的bowl列表继续处理")
        
        rospy.loginfo("开始筛选排除区域外的物体...")
        for pice in cube_list_tmp:
            xp = pice[1][0]
            yp = pice[1][1]
            # 检查是否在排除区域
            if 0 <= int(yp) < self.exclude.shape[0] and 0 <= int(xp) < self.exclude.shape[1]:
                if self.exclude[int(yp), int(xp)] >= 128:
                    cube_list.append(pice)
                    rospy.loginfo(f"物体ID={pice[0]}在允许区域内，添加到目标列表")
                else:
                    rospy.loginfo(f"物体ID={pice[0]}在排除区域内，忽略")
            else:
                # 如果坐标超出边界，直接添加
                cube_list.append(pice)
                rospy.loginfo(f"物体ID={pice[0]}坐标超出边界，默认添加到目标列表")

        if len(cube_list) == 0:
            rospy.logwarn("筛选后没有可抓取的物体，退出抓取流程")
            return 0

        # 根据y坐标对物体列表进行排序（从远到近）
        cube_list.sort(key=lambda x: x[1][1])
        
        # 检查是否所有物体都是bowl (ID=45)
        all_bowls = all(item[0] == 45 for item in cube_list)
        rospy.loginfo(f"是否所有物体都是bowl: {all_bowls}")
        
        # 处理三个bowl的情况
        if len(cube_list) == 3 and all_bowls:
            rospy.loginfo("检测到3个bowl，开始执行堆叠流程")
            
            # 最近的bowl（最后一个bowl）作为放置点
            target_bowl = cube_list[-1]
            target_x = self.x_kb[0] * target_bowl[1][1] + self.x_kb[1]
            target_y = self.y_kb[0] * target_bowl[1][0] + self.y_kb[1]
            
            # 记录当前已堆叠的bowl数量，用于确定下一个bowl的高度
            stacked_bowls = 0
            
            # 获取要处理的bowl列表（除了目标bowl外的所有bowl）
            bowls_to_process = cube_list[:-1]
            rospy.loginfo(f"目标bowl(放置点)位置: ({target_bowl[1][0]}, {target_bowl[1][1]})")
            rospy.loginfo(f"需要处理 {len(bowls_to_process)} 个bowl进行堆叠")
            
            # 处理其他两个bowl
            for idx, bowl in enumerate(bowls_to_process):
                rospy.loginfo(f"处理第 {idx+1}/{len(bowls_to_process)} 个bowl...")
                
                # 移动到bowl上方
                x = self.x_kb[0] * bowl[1][1] + self.x_kb[1]
                y = self.y_kb[0] * bowl[1][0] + self.y_kb[1]
                z = -15.0
                
                rospy.loginfo(f"移动到bowl (ID={bowl[0]}) 上方... 坐标: ({x}, {y}, {z+60})")
                self.interface.set_pose(x, y, z + 60)
                rospy.sleep(3.0)
                
                # 下移并开启吸盘
                rospy.loginfo("下移并开启吸盘...")
                self.interface.set_pose(x, y, z)
                rospy.sleep(3.0)
                self.interface.set_pump(True)
                rospy.sleep(1.5)
                
                # 上抬
                rospy.loginfo("上抬bowl...")
                self.interface.set_pose(x, y, z + 120)
                rospy.sleep(3.0)
                
                # 移动到放置点上方
                rospy.loginfo(f"移动到放置点上方... 坐标: ({target_x}, {target_y}, {z+120})")
                self.interface.set_pose(target_x, target_y, z + 120)
                rospy.sleep(3.0)
                
                # 根据当前堆叠数量确定释放高度
                # 第一个bowl放在z=-5, 第二个bowl放在z=0
                release_heights = [-5, 0]
                release_height = release_heights[stacked_bowls] if stacked_bowls < len(release_heights) else release_heights[-1]
                
                # 下移到释放高度上方5厘米
                approach_height = release_height + 50  # 留出足够的空间
                rospy.loginfo(f"下移到释放高度上方: z={approach_height}...")
                self.interface.set_pose(target_x, target_y, approach_height)
                rospy.sleep(3.0)
                
                # 下移到释放高度
                rospy.loginfo(f"下移到释放高度: z={release_height}...")
                self.interface.set_pose(target_x, target_y, release_height)
                rospy.sleep(3.0)
                
                # 关闭吸盘
                rospy.loginfo("释放bowl...")
                # 多次发送关闭吸盘命令以确保可靠性
                for _ in range(3):
                    self.interface.set_pump(False)
                    rospy.sleep(0.5)
                rospy.loginfo("已发送多次关闭吸盘命令")
                rospy.sleep(1.0)
                
                # 先小幅上抬，防止卡住
                rospy.loginfo("释放后小幅上抬，防止卡住...")
                self.interface.set_pose(target_x, target_y, release_height + 15)
                rospy.sleep(1.5)
                
                # 再上抬到安全位置
                rospy.loginfo("上抬到安全位置...")
                self.interface.set_pose(target_x, target_y, z + 120)
                rospy.sleep(3.0)

                # 增加已堆叠数量
                stacked_bowls += 1
                rospy.loginfo(f"已堆叠bowl数量: {stacked_bowls}/{len(bowls_to_process)}")
            
            # 再次确认堆叠结果
            if stacked_bowls == len(bowls_to_process):
                rospy.loginfo(f"全部 {stacked_bowls} 个bowl堆叠完成！")
            else:
                rospy.logwarn(f"只完成了 {stacked_bowls}/{len(bowls_to_process)} 个bowl的堆叠")
                
            rospy.loginfo("三个bowl堆叠完成")
            return 3
            
        # 处理两个bowl的情况
        elif len(cube_list) == 2:
            rospy.loginfo("检测到2个bowl，开始执行堆叠流程")
            # 较近的bowl作为放置点
            target_bowl = cube_list[-1]
            target_x = self.x_kb[0] * target_bowl[1][1] + self.x_kb[1]
            target_y = self.y_kb[0] * target_bowl[1][0] + self.y_kb[1]
            
            # 处理较远的bowl
            bowl = cube_list[0]
            x = self.x_kb[0] * bowl[1][1] + self.x_kb[1]
            y = self.y_kb[0] * bowl[1][0] + self.y_kb[1]
            z = -10.0
            
            rospy.loginfo(f"移动到远处bowl (ID={bowl[0]}) 上方... 坐标: ({x}, {y}, {z+40})")
            self.interface.set_pose(x, y, z + 40)
            rospy.sleep(2.0)
            
            # 下移并开启吸盘
            rospy.loginfo("下移并开启吸盘...")
            self.interface.set_pose(x, y, z)
            rospy.sleep(2.0)
            self.interface.set_pump(True)
            rospy.sleep(1.0)
            
            # 上抬
            rospy.loginfo("上抬bowl...")
            self.interface.set_pose(x, y, z + 120)
            rospy.sleep(2.0)
            
            # 移动到放置点上方
            rospy.loginfo(f"移动到放置点上方... 坐标: ({target_x}, {target_y}, {z+120})")
            self.interface.set_pose(target_x, target_y, z + 120)
            rospy.sleep(2.0)
            
            # 下移到释放高度上方5厘米
            rospy.loginfo("下移到释放高度上方5厘米: z=-5+5=0...")
            self.interface.set_pose(target_x, target_y, 0)
            rospy.sleep(2.0)
            
            # 下移到释放高度
            rospy.loginfo("下移到释放高度: z=-5...")
            self.interface.set_pose(target_x, target_y, -5)
            rospy.sleep(1.0)
            
            # 关闭吸盘
            rospy.loginfo("释放bowl...")
            # 多次发送关闭吸盘命令以确保可靠性
            for _ in range(3):
                self.interface.set_pump(False)
                rospy.sleep(0.5)
            rospy.loginfo("已发送多次关闭吸盘命令")
            rospy.sleep(1.0)
            
            # 先小幅上抬，防止卡住
            rospy.loginfo("释放后小幅上抬，防止卡住...")
            self.interface.set_pose(target_x, target_y, 10)  # 上抬到z=10
            rospy.sleep(1.5)
            
            # 上抬到安全位置
            rospy.loginfo("上抬到安全位置...")
            self.interface.set_pose(target_x, target_y, z + 120)
            rospy.sleep(2.0)
            
            rospy.loginfo("两个bowl堆叠完成")
            return 2
            
        # 如果只有一个bowl或其他情况，执行原有的抓取逻辑
        else:
            if len(cube_list) == 1:
                rospy.loginfo("====== 检测结果 ======")
                rospy.loginfo("桌面整洁，无需整理")
                rospy.loginfo("====================")
                return 0
                
            # 原有的抓取逻辑
            closest_x = cube_list[0][1][0]
            closest_y = cube_list[0][1][1]
            id = cube_list[0][0]
            
            # 获取机械臂目标位置
            x = self.x_kb[0] * closest_y + self.x_kb[1]
            y = self.y_kb[0] * closest_x + self.y_kb[1]
            z = -10.0
            rospy.loginfo(f"转换为机械臂坐标: x={x}, y={y}, z={z}")
            
            # 机械臂移动到目标位置上方
            rospy.loginfo("移动机械臂到目标上方...")
            self.interface.set_pose(x, y, z + 40)
            rospy.sleep(2.5)
            
            # 机械臂移动到目标位置
            rospy.loginfo("移动机械臂到目标位置...")
            self.interface.set_pose(x, y, z)
            rospy.sleep(2.0)

            # 打开气泵，进行吸取
            rospy.loginfo("启动气泵进行吸取...")
            self.interface.set_pump(True)
            rospy.sleep(1.0)
            
            # 抬起目标方块
            rospy.loginfo("抬起物体...")
            self.interface.set_pose(x, y, z + 120)
            rospy.sleep(2.0)
            
            rospy.loginfo("将机械臂移动到安全位置...")
            self.arm_default_pose()
            rospy.sleep(1.75)
            
            # 检查是否成功抓取
            rospy.loginfo("检查抓取是否成功...")
            grasp_success = self.cam.check_if_grasp(closest_x, closest_y, timeout=1.5, confidence=0.3)
            
            if not grasp_success:
                rospy.logwarn("抓取可能失败，重置机械臂...")
                self.interface.set_pump(False)  # 确保关闭气泵
                rospy.sleep(0.5)
                self.reset_pub.publish(position(10, 150, 160, 0))
                return 1
                
            rospy.loginfo(f"抓取成功，物体ID: {id}")
            self.grasp_status_pub.publish(String("0"))
            
            # 更新堆叠次数
            if id in self.time and self.time[id] < 3:
                self.time[id] += 1
                rospy.loginfo(f"物体ID={id}的堆叠次数更新为: {self.time[id]}")
            else:
                if id not in self.time:
                    rospy.logwarn(f"未在堆叠记录中找到ID={id}，默认不更新堆叠次数")
                    self.time[id] = 1
                    rospy.loginfo(f"已添加新ID={id}到堆叠记录，初始堆叠次数: 1")
                    
            rospy.loginfo("====== 抓取流程结束 ======")
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
        self.interface.set_pose(200, 110, 175, 1000)
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
            self.interface.set_pose(200, 110, 175, 1000)
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
                self.interface.set_pose(200, 110, 175, 1000)
                rospy.sleep(0.75)
                self.arm_default_pose()
                self.grasp_status_pub.publish(String("0"))
                self.complete[item] = True
                
                # 检查是否所有物体都已完成堆叠
                all_complete = True
                for key, value in self.complete.items():
                    if not value:
                        all_complete = False
                        break
                
                # 如果所有物体都完成了堆叠，发送重置bowl列表的信号
                if all_complete:
                    rospy.loginfo("所有物体堆叠任务完成，发送重置bowl列表信号")
                    self.task_complete_pub.publish(String("complete_task"))
                    # 重置完成状态，为下一轮做准备
                    for key in self.complete:
                        self.complete[key] = False
                    rospy.loginfo("已重置堆叠完成状态")
                
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
        rospy.loginfo("接收到激光雷达数据，开始检查前方物体...")
        tested = False
        rospy.sleep(3.0)
        
        for i in range(10):
            obstacle_detected = False
            for idx, distance in enumerate(data.ranges):
                if distance == 0.0:
                    continue
                if distance < 0.3:
                    angle = idx * data.angle_increment + data.angle_min
                    angle_degrees = angle * 180.0 / math.pi
                    rospy.loginfo(f"检测到障碍物: 距离={distance}m, 角度={angle_degrees:.1f}度")
                    obstacle_detected = True
                    break
                    
            if obstacle_detected:
                tested = True
                rospy.loginfo("确认前方有物体")
            else:
                rospy.loginfo("前方未检测到物体")
                
            rospy.sleep(0.3)
            break

        self.is_in = tested
        rospy.loginfo(f"物体检测状态更新为: {self.is_in}")
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
        self.interface.set_pose(10, 170, 160, 1000)
        rospy.sleep(1.0)


class GraspStack:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('grasp_stack_node', anonymous=True)
        
        # 实例化Arm操作类
        self.arm = ArmAction()
        
        # 订阅控制命令
        rospy.loginfo("订阅grasp_cmd话题，等待控制命令...")
        self.cmd_sub = rospy.Subscriber("grasp_cmd", String, self.cmd_callback)
        
        # 其他初始化
        self.item_type = 0
        
        # 添加一个定期检查的定时器
        self.check_timer = rospy.Timer(rospy.Duration(10), self.periodic_check)
        
        # 添加重置bowl列表的发布者
        self.reset_bowl_pub = rospy.Publisher("/reset_bowl_list", String, queue_size=1)
        rospy.loginfo("创建重置bowl列表触发信号发布者")
        
        # 添加物料检测控制发布者
        self.detection_control_pub = rospy.Publisher("/grasp_cmd", String, queue_size=1)
        rospy.loginfo("创建物料检测控制信号发布者")
        
        rospy.loginfo("Grasp and stack node初始化完成，等待命令...")
        
    def periodic_check(self, event):
        """定期检查系统状态"""
        cube_list = self.arm.cam.detector()
        rospy.loginfo(f"定期检查: 当前视野中有 {len(cube_list)} 个物体")
        if len(cube_list) > 0:
            rospy.loginfo(f"可见物体: {cube_list}")
        
    def reset_bowl_list(self):
        """发送重置bowl列表的触发信号"""
        rospy.loginfo("发布重置bowl列表的触发信号")
        self.reset_bowl_pub.publish(String("reset_all_bowls"))
        rospy.loginfo("触发信号已发送")

    def cmd_callback(self, msg):
        """
        接收命令并执行相应操作
        命令格式:
            - "grasp": 执行抓取操作
            - "drop": 执行放置操作
            - "reset": 重置机械臂到默认位置
        """
        cmd = msg.data
        rospy.loginfo(f"收到命令: '{cmd}'")
        
        if cmd == "grasp":
            # 暂停物料检测更新 - 仅暂停其他检测操作，不影响抓取检测
            rospy.loginfo("暂停外部物料检测更新...")
            self.detection_control_pub.publish(String("pause"))
            rospy.sleep(0.5)  # 等待命令生效
            
            # 执行抓取
            rospy.loginfo("开始执行抓取操作...")
            self.item_type = self.arm.grasp()
            rospy.loginfo(f"抓取完成，物体类型: {self.item_type}")
            
            # 抓取结果处理
            if self.item_type == 0:
                rospy.logwarn("未能找到或抓取物体!")
                # 恢复物料检测更新
                rospy.loginfo("恢复物料检测更新...")
                self.detection_control_pub.publish(String("resume"))
            elif self.item_type == 1:
                rospy.logwarn("抓取可能不成功，需要重试")
                # 恢复物料检测更新
                rospy.loginfo("恢复物料检测更新...")
                self.detection_control_pub.publish(String("resume"))
            else:
                rospy.loginfo(f"成功抓取ID为{self.item_type}的物体，保持物料检测暂停状态，直到放置完成")
            
        elif cmd == "drop":
            # 执行放置
            if self.item_type > 0:
                rospy.loginfo(f"开始放置ID为{self.item_type}的物体...")
                self.arm.drop(self.item_type)
                rospy.loginfo(f"放置完成，物体: {self.item_type}")
                self.item_type = 0
                
                # 恢复物料检测更新
                rospy.loginfo("恢复物料检测更新...")
                self.detection_control_pub.publish(String("resume"))
                rospy.sleep(0.5)  # 等待命令生效
            else:
                rospy.logwarn("没有抓取的物体，无法执行放置操作")
                
        elif cmd == "reset":
            # 重置机械臂
            rospy.loginfo("重置机械臂到默认位置...")
            self.arm.arm_default_pose()
            rospy.loginfo("机械臂重置完成")
            
            # 恢复物料检测更新
            rospy.loginfo("恢复物料检测更新...")
            self.detection_control_pub.publish(String("resume"))
            
        elif cmd == "check":
            # 检查当前状态
            cube_list = self.arm.cam.detector()
            rospy.loginfo(f"当前状态检查: 视野中有 {len(cube_list)} 个物体")
            if len(cube_list) > 0:
                rospy.loginfo(f"可见物体: {cube_list}")
            rospy.loginfo(f"当前抓取的物体ID: {self.item_type}")
            
        elif cmd == "reset_bowl_list":
            # 重置bowl列表的触发命令
            self.reset_bowl_list()
            
        elif cmd == "complete_task":
            # 完成所有任务，重置bowl列表并重置机械臂
            rospy.loginfo("完成所有任务，重置bowl列表并重置机械臂")
            self.reset_bowl_list()
            self.arm.arm_default_pose()
            rospy.loginfo("任务完成，所有系统已重置")
            
        elif cmd == "pause" or cmd == "resume":
            # 这些命令由检测器处理，这里不做任何操作
            rospy.loginfo(f"收到检测控制命令: {cmd}，由检测器处理")
            
        else:
            rospy.logwarn(f"未知命令: '{cmd}'，支持的命令有: grasp, drop, reset, check, reset_bowl_list, complete_task")

    def run(self):
        """
        主运行循环
        """
        rospy.loginfo("Grasp and Stack节点正在运行...")
        # 启动时进行一次检查
        rospy.Timer(rospy.Duration(2), lambda event: self.cmd_callback(String("check")), oneshot=True)
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.loginfo("====================================================")
        rospy.loginfo("               启动抓取堆叠独立节点                  ")
        rospy.loginfo("====================================================")
        rospy.loginfo("支持的命令:")
        rospy.loginfo("  - rostopic pub /grasp_cmd std_msgs/String \"grasp\" -1")
        rospy.loginfo("  - rostopic pub /grasp_cmd std_msgs/String \"drop\" -1")
        rospy.loginfo("  - rostopic pub /grasp_cmd std_msgs/String \"reset\" -1")
        rospy.loginfo("  - rostopic pub /grasp_cmd std_msgs/String \"check\" -1")
        rospy.loginfo("  - rostopic pub /grasp_cmd std_msgs/String \"reset_bowl_list\" -1")
        rospy.loginfo("  - rostopic pub /grasp_cmd std_msgs/String \"complete_task\" -1")
        rospy.loginfo("  - rostopic pub /grasp_cmd std_msgs/String \"pause\" -1  # 暂停物料检测更新")
        rospy.loginfo("  - rostopic pub /grasp_cmd std_msgs/String \"resume\" -1  # 恢复物料检测更新")
        rospy.loginfo("====================================================")
        rospy.loginfo("物料检测控制逻辑：")
        rospy.loginfo("  - 执行抓取时自动暂停物料检测更新")
        rospy.loginfo("  - 完成抓取或失败时自动恢复物料检测更新")
        rospy.loginfo("  - 检测到3个bowl时不再更新bowl列表")
        rospy.loginfo("====================================================")
        
        node = GraspStack()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Grasp and Stack node terminated.") 