import socket
import time
from uarm.wrapper import SwiftAPI
import rospy
from geometry_msgs.msg import Twist
import signal
import sys
import threading
import json
import math

class TakeUser:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("take_user_node", anonymous=True)
        # 发布速度命令到机器人
        self.vel_pub = rospy.Publisher("/robo5/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz
        
        # 初始化机械臂
        self.swift = SwiftAPI(port='/dev/ttyACM0')
        self.running = True
        
        # 统一的theta值定义
        TARGET_THETA = 1.57
        
        # Home位置定义
        self.home_position = {'x': 0.79, 'y': -5.87, 'theta': TARGET_THETA}
        
        # 目标位置定义
        self.target_positions = {
            'cell.phone': {'x': 0.74, 'y': -4.80, 'theta': TARGET_THETA},
            'cup': {'x': 1.30, 'y': -4.89, 'theta': TARGET_THETA},
            'mouse': {'x': 0.20, 'y': -4.82, 'theta': TARGET_THETA}
        }
        
        # 添加等待确认的标志
        self.waiting_for_confirmation = False
        self.current_action = None
        self.received_coordinates = None
    
    def signal_handler(self, signum, frame):
        """处理Ctrl+C信号"""
        print("\n收到退出信号，正在安全退出...")
        self.running = False
        self.stop_robot()
        try:
            if hasattr(self.swift, 'disconnect'):
                self.swift.disconnect()
        except:
            pass
        sys.exit(0)
    
    def stop_robot(self):
        """停止底盘"""
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)
        print("底盘已停止")

    def calculate_distance(self, current_pos, target_pos):
        """计算当前位置到目标位置的距离"""
        dx = target_pos['x'] - current_pos['x']
        dy = target_pos['y'] - current_pos['y']
        return math.sqrt(dx*dx + dy*dy)

    def calculate_angle_diff(self, current_theta, target_theta):
        """计算角度差，返回最短旋转角度"""
        diff = target_theta - current_theta
        # 将角度差限制在[-π, π]范围内
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def move_to_target(self, target_pos, position_tolerance=0.05, angle_tolerance=0.03, allow_backward=False):
        """改进的移动到目标位置方法"""
        print(f"开始移动到目标位置: x:{target_pos['x']:.2f} y:{target_pos['y']:.2f} theta:{target_pos['theta']:.2f}")
        
        step1_reached = False
        step2_reached = False
        
        while self.running and not rospy.is_shutdown():
            current_pos = self.spark_uarm_position.copy()
            
            # 计算距离和角度差
            distance = self.calculate_distance(current_pos, target_pos)
            
            #print(f"当前位置: x:{current_pos['x']:.2f} y:{current_pos['y']:.2f} theta:{current_pos['theta']:.2f}")
            #print(f"距离目标: {distance:.2f}m")
            
            vel_msg = Twist()
            
            # Step 1: 移动到目标位置附近
            if not step1_reached:
                if distance > position_tolerance:
                    vel_msg.linear.x = self.move_linear_vel(target_pos, current_pos)
                    angular_vel = self.move_angular_vel(target_pos, current_pos)
                    # 进一步限制角速度
                    vel_msg.angular.z = max(min(angular_vel, 3), -3)
                    #print(f"Step1 移动中: linear={vel_msg.linear.x:.2f}, angular={vel_msg.angular.z:.2f}")
                else:
                    step1_reached = True
                    print("Step1 完成：到达目标位置")
            
            # Step 2: 精确调整角度
            else:
                angle_dist, angular_vel = self.get_angular_control(target_pos['theta'], current_pos)
                vel_msg.linear.x = 0
                vel_msg.angular.z = angular_vel
                #print(f"Step2 角度调整: 角度差={angle_dist:.2f}, 角速度={angular_vel:.2f}")
                
                if abs(angle_dist) < angle_tolerance:
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = 0
                    step2_reached = True
                    print("Step2 完成：角度调整完成")
            
            if step2_reached:
                print("已到达目标位置")
                self.stop_robot()
                break
                
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()

    def move_linear_vel(self, goal_pose, current_pos, constant=0.8):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return max(min(constant * self.calculate_distance(current_pos, goal_pose), 0.1), -0.1)

    def move_angular_vel(self, goal_pose, self_pose, constant=1):
        def dotproduct(v1, v2):
            return sum((a*b) for a, b in zip(v1, v2))
        def length(v):
            return math.sqrt(dotproduct(v, v))
        def angle(v1, v2):
            return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

        def crossproduct(v1,v2):
            x1, y1 =v1
            x2, y2 =v2
            return x1*y2 -x2*y1
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        v1 = [math.cos(self_pose['theta']), math.sin(self_pose['theta'])]
        v2 = [goal_pose['x'] - self_pose['x'], goal_pose['y'] - self_pose['y']]
        return max(min(constant * math.copysign(angle(v1,v2),crossproduct(v1,v2)), 3), -3)

    def get_angular_control(self, target_theta, current_pose):
        """精确角度控制"""
        angle_diff = self.calculate_angle_diff(current_pose['theta'], target_theta)
        
        max_angular = 0.2
        min_angular = 0.02
        
        if abs(angle_diff) > 0.3:
            angular_vel = max_angular if angle_diff > 0 else -max_angular
        elif abs(angle_diff) > 0.1:
            angular_vel = max_angular * 0.5 if angle_diff > 0 else -max_angular * 0.5
        else:
            angular_vel = max(min_angular, abs(angle_diff) * 0.8)
            angular_vel = angular_vel if angle_diff > 0 else -angular_vel
        
        return angle_diff, angular_vel

    def execute_scissor_action(self):
        """执行手机(cell.phone)抓取动作"""
        try:
            print("=== 开始执行手机抓取动作 ===")
            
            # 获取手机目标位置
            target_pos = self.target_positions['cell.phone']
            
            # 直接移动到目标位置
            self.move_to_target(target_pos)
            
            # 到达目标位置后，等待2秒再通知serve可以查找物品
            time.sleep(5)
            client_socket.sendto(f"ready:cell.phone".encode(), server_address)
            self.waiting_for_confirmation = True
            self.current_action = "cell.phone"
            print("\n已到达目标点，等待服务端发送物品位置确认...")
            
            # 等待带坐标消息
            while self.waiting_for_confirmation and self.running:
                time.sleep(0.1)
            
            if not self.running:
                return
                
            # 到达目标位置后执行机械臂动作
            print("\n开始机械臂抓取动作")
            self.swift.connect()
            time.sleep(1)

            self.swift.set_position(x=210, y=0, z=100, wait=True)
            print("机械臂准备位置:", self.swift.get_position())
            time.sleep(0.5)
            
            # 机械臂抓取动作
            if self.received_coordinates:
                px, py = self.received_coordinates
                x, y = map_delta_to_xy(px, py)
                self.swift.set_position(x=x, y=y, z=-115, wait=True)

            print("到达抓取位置:", self.swift.get_position())
            time.sleep(0.5)

            self.swift.set_pump(on=True)
            time.sleep(1)

            self.swift.set_position(x=210, y=-100, z=50, wait=True)
            print("抬升完成:", self.swift.get_position())
            time.sleep(0.5)
            
            # 直接移动到旋转位置
            print("\n移动到旋转位置")
            rotation_position = {'x': 0.74, 'y': -5.87, 'theta': 0.59}
            self.move_to_target(rotation_position)
            
            # 在旋转位置放置物品
            print("\n在旋转位置放置物品")
            self.swift.set_position(x=250, y=-150, z=-55, wait=True)
            print("下降到放置位置:", self.swift.get_position())
            time.sleep(0.5)

            self.swift.set_pump(on=False)
            time.sleep(0.5)
            self.swift.set_pump(on=False, timeout=1)  # 明确吹气
            time.sleep(3)

            # 旋转与气泵相连的舵机甩下物品
            self.swift.set_servo_angle(servo_id=3, angle=90, wait=True)  # 旋转90度
            print("气泵舵机旋转90度甩物品")
            time.sleep(0.5)
            self.swift.set_servo_angle(servo_id=3, angle=0, wait=True)   # 回正
            print("气泵舵机回正")
            time.sleep(0.5)

            self.swift.set_position(x=210, y=0, z=100, wait=True)
            print("机械臂回到准备位置:", self.swift.get_position())
            time.sleep(0.5)

            self.swift.disconnect()
            
            # 直接回到home位置
            print("\n直接回到home位置")
            self.move_to_target(self.home_position)
            
            print("\n=== 手机抓取动作执行完成 ===")
            
        except Exception as e:
            print(f"执行过程中发生错误: {e}")
        finally:
            self.stop_robot()

    def execute_cup_action(self):
        """执行杯子抓取动作"""
        try:
            print("=== 开始执行杯子抓取动作 ===")
            
            # 定义中转点
            waypoint = {'x': 0.80, 'y': -5.16, 'theta': 1.51}
            return_waypoint = {'x': 0.76, 'y': -5.00, 'theta': -1.59}
            
            # 先移动到中转点
            print("\n移动到中转点")
            self.move_to_target(waypoint)
            
            # 获取杯子目标位置
            target_pos = self.target_positions['cup']
            
            # 从中转点移动到目标位置
            self.move_to_target(target_pos)
            
            # 到达目标位置后，等待2秒再通知serve可以查找物品
            time.sleep(5)
            client_socket.sendto(f"ready:cup".encode(), server_address)
            self.waiting_for_confirmation = True
            self.current_action = "cup"
            print("\n已到达目标点，等待服务端发送物品位置确认...")
            
            # 等待带坐标消息
            while self.waiting_for_confirmation and self.running:
                time.sleep(0.1)
            
            if not self.running:
                return
                
            # 到达目标位置后执行机械臂动作
            print("\n开始机械臂抓取动作")
            self.swift.connect()
            time.sleep(1)

            self.swift.set_position(x=210, y=0, z=100, wait=True)
            print("机械臂准备位置:", self.swift.get_position())
            time.sleep(0.5)
            
            # 机械臂抓取动作
            if self.received_coordinates:
                px, py = self.received_coordinates
                x, y = map_delta_to_xy(px, py)
                self.swift.set_position(x=x, y=y, z=-65, wait=True)
            print("到达抓取位置:", self.swift.get_position())
            time.sleep(0.5)

            self.swift.set_pump(on=True)
            time.sleep(1)

            self.swift.set_position(x=210, y=-100, z=50, wait=True)
            print("抬升完成:", self.swift.get_position())
            time.sleep(0.5)
            
            # 第一步：先回到返回中转点
            print("\n第一步：返回中转点")
            self.move_to_target(return_waypoint)
            
            # 第二步：移动到旋转位置
            print("\n第二步：移动到旋转位置")
            rotation_position = {'x': 0.74, 'y': -5.87, 'theta': 0.59}
            self.move_to_target(rotation_position)
            
            # 第三步：在旋转位置放置物品
            print("\n第三步：在旋转位置放置物品")
            self.swift.set_position(x=250, y=-150, z=0, wait=True)
            print("下降到放置位置:", self.swift.get_position())
            time.sleep(0.5)

            self.swift.set_pump(on=False)
            time.sleep(0.5)
            self.swift.set_pump(on=False, timeout=1)  # 明确吹气
            time.sleep(3)

            # 旋转与气泵相连的舵机甩下物品
            self.swift.set_servo_angle(servo_id=3, angle=90, wait=True)  # 旋转90度
            print("气泵舵机旋转90度甩物品")
            time.sleep(0.5)
            self.swift.set_servo_angle(servo_id=3, angle=0, wait=True)   # 回正
            print("气泵舵机回正")
            time.sleep(0.5)

            self.swift.set_position(x=210, y=0, z=100, wait=True)
            print("机械臂回到准备位置:", self.swift.get_position())
            time.sleep(0.5)

            self.swift.disconnect()
            
            # 直接回到home位置，不经过中转点
            print("\n最后回到home位置")
            self.move_to_target(self.home_position)
            
            print("\n=== 杯子抓取动作执行完成 ===")
            
        except Exception as e:
            print(f"执行过程中发生错误: {e}")
        finally:
            self.stop_robot()

    def execute_mouse_action(self):
        """执行鼠标抓取动作"""
        try:
            print("=== 开始执行鼠标抓取动作 ===")
            
            # 定义中转点
            waypoint = {'x': 0.80, 'y': -5.16, 'theta': 1.51}
            return_waypoint = {'x': 0.76, 'y': -5.00, 'theta': -1.59}
            
            # 先移动到中转点
            print("\n移动到中转点")
            self.move_to_target(waypoint)
            
            # 获取鼠标目标位置
            target_pos = self.target_positions['mouse']
            print("mouse:",target_pos)
            
            # 从中转点移动到目标位置
            self.move_to_target(target_pos)
            
            # 到达目标位置后，等待2秒再通知serve可以查找物品
            time.sleep(5)
            client_socket.sendto(f"ready:mouse".encode(), server_address)
            self.waiting_for_confirmation = True
            self.current_action = "mouse"
            print("\n已到达目标点，等待服务端发送物品位置确认...")
            
            # 等待带坐标消息
            while self.waiting_for_confirmation and self.running:
                time.sleep(0.1)
            
            if not self.running:
                return
                
            # 到达目标位置后执行机械臂动作
            print("\n开始机械臂抓取动作")
            self.swift.connect()
            time.sleep(1)

            self.swift.set_position(x=210, y=0, z=100, wait=True)
            print("机械臂准备位置:", self.swift.get_position())
            time.sleep(0.5)
            
            # 机械臂抓取动作
            if self.received_coordinates:
                px, py = self.received_coordinates
                x, y = map_delta_to_xy(px, py)
                self.swift.set_position(x=x, y=y, z=-115, wait=True)
            print("到达抓取位置:", self.swift.get_position())
            time.sleep(0.5)

            self.swift.set_pump(on=True)
            time.sleep(1)

            self.swift.set_position(x=210, y=-100, z=50, wait=True)
            print("抬升完成:", self.swift.get_position())
            time.sleep(0.5)
            
            # 第一步：先回到返回中转点
            print("\n第一步：返回中转点")
            self.move_to_target(return_waypoint)
            
            # 第二步：移动到旋转位置
            print("\n第二步：移动到旋转位置")
            rotation_position = {'x': 0.74, 'y': -5.87, 'theta': 0.59}
            self.move_to_target(rotation_position)
            
            # 第三步：在旋转位置放置物品
            print("\n第三步：在旋转位置放置物品")
            self.swift.set_position(x=250, y=-150, z=-55, wait=True)
            print("下降到放置位置:", self.swift.get_position())
            time.sleep(0.5)

            self.swift.set_pump(on=False)
            time.sleep(0.5)
            self.swift.set_pump(on=False, timeout=1)  # 明确吹气
            time.sleep(1)
            
            # 旋转与气泵相连的舵机甩下物品
            self.swift.set_servo_angle(servo_id=3, angle=90, wait=True)  # 旋转90度
            print("气泵舵机旋转90度甩物品")
            time.sleep(0.5)
            self.swift.set_servo_angle(servo_id=3, angle=0, wait=True)   # 回正
            print("气泵舵机回正")
            time.sleep(0.5)

            self.swift.set_position(x=210, y=0, z=100, wait=True)
            print("机械臂回到准备位置:", self.swift.get_position())
            time.sleep(0.5)

            self.swift.disconnect()
            
            # 直接回到home位置，不经过中转点
            print("\n最后回到home位置")
            self.move_to_target(self.home_position)
            
            print("\n=== 鼠标抓取动作执行完成 ===")
            
        except Exception as e:
            print(f"执行过程中发生错误: {e}")
        finally:
            self.stop_robot()

def linear_map(val, src_min, src_max, dst_min, dst_max):
    if src_max == src_min:
        return (dst_min + dst_max) / 2
    return dst_min + (val - src_min) * (dst_max - dst_min) / (src_max - src_min)

def map_delta_to_xy(dy, dx):
    x = 426 - 0.4761904762 * dx
    y = 143.7 - 0.3666666667 * dy
    print(f"map_delta_to_xy: dy={dy}, dx={dx} => x={x}, y={y}")
    return x, y

# 创建UDP客户端连接spark_serve
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ("192.168.1.26", 9090)

# 创建UDP客户端连接spark_uarm
uarm_client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
uarm_server_address = ("192.168.1.73", 9091)

# 主动发送消息给两个服务端
client_socket.sendto("Hello, UDP Server!".encode(), server_address)
uarm_client_socket.sendto("Hello, Spark UArm Server!".encode(), uarm_server_address)
print("等待服务端指令...")
print("已连接到Spark UArm服务器...")

# 创建机器人控制实例
take_user = TakeUser()

# 注册信号处理器
signal.signal(signal.SIGINT, take_user.signal_handler)
signal.signal(signal.SIGTERM, take_user.signal_handler)

def uarm_position_listener():
    """监听spark_uarm位置信息的线程"""
    while take_user.running:
        try:
            uarm_client_socket.settimeout(1.0)
            data, _ = uarm_client_socket.recvfrom(1024)
            position_data = json.loads(data.decode())
            take_user.spark_uarm_position = position_data
            #print(f"[UArm位置] x:{position_data['x']:.2f} y:{position_data['y']:.2f} theta:{position_data['theta']:.2f}")
        except socket.timeout:
            continue
        except Exception as e:
            if take_user.running:
                print(f"接收UArm位置数据错误: {e}")
            break

# 启动UArm位置监听线程
uarm_thread = threading.Thread(target=uarm_position_listener, daemon=True)
uarm_thread.start()

def udp_message_listener():
    """监听UDP消息的线程"""
    while take_user.running:
        try:
            client_socket.settimeout(1.0)
            data, _ = client_socket.recvfrom(1024)
            msg = data.decode().strip()
            print(f"收到服务端消息: {msg}")
            
            if not take_user.running:
                break
            
            # 解析消息格式，支持带坐标的指令
            object_name = None
            coordinates = None
            
            if ": Center(" in msg:
                # 解析格式: "mouse: Center(1057, 102)"
                parts = msg.split(": Center(")
                if len(parts) == 2:
                    object_name = parts[0]
                    coord_str = parts[1].rstrip(")")
                    try:
                        x, y = map(int, coord_str.split(", "))
                        coordinates = (x, y)
                        print(f"收到{object_name}位置信息：({x}, {y})，开始抓取")
                        
                        # 如果正在等待确认且物品匹配，则继续执行
                        if take_user.waiting_for_confirmation and take_user.current_action == object_name:
                            take_user.received_coordinates = coordinates
                            take_user.waiting_for_confirmation = False
                            print(f"✓ 收到服务端确认，{object_name}位置：({x}, {y})，继续执行抓取")
                        elif not take_user.waiting_for_confirmation:
                            # 如果没有在等待确认，直接启动对应的抓取动作
                            if object_name == "cup":
                                print("执行：抓取杯子的动作！")
                                threading.Thread(target=take_user.execute_cup_action, daemon=True).start()
                            elif object_name == "cell.phone":
                                print("执行：抓取手机(cell.phone)的动作！")
                                threading.Thread(target=take_user.execute_scissor_action, daemon=True).start()
                            elif object_name == "mouse":
                                print("执行：抓取鼠标的动作！")
                                threading.Thread(target=take_user.execute_mouse_action, daemon=True).start()
                        
                    except ValueError:
                        print(f"坐标解析失败: {coord_str}")
                        object_name = parts[0]  # 仍然尝试执行动作
            else:
                # 普通格式，只有物品名称
                object_name = msg
                print(f"收到{object_name}指令，开始抓取")
                
                if object_name == "cup":
                    print("执行：抓取杯子的动作！")
                    threading.Thread(target=take_user.execute_cup_action, daemon=True).start()
                elif object_name == "cell.phone":
                    print("执行：抓取手机(cell.phone)的动作！")
                    threading.Thread(target=take_user.execute_scissor_action, daemon=True).start()
                elif object_name == "mouse":
                    print("执行：抓取鼠标的动作！")
                    threading.Thread(target=take_user.execute_mouse_action, daemon=True).start()
                else:
                    print("收到未知指令，忽略。")
            
        except socket.timeout:
            continue
        except Exception as e:
            if take_user.running:
                print(f"接收消息时发生错误: {e}")
            break

# 启动UDP消息监听线程
udp_thread = threading.Thread(target=udp_message_listener, daemon=True)
udp_thread.start()

try:
    while take_user.running:
        time.sleep(0.1)  # 主循环只需要保持运行状态
        
except KeyboardInterrupt:
    print("程序被中断。")
finally:
    take_user.running = False
    take_user.stop_robot()
    try:
        take_user.swift.disconnect()
    except:
        pass
    client_socket.close()
    uarm_client_socket.close()
    print("客户端已安全退出。")
