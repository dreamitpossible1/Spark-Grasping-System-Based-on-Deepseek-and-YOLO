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
        rospy.init_node("take_user_node", anonymous=True) 
        self.vel_pub = rospy.Publisher("/robo5/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10) 
        self.swift = SwiftAPI(port='/dev/ttyACM0')
        self.running = True   
        TARGET_THETA = 1.57
        self.home_position = {'x': 0.79, 'y': -5.87, 'theta': TARGET_THETA} 
        self.spark_target_positions = {
            'cell.phone': {'x': 0.74, 'y': -4.80, 'theta': TARGET_THETA},
            'cup': {'x': 1.30, 'y': -4.89, 'theta': TARGET_THETA},
            'mouse': {'x': 0.20, 'y': -4.82, 'theta': TARGET_THETA}
        }       
        self.waiting_for_confirmation = False
        self.current_action = None
        self.received_coordinates = None
    
    def signal_handler(self, signum, frame):
        self.running = False
        self.stop_robot()
        try:
            if hasattr(self.swift, 'disconnect'):
                self.swift.disconnect()
        except:
            pass
        sys.exit(0)
    
    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)
        print("The robot has stopped")

    def calculate_distance(self, current_pos, target_pos):
        dx = target_pos['x'] - current_pos['x']
        dy = target_pos['y'] - current_pos['y']
        return math.sqrt(dx*dx + dy*dy)

    def calculate_angle_diff(self, current_theta, target_theta):
        diff = target_theta - current_theta
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def move_to_target(self, target_pos, position_tolerance=0.05, angle_tolerance=0.03, allow_backward=False): 
        print(f"goal position: x:{target_pos['x']:.2f} y:{target_pos['y']:.2f} theta:{target_pos['theta']:.2f}")
        step1_reached = False
        step2_reached = False
        
        while self.running and not rospy.is_shutdown():
            current_pos = self.spark_uarm_position.copy()
            distance = self.calculate_distance(current_pos, target_pos) 
            vel_msg = Twist()
            if not step1_reached:
                if distance > position_tolerance:
                    vel_msg.linear.x = self.move_linear_vel(target_pos, current_pos)
                    angular_vel = self.move_angular_vel(target_pos, current_pos)
                    vel_msg.angular.z = max(min(angular_vel, 3), -3)
                else:
                    step1_reached = True 
            else:
                angle_dist, angular_vel = self.get_angular_control(target_pos['theta'], current_pos)
                vel_msg.linear.x = 0
                vel_msg.angular.z = angular_vel
                if abs(angle_dist) < angle_tolerance:
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = 0
                    step2_reached = True
            if step2_reached:
                print("Arrived at the target location")
                self.stop_robot()
                break
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()

    def move_linear_vel(self, goal_pose, current_pos, constant=0.8):        
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
        v1 = [math.cos(self_pose['theta']), math.sin(self_pose['theta'])]
        v2 = [goal_pose['x'] - self_pose['x'], goal_pose['y'] - self_pose['y']]
        return max(min(constant * math.copysign(angle(v1,v2),crossproduct(v1,v2)), 3), -3)

    def get_angular_control(self, target_theta, current_pose):       
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
        try:           
            target_pos = self.spark_target_positions['cell.phone'] 
            self.move_to_target(target_pos)           
            time.sleep(5)
            client_socket.sendto(f"ready:cell.phone".encode(), server_address)
            self.waiting_for_confirmation = True
            self.current_action = "cell.phone" 
            while self.waiting_for_confirmation and self.running:
                time.sleep(0.1)
            if not self.running:
                return           
            self.swift.connect()
            time.sleep(1)
            self.swift.set_position(x=210, y=0, z=100, wait=True)            
            time.sleep(0.5)       
            if self.received_coordinates:
                px, py = self.received_coordinates
                x, y = map_delta_to_xy(px, py)
                self.swift.set_position(x=x, y=y, z=-115, wait=True)
            time.sleep(0.5)
            self.swift.set_pump(on=True)
            time.sleep(1)
            self.swift.set_position(x=210, y=-100, z=50, wait=True)
            time.sleep(0.5)
            rotation_position = {'x': 0.74, 'y': -5.87, 'theta': 0.59}
            self.move_to_target(rotation_position)
            self.swift.set_position(x=250, y=-150, z=-55, wait=True)
            time.sleep(0.5)
            self.swift.set_pump(on=False)
            time.sleep(0.5)
            self.swift.set_pump(on=False, timeout=1)  
            time.sleep(3)
            self.swift.set_servo_angle(servo_id=3, angle=90, wait=True)  
            time.sleep(0.5)
            self.swift.set_servo_angle(servo_id=3, angle=0, wait=True)  
            time.sleep(0.5)
            self.swift.set_position(x=210, y=0, z=100, wait=True)
            time.sleep(0.5)
            self.swift.disconnect()
            self.move_to_target(self.home_position)
            print("\n=== completed ===")
            
        except Exception as e:
            print(f"error: {e}")
        finally:
            self.stop_robot()

    def execute_cup_action(self):
        try:
            waypoint = {'x': 0.80, 'y': -5.16, 'theta': 1.51}
            return_waypoint = {'x': 0.76, 'y': -5.00, 'theta': -1.59}
            self.move_to_target(waypoint)
            target_pos = self.spark_target_positions['cup']
            self.move_to_target(target_pos)
            time.sleep(5)
            client_socket.sendto(f"ready:cup".encode(), server_address)
            self.waiting_for_confirmation = True
            self.current_action = "cup"
            while self.waiting_for_confirmation and self.running:
                time.sleep(0.1)
            if not self.running:
                return
            self.swift.connect()
            time.sleep(1)
            self.swift.set_position(x=210, y=0, z=100, wait=True)
            time.sleep(0.5)
            if self.received_coordinates:
                px, py = self.received_coordinates
                x, y = map_delta_to_xy(px, py)
                self.swift.set_position(x=x, y=y, z=-65, wait=True)
            time.sleep(0.5)
            self.swift.set_pump(on=True)
            time.sleep(1)
            self.swift.set_position(x=210, y=-100, z=50, wait=True)
            time.sleep(0.5)
            self.move_to_target(return_waypoint)
            rotation_position = {'x': 0.74, 'y': -5.87, 'theta': 0.59}
            self.move_to_target(rotation_position)
            self.swift.set_position(x=250, y=-150, z=0, wait=True)
            time.sleep(0.5)
            self.swift.set_pump(on=False)
            time.sleep(0.5)
            self.swift.set_pump(on=False, timeout=1)
            time.sleep(3)
            self.swift.set_servo_angle(servo_id=3, angle=90, wait=True) 
            time.sleep(0.5)
            self.swift.set_servo_angle(servo_id=3, angle=0, wait=True) 
            time.sleep(0.5)
            self.swift.set_position(x=210, y=0, z=100, wait=True)
            time.sleep(0.5)
            self.swift.disconnect()
            self.move_to_target(self.home_position)
            
        except Exception as e:
            print(f"error: {e}")
        finally:
            self.stop_robot()

    def execute_mouse_action(self):
        try:
            waypoint = {'x': 0.80, 'y': -5.16, 'theta': 1.51}
            return_waypoint = {'x': 0.76, 'y': -5.00, 'theta': -1.59}
            self.move_to_target(waypoint)
            target_pos = self.spark_target_positions['mouse']
            self.move_to_target(target_pos)
            time.sleep(5)
            client_socket.sendto(f"ready:mouse".encode(), server_address)
            self.waiting_for_confirmation = True
            self.current_action = "mouse"
            while self.waiting_for_confirmation and self.running:
                time.sleep(0.1)
            if not self.running:
                return
            self.swift.connect()
            time.sleep(1)
            self.swift.set_position(x=210, y=0, z=100, wait=True)
            time.sleep(0.5)
            if self.received_coordinates:
                px, py = self.received_coordinates
                x, y = map_delta_to_xy(px, py)
                self.swift.set_position(x=x, y=y, z=-115, wait=True)
            time.sleep(0.5)
            self.swift.set_pump(on=True)
            time.sleep(1)
            self.swift.set_position(x=210, y=-100, z=50, wait=True)
            time.sleep(0.5)
            self.move_to_target(return_waypoint)
            rotation_position = {'x': 0.74, 'y': -5.87, 'theta': 0.59}
            self.move_to_target(rotation_position)
            self.swift.set_position(x=250, y=-150, z=-55, wait=True)
            time.sleep(0.5)
            self.swift.set_pump(on=False)
            time.sleep(0.5)
            self.swift.set_pump(on=False, timeout=1) 
            time.sleep(1)
            self.swift.set_servo_angle(servo_id=3, angle=90, wait=True) 
            time.sleep(0.5)
            self.swift.set_servo_angle(servo_id=3, angle=0, wait=True) 
            time.sleep(0.5)
            self.swift.set_position(x=210, y=0, z=100, wait=True)
            time.sleep(0.5)
            self.swift.disconnect()
            self.move_to_target(self.home_position)
            
        except Exception as e:
            print(f"erro: {e}")
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

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ("192.168.1.26", 9090)
uarm_client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
uarm_server_address = ("192.168.1.73", 9091)
client_socket.sendto("Hello, UDP Server!".encode(), server_address)
uarm_client_socket.sendto("Hello, Spark UArm Server!".encode(), uarm_server_address)
take_user = TakeUser()
signal.signal(signal.SIGINT, take_user.signal_handler)
signal.signal(signal.SIGTERM, take_user.signal_handler)

def uarm_position_listener():
    while take_user.running:
        try:
            uarm_client_socket.settimeout(1.0)
            data, _ = uarm_client_socket.recvfrom(1024)
            position_data = json.loads(data.decode())
            take_user.spark_uarm_position = position_data            
        except socket.timeout:
            continue
        except Exception as e:
            if take_user.running:
                print(f"接收UArm位置数据错误: {e}")
            break

uarm_thread = threading.Thread(target=uarm_position_listener, daemon=True)
uarm_thread.start()

def udp_message_listener():
    while take_user.running:
        try:
            client_socket.settimeout(1.0)
            data, _ = client_socket.recvfrom(1024)
            msg = data.decode().strip()
            
            if not take_user.running:
                break
            
            object_name = None
            coordinates = None
            if ": Center(" in msg:
                parts = msg.split(": Center(")
                if len(parts) == 2:
                    object_name = parts[0]
                    coord_str = parts[1].rstrip(")")
                    try:
                        x, y = map(int, coord_str.split(", "))
                        coordinates = (x, y)
                        if take_user.waiting_for_confirmation and take_user.current_action == object_name:
                            take_user.received_coordinates = coordinates
                            take_user.waiting_for_confirmation = False                           
                        elif not take_user.waiting_for_confirmation:
                            if object_name == "cup":
                                threading.Thread(target=take_user.execute_cup_action, daemon=True).start()
                            elif object_name == "cell.phone":
                                threading.Thread(target=take_user.execute_scissor_action, daemon=True).start()
                            elif object_name == "mouse":
                                threading.Thread(target=take_user.execute_mouse_action, daemon=True).start()
                        
                    except ValueError:
                        object_name = parts[0] 
            else:                
                object_name = msg               
                if object_name == "cup":
                    threading.Thread(target=take_user.execute_cup_action, daemon=True).start()
                elif object_name == "cell.phone":
                    threading.Thread(target=take_user.execute_scissor_action, daemon=True).start()
                elif object_name == "mouse":
                    threading.Thread(target=take_user.execute_mouse_action, daemon=True).start()
                else:
                    print("收到未知指令，忽略。")
            
        except socket.timeout:
            continue
        except Exception as e:
            if take_user.running:
                print(f"接收消息时发生错误: {e}")
            break

udp_thread = threading.Thread(target=udp_message_listener, daemon=True)
udp_thread.start()

try:
    while take_user.running:
        time.sleep(0.1)  
        
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

