#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus, GoalID
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
import sys
import traceback

# 导入ROS消息和服务类型
try:
    import common.msg
    import common.srv
    from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
except ImportError as e:
    rospy.logerr(f"导入错误: {e}")
    rospy.logerr("这可能是因为ROS包路径未正确设置，或者消息类型未正确编译")
    sys.exit(1)


class RobotNavigator:
    def __init__(self):
        """Initialize the robot navigation class with all required clients and publishers"""
        # Initialize ROS node if not already initialized
        if not rospy.core.is_initialized():
            try:
                rospy.init_node('robot_navigator', anonymous=True)
            except Exception as e:
                rospy.logerr(f"初始化节点失败: {e}")
                raise

        try:
            # Create action client for controlling linear movement
            rospy.loginfo("正在连接 move_straight ActionServer...")
            self.move_action_cli = actionlib.SimpleActionClient(
                'move_straight', MoveStraightDistanceAction)
            server_exists = self.move_action_cli.wait_for_server(
                timeout=rospy.Duration.from_sec(3))
            
            if not server_exists:
                rospy.logwarn("无法连接 move_straight ActionServer，部分功能可能不可用")
            
            # Create action client for controlling rotation
            rospy.loginfo("正在连接 turn_body ActionServer...")
            self.turn_action_cli = actionlib.SimpleActionClient(
                'turn_body', TurnBodyDegreeAction)
            server_exists = self.turn_action_cli.wait_for_server(
                timeout=rospy.Duration.from_sec(3))
            
            if not server_exists:
                rospy.logwarn("无法连接 turn_body ActionServer，部分功能可能不可用")
    
            # Create publisher for direct velocity commands
            self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    
            # Create service client for distance measurements
            rospy.loginfo("正在连接 get_distance Service...")
            try:
                rospy.wait_for_service('/get_distance', timeout=3.0)
                self.distance_srv = rospy.ServiceProxy(
                    'get_distance', common.srv.GetFrontBackDistance)
            except (rospy.ROSException, rospy.ServiceException) as e:
                rospy.logwarn(f"无法连接 get_distance 服务: {e}")
                rospy.logwarn("adjust_by_tag 功能将不可用")
    
            # Create publisher for navigation goals
            self.goto_local_pub = rospy.Publisher(
                "mark_nav", String, queue_size=1)
            
            # 创建导航命令订阅者
            self.navigation_sub = rospy.Subscriber(
                "navigation_command", String, self.navigation_command_callback)
            
            # 是否正在导航的标志
            self.is_navigating = False
            
            rospy.loginfo("Robot navigator initialized and ready")
            
        except Exception as e:
            rospy.logerr(f"初始化 RobotNavigator 失败: {e}")
            rospy.logerr(traceback.format_exc())
            raise

    def navigation_command_callback(self, msg):
        """
        处理导航命令
        
        Args:
            msg (String): 导航命令，格式为 "goto location_name"
        """
        try:
            command = msg.data.strip()
            if command.startswith("goto "):
                location = command[5:]  # 提取位置名称
                rospy.loginfo(f"收到导航命令: 前往 {location}")
                self.goto_local(location)
            else:
                rospy.logwarn(f"未知导航命令: {command}")
        except Exception as e:
            rospy.logerr(f"处理导航命令错误: {e}")

    def goto_local(self, name):
        """
        Navigate to a predefined location using the move_base server
        
        Args:
            name (str): The name of the target location
            
        Returns:
            bool: True if successfully reached the target location, False otherwise
        """
        try:
            # 如果已经在导航中，则忽略此命令
            if self.is_navigating:
                rospy.logwarn("已有导航任务正在执行，忽略此命令")
                return False
                
            self.is_navigating = True
            
            # Publish target location
            self.goto_local_pub.publish(f"go {name}")
            rospy.loginfo(f"正在导航到位置: {name}")
    
            # Wait for result with 1-minute timeout
            try:
                rospy.loginfo("等待导航结果...")
                ret_status = rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
            except rospy.ROSException:
                rospy.logwarn("导航超时!")
                ret_status = GoalStatus.ABORTED
                self.is_navigating = False
                return False
    
            # If goal not reached within timeout, cancel the goal
            if ret_status != GoalStatus.SUCCEEDED:
                rospy.loginfo("目标未达成，正在取消导航目标...")
                cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)
                cancel_pub.publish(GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
                try:
                    rospy.wait_for_message(
                        'move_base/result', MoveBaseActionResult, rospy.Duration(3))
                except rospy.ROSException:
                    rospy.logwarn("move_base result timeout. This is abnormal.")
                rospy.loginfo("==========导航未能到达目标==========")
                self.is_navigating = False
                return False
            else:
                rospy.loginfo("==========导航成功到达目标==========")
                self.is_navigating = False
                return True
        except Exception as e:
            rospy.logerr(f"导航过程发生错误: {e}")
            rospy.logerr(traceback.format_exc())
            self.is_navigating = False
            return False

    def adjust_by_tag(self):
        """
        Adjust robot position in front of a desk using positioning tag
        
        Returns:
            bool: True if adjustment is successful, False otherwise
        """
        try:
            # Adjust the robot's distance to the desk, target distance is 0.25m
            walk_distance = self.distance_srv(None).front_distance - 0.25
            self.move_action_cli.send_goal_and_wait(
                common.msg.MoveStraightDistanceGoal(
                    type=common.msg.MoveStraightDistanceGoal.TYPE_SCAN,
                    const_rot_vel=0.1,
                    move_distance=walk_distance,
                ),
                rospy.Duration.from_sec(5)  # 5s timeout
            )
            return True
        except Exception as e:
            rospy.logerr(f"调整位置时发生错误: {e}")
            return False
    
    def step_back(self, distance=0.2):
        """
        Move the robot backward a specified distance
        
        Args:
            distance (float): Distance to move backward in meters
            
        Returns:
            bool: True if movement is successful, False otherwise
        """
        try:
            self.move_action_cli.send_goal_and_wait(
                common.msg.MoveStraightDistanceGoal(
                    type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                    const_rot_vel=-0.1,
                    move_distance=distance,
                ),
                rospy.Duration.from_sec(5)  # 5s timeout
            )
            return True
        except Exception as e:
            rospy.logerr(f"后退时发生错误: {e}")
            return False
    
    def step_go(self, distance):
        """
        Move the robot forward a specified distance
        
        Args:
            distance (float): Distance to move forward in meters
            
        Returns:
            bool: True if movement is successful, False otherwise
        """
        try:
            self.move_action_cli.send_goal_and_wait(
                common.msg.MoveStraightDistanceGoal(
                    type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                    const_rot_vel=0.1,
                    move_distance=distance,
                ),
                rospy.Duration.from_sec(5)  # 5s timeout
            )
            return True
        except Exception as e:
            rospy.logerr(f"前进时发生错误: {e}")
            return False

    def step_rotate(self, rad):
        """
        Rotate the robot at a specified angular velocity
        
        Args:
            rad (float): Angular velocity in radians/second
        """
        try:
            twist = Twist()
            twist.angular.z = rad
            self.cmd_pub.publish(twist)
        except Exception as e:
            rospy.logerr(f"旋转时发生错误: {e}")

    def step_rotate_pro(self, rad, time=0.5):
        """
        Rotate the robot for a specified time and then stop
        
        Args:
            rad (float): Angular velocity in radians/second
            time (float): Duration of rotation in seconds
        """
        try:
            twist = Twist()
            twist.angular.z = rad
            self.cmd_pub.publish(twist)
            rospy.sleep(time)
            twist.angular.z = 0
            self.cmd_pub.publish(twist)
        except Exception as e:
            rospy.logerr(f"定时旋转时发生错误: {e}")

    def step_go_pro(self, linear, time=0.3):
        """
        Move the robot at a specified linear velocity for a time and then stop
        
        Args:
            linear (float): Linear velocity in meters/second
            time (float): Duration of movement in seconds
        """
        try:
            twist_go = Twist()
            twist_go.linear.x = linear
            self.cmd_pub.publish(twist_go)
            rospy.sleep(time)
            twist_go.linear.x = 0
            self.cmd_pub.publish(twist_go)
        except Exception as e:
            rospy.logerr(f"定时移动时发生错误: {e}")


if __name__ == '__main__':
    try:
        # Simple test of the navigation functionality
        navigator = RobotNavigator()
        rospy.loginfo("RobotNavigator initialized. Ready for commands.")
        rospy.loginfo("请发布导航命令到 /navigation_command 话题，格式: 'goto 位置名'")
        
        # 等待导航命令
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node terminated.")
    except Exception as e:
        rospy.logerr(f"运行错误: {e}")
        rospy.logerr(traceback.format_exc()) 