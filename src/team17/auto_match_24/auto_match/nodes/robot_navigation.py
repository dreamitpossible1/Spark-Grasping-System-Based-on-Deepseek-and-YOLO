#!/usr/bin/python3

import rospy
import actionlib
from std_msgs.msg import String, GoalID
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
import common.msg
import common.srv
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction


class RobotNavigator:
    def __init__(self):
        """Initialize the robot navigation class with all required clients and publishers"""
        # Initialize ROS node if not already initialized
        if not rospy.core.is_initialized():
            rospy.init_node('robot_navigator', anonymous=True)

        # Create action client for controlling linear movement
        self.move_action_cli = actionlib.SimpleActionClient(
            'move_straight', MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # Create action client for controlling rotation
        self.turn_action_cli = actionlib.SimpleActionClient(
            'turn_body', TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # Create publisher for direct velocity commands
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Create service client for distance measurements
        rospy.wait_for_service('/get_distance')
        self.distance_srv = rospy.ServiceProxy(
            'get_distance', common.srv.GetFrontBackDistance)

        # Create publisher for navigation goals
        self.goto_local_pub = rospy.Publisher(
            "mark_nav", String, queue_size=1)
        
        rospy.loginfo("Robot navigator initialized and ready")

    def goto_local(self, name):
        """
        Navigate to a predefined location using the move_base server
        
        Args:
            name (str): The name of the target location
            
        Returns:
            bool: True if successfully reached the target location, False otherwise
        """
        # Publish target location
        self.goto_local_pub.publish("go " + name)
        rospy.loginfo(f"Navigating to location: {name}")

        # Wait for result with 1-minute timeout
        try:
            ret_status = rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("Navigation timeout!")
            ret_status = GoalStatus.ABORTED

        # If goal not reached within timeout, cancel the goal
        if ret_status != GoalStatus.SUCCEEDED:
            rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
                GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
            try:
                rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(3))
            except Exception:
                rospy.logwarn("move_base result timeout. This is abnormal.")
            rospy.loginfo("==========Timed out achieving goal==========")
            return False
        else:
            rospy.loginfo("==========Goal succeeded==========")
            return True

    def adjust_by_tag(self):
        """
        Adjust robot position in front of a desk using positioning tag
        
        Returns:
            bool: True if adjustment is successful, False otherwise
        """
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
    
    def step_back(self, distance=0.2):
        """
        Move the robot backward a specified distance
        
        Args:
            distance (float): Distance to move backward in meters
            
        Returns:
            bool: True if movement is successful, False otherwise
        """
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=-0.1,
                move_distance=distance,
            ),
            rospy.Duration.from_sec(5)  # 5s timeout
        )
        return True
    
    def step_go(self, distance):
        """
        Move the robot forward a specified distance
        
        Args:
            distance (float): Distance to move forward in meters
            
        Returns:
            bool: True if movement is successful, False otherwise
        """
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=0.1,
                move_distance=distance,
            ),
            rospy.Duration.from_sec(5)  # 5s timeout
        )
        return True

    def step_rotate(self, rad):
        """
        Rotate the robot at a specified angular velocity
        
        Args:
            rad (float): Angular velocity in radians/second
        """
        twist = Twist()
        twist.angular.z = rad
        self.cmd_pub.publish(twist)

    def step_rotate_pro(self, rad, time=0.5):
        """
        Rotate the robot for a specified time and then stop
        
        Args:
            rad (float): Angular velocity in radians/second
            time (float): Duration of rotation in seconds
        """
        twist = Twist()
        twist.angular.z = rad
        self.cmd_pub.publish(twist)
        rospy.sleep(time)
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

    def step_go_pro(self, linear, time=0.3):
        """
        Move the robot at a specified linear velocity for a time and then stop
        
        Args:
            linear (float): Linear velocity in meters/second
            time (float): Duration of movement in seconds
        """
        twist_go = Twist()
        twist_go.linear.x = linear
        self.cmd_pub.publish(twist_go)
        rospy.sleep(time)
        twist_go.linear.x = 0
        self.cmd_pub.publish(twist_go)


if __name__ == '__main__':
    try:
        # Simple test of the navigation functionality
        navigator = RobotNavigator()
        rospy.loginfo("RobotNavigator initialized. Ready for commands.")
        
        # Basic example usage (commented out)
        # navigator.goto_local("sorting_area")
        # navigator.step_go(0.3)
        # navigator.step_rotate_pro(0.5, 1.0)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node terminated.") 