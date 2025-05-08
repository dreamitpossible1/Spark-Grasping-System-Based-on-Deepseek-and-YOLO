#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Control Server - Runs in ROS Python 3.8 environment
This module handles all ROS-related operations and communicates with the LLM client
via TCP sockets.
"""
import rospy
import socket
import json
import threading
import time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from swiftpro.msg import SwiftproState
from swiftpro.msg import position
from swiftpro.msg import status

class RobotControlServer:
    """Server that handles robot control operations and communicates with LLM client"""
    
    def __init__(self, host='localhost', port=9999):
        self.host = host
        self.port = port
        self.node_name = 'robot_control_server'
        self.is_node_created = False
        self.end_pose = None
        self.is_get_end_pose = False
        self.joint_states = None
        self.is_get_joint_states = False
        self.running = False
        self.socket = None
        self.clients = []
        self.pump_status = 0  # 保存当前吸盘状态（0=off, 1=on）
        
        # Initialize ROS node and subscribers/publishers
        self.create_ros_node()
    
    def create_ros_node(self):
        """Initialize ROS node and create publishers and subscribers"""
        if not rospy.get_node_uri():
            rospy.init_node(self.node_name, anonymous=True)
        
        # Create publishers for position and pump control
        self.pub_position = rospy.Publisher('position_write_topic', position, queue_size=10)
        self.pub_pump = rospy.Publisher('pump_topic', status, queue_size=10)
        
        # Create subscribers for robot state
        self.sub_end_pose = rospy.Subscriber(
            'SwiftproState_topic',
            SwiftproState,
            self.end_pose_listener_callback,
            queue_size=10)
            
        self.sub_joint_states = rospy.Subscriber(
            '/joint_states',
            JointState,
            self.joint_states_listener_callback,
            queue_size=10)
        
        self.is_node_created = True
        rospy.loginfo(f"{self.node_name} initialized")
    
    def end_pose_listener_callback(self, msg):
        """Callback for robot end effector pose"""
        self.end_pose = [msg.x, msg.y, msg.z, 0.0, 0.0, 0.0, 1.0]  # Default orientation values
        self.pump_status = msg.pump  # 更新当前吸盘状态
        self.is_get_end_pose = True
    
    def joint_states_listener_callback(self, msg):
        """Callback for joint states"""
        self.joint_states = msg
        self.is_get_joint_states = True
    
    def get_end_pose(self):
        """Get the current end effector pose"""
        # Wait for end pose data
        timeout_counter = 0
        while not self.is_get_end_pose and timeout_counter < 50:
            rospy.sleep(0.1)
            timeout_counter += 1
        
        if not self.is_get_end_pose:
            rospy.logwarn("Failed to get end effector pose (timeout)")
            return None
        
        end_pose = self.end_pose[:]
        self.is_get_end_pose = False
        return end_pose
    
    def get_pump_state(self):
        """Get the current pump state (on/off)"""
        # Wait for end pose data which also contains pump status
        timeout_counter = 0
        while not self.is_get_end_pose and timeout_counter < 50:
            rospy.sleep(0.1)
            timeout_counter += 1
        
        if not self.is_get_end_pose:
            rospy.logwarn("Failed to get end effector pose with pump state (timeout)")
            return "unknown"
        
        return "on" if self.pump_status == 1 else "off"
    
    def execute_robot_command(self, command_data):
        """Execute a robot control command"""
        try:
            # Extract position data
            pos_msg = position()
            pos_msg.x = float(command_data['position.x'])
            pos_msg.y = float(command_data['position.y'])
            pos_msg.z = float(command_data['position.z'])
            pos_msg.speed = 1000  # Default speed

            rospy.loginfo(f"Preparing to move robot to position: x={pos_msg.x}, y={pos_msg.y}, z={pos_msg.z}, speed={pos_msg.speed}")
            
            # Store position values for response
            position_values = {
                "position.x": pos_msg.x,
                "position.y": pos_msg.y,
                "position.z": pos_msg.z
            }
            
            # Extract pump state (if provided)
            pump_state = None
            if 'pump_value' in command_data or 'pump_state' in command_data:
                pump_msg = status()
                
                # Check pump_value first (direct numeric value)
                if 'pump_value' in command_data:
                    pump_msg.status = int(command_data['pump_value'])
                    pump_state = "on" if pump_msg.status == 1 else "off"
                # Fall back to pump_state (string "on"/"off")
                elif 'pump_state' in command_data:
                    pump_state = command_data['pump_state'].lower()
                    pump_msg.status = 1 if pump_state == 'on' else 0
                
                # Send pump command
                self.pub_pump.publish(pump_msg)
                rospy.loginfo(f"Pump command sent: {pump_state}")
            
            # Debug output of message before publishing
            rospy.loginfo(f"Position message details - Type: {type(pos_msg).__name__}, Fields: x={pos_msg.x}, y={pos_msg.y}, z={pos_msg.z}, speed={pos_msg.speed}")
            
            # Send position command
            rospy.loginfo(f"Publishing position command to topic: position_write_topic")
            try:
                # Try publishing the message and log the result
                self.pub_position.publish(pos_msg)
                rospy.loginfo("Position message published successfully")
                
                # For debugging, also try publishing using the raw rostopic command format
                debug_cmd = f"rostopic pub -1 /position_write_topic swiftpro/position \"x: {pos_msg.x}\ny: {pos_msg.y}\nz: {pos_msg.z}\nspeed: {pos_msg.speed}\" &"
                rospy.loginfo(f"Equivalent rostopic command would be: {debug_cmd}")
                
            except Exception as pub_err:
                rospy.logerr(f"Error publishing position message: {str(pub_err)}")
            
            # Wait for execution (with timeout)
            wait_time = 3.0
            rospy.loginfo(f"Waiting {wait_time} seconds for robot movement to complete...")
            rospy.sleep(wait_time)
            
            # Try to get final position after movement
            final_pose = self.get_end_pose()
            if final_pose:
                rospy.loginfo(f"Final robot position after movement: x={final_pose[0]}, y={final_pose[1]}, z={final_pose[2]}")
                position_values = {
                    "position.x": final_pose[0],
                    "position.y": final_pose[1],
                    "position.z": final_pose[2]
                }
            
            rospy.loginfo(f"Command executed: position [{pos_msg.x}, {pos_msg.y}, {pos_msg.z}]")
            
            # Create response with position data and pump state if available
            response = {
                "status": "success",
                "message": "Command executed successfully",
                **position_values  # Include position values in response
            }
            
            # Add pump state to response if it was included in the command
            if pump_state:
                response["pump_state"] = pump_state
            
            return response
            
        except Exception as e:
            rospy.logerr(f"Error executing command: {str(e)}")
            return {"status": "error", "message": str(e)}
    
    def handle_client_connection(self, client_socket, client_address):
        """Handle communication with a connected client"""
        rospy.loginfo(f"New client connected: {client_address}")
        
        while self.running:
            try:
                # Receive data from client
                data = client_socket.recv(4096)
                if not data:
                    break
                
                # Parse request
                request = json.loads(data.decode('utf-8'))
                response = {"status": "error", "message": "Unknown command"}
                
                # Process request based on command type
                if request["command"] == "get_robot_state":
                    rospy.loginfo(f"Received get_robot_state request")
                    end_pose = self.get_end_pose()
                    pump_state = self.get_pump_state()
                    joint_states = self.get_joint_states()
                    
                    if end_pose:
                        rospy.loginfo(f"Retrieved end pose: {end_pose}")
                        response = {
                            "status": "success",
                            "position.x": end_pose[0],
                            "position.y": end_pose[1],
                            "position.z": end_pose[2],
                            "orientation.x": end_pose[3],
                            "orientation.y": end_pose[4],
                            "orientation.z": end_pose[5],
                            "orientation.w": end_pose[6],
                            "pump_state": pump_state
                        }
                        
                        # Add joint states if available
                        if joint_states:
                            rospy.loginfo(f"Retrieved joint states: names={joint_states.name}, positions={joint_states.position}")
                            joint_names = joint_states.name
                            joint_positions = joint_states.position
                            
                            # Add joint positions to response
                            for i, name in enumerate(joint_names):
                                if i < len(joint_positions):
                                    response[f"joint.{name}"] = joint_positions[i]
                    else:
                        response = {"status": "error", "message": "Failed to get robot state"}
                
                elif request["command"] == "execute_command":
                    rospy.loginfo(f"Received execute_command request: {request['data']}")
                    response = self.execute_robot_command(request["data"])
                
                # Send response back to client
                rospy.loginfo(f"Sending response: {json.dumps(response)}")
                client_socket.sendall(json.dumps(response).encode('utf-8'))
                
            except json.JSONDecodeError:
                rospy.logerr("Invalid JSON received from client")
                client_socket.sendall(json.dumps({"status": "error", "message": "Invalid JSON"}).encode('utf-8'))
            
            except Exception as e:
                rospy.logerr(f"Error handling client request: {str(e)}")
                try:
                    client_socket.sendall(json.dumps({"status": "error", "message": str(e)}).encode('utf-8'))
                except:
                    pass
                break
        
        # Clean up
        rospy.loginfo(f"Client disconnected: {client_address}")
        if client_socket in self.clients:
            self.clients.remove(client_socket)
        try:
            client_socket.close()
        except:
            pass
    
    def start_server(self):
        """Start the TCP server"""
        self.running = True
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.socket.bind((self.host, self.port))
            self.socket.listen(5)
            rospy.loginfo(f"Server started on {self.host}:{self.port}")
            
            while self.running and not rospy.is_shutdown():
                try:
                    # Accept client connection
                    client_socket, client_address = self.socket.accept()
                    self.clients.append(client_socket)
                    
                    # Start a new thread to handle client communication
                    client_thread = threading.Thread(
                        target=self.handle_client_connection,
                        args=(client_socket, client_address)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    
                except Exception as e:
                    if self.running:
                        rospy.logerr(f"Error accepting connection: {str(e)}")
                        time.sleep(1)
        
        except Exception as e:
            rospy.logerr(f"Server error: {str(e)}")
        
        finally:
            self.stop_server()
    
    def stop_server(self):
        """Stop the TCP server and clean up resources"""
        self.running = False
        
        # Close all client connections
        for client in self.clients:
            try:
                client.close()
            except:
                pass
        self.clients.clear()
        
        # Close server socket
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        # Unregister ROS subscribers
        if self.is_node_created:
            self.sub_end_pose.unregister()
            self.sub_joint_states.unregister()
            self.is_node_created = False
        
        rospy.loginfo("Server stopped")

    def get_joint_states(self):
        """Get the current joint states"""
        # Wait for joint states data
        timeout_counter = 0
        while not self.is_get_joint_states and timeout_counter < 50:
            rospy.sleep(0.1)
            timeout_counter += 1
        
        if not self.is_get_joint_states:
            rospy.logwarn("Failed to get joint states (timeout)")
            return None
        
        joint_states = self.joint_states
        self.is_get_joint_states = False
        return joint_states

if __name__ == "__main__":
    try:
        server = RobotControlServer()
        server.start_server()
    except rospy.ROSInterruptException:
        pass 