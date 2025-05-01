#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compatibility layer for robot control nodes.
This module provides backward compatibility with existing code while
delegating actual functionality to the new client-server architecture.
"""
from NodeGraphQt import BaseNode, NodeBaseWidget
import socket
import json
from Qt import QtCore, QtWidgets

__all__ = ['PandaArmControlNode','PandaArmDeepSeekControlNode', 'LLMClientNode']

# Import the LLMClientNode if in Python 3.12 environment
try:
    from src.team17.graph_executer_controller.nodes.llm_client_node import LLMClientNode, PandaArmDeepSeekControlNode
except ImportError:
    # Define placeholder classes if in Python 3.8 environment
    class LLMClientNode(BaseNode):
        """Placeholder for LLMClientNode when running in Python 3.8 environment"""
        __identifier__ = 'nodes.arm.llm_control'
        NODE_NAME = 'LLM Robot Control (Unavailable)'
        
        def __init__(self):
            super(LLMClientNode, self).__init__()
            self.add_input('text_in')
            self.add_output('next_step')
            self.add_text_input('error_message', 'Error', text='This node requires Python 3.12 environment')
            self.text_out = ""
        
        def execute(self):
            """Display error message"""
            self.text_out = "This node cannot be executed in Python 3.8 environment"
            if hasattr(self, 'messageSignal'):
                self.messageSignal.emit(f'{self.NODE_NAME}: Cannot run in Python 3.8 environment')
        
        def set_messageSignal(self, messageSignal):
            """Set the message signal for logging"""
            self.messageSignal = messageSignal
    
    # Legacy compatibility class
    class PandaArmDeepSeekControlNode(BaseNode):
        """Legacy node pointing to use the new client-server architecture"""
        __identifier__ = 'nodes.arm.control'
        NODE_NAME = 'panda arm controlled by deepseek'

        def __init__(self):
            super(PandaArmDeepSeekControlNode, self).__init__()
            self.add_input('text_in')
            self.add_output('next_step')
            self.add_text_input('max_mem_len', label="Max Memory Length")
            self.set_property("max_mem_len", "20")
            self.add_text_input('server_host', 'Robot Server Host', text='localhost')
            self.add_text_input('server_port', 'Robot Server Port', text='9999')
            self.text_out = ""
            
        def execute(self):
            """Advise users to use the new architecture"""
            self.text_out = "This node is deprecated. Please use the new LLM Robot Control node with the Robot Control Server."
            if hasattr(self, 'messageSignal'):
                self.messageSignal.emit(f'{self.NODE_NAME}: This node is deprecated')
        
        def set_messageSignal(self, messageSignal):
            """Set the message signal for logging"""
            self.messageSignal = messageSignal


class MyCustomWidget(QtWidgets.QWidget):
    """
    Custom widget to be embedded inside a node.
    """
    def __init__(self, parent=None):
        super(MyCustomWidget, self).__init__(parent)
        
        self.btn = QtWidgets.QPushButton("get_end_effector_pose")
        self.node_obj = None

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.btn)

        self.btn.clicked.connect(self.get_end_pose)

    def set_node_obj(self, obj):
        """Set the parent node object"""
        self.node_obj = obj

    def get_end_pose(self):
        """Get current robot pose from server"""
        if self.node_obj:
            self.node_obj.get_robot_state()


class NodeWidgetWrapper(NodeBaseWidget):
    """
    Wrapper that allows the widget to be added in a node object.
    """
    def __init__(self, parent=None):
        super(NodeWidgetWrapper, self).__init__(parent)
        # set the name for node property.
        self.set_name('my_widget')
        # set the custom widget.
        self.set_custom_widget(MyCustomWidget())
        # connect up the signals & slots.
        self.wire_signals()
    
    def wire_signals(self):
        """Wire up signals"""
        pass
    
    def get_value(self):
        """Get widget value"""
        pass
    
    def set_value(self, value):
        """Set widget value"""
        pass


class PandaArmControlNode(BaseNode):
    """Direct robot control node using socket connection to server"""
    __identifier__ = 'nodes.arm.control'
    NODE_NAME = 'panda arm control'

    def __init__(self):
        super(PandaArmControlNode, self).__init__()
        self.add_input('in')
        self.add_output('next_step')

        # Add custom widget for get pose button
        node_widget = NodeWidgetWrapper(self.view)
        node_widget.get_custom_widget().set_node_obj(self)
        self.add_custom_widget(node_widget, tab='Custom')
        
        # Server connection settings
        self.add_text_input('server_host', 'Robot Server Host', text='localhost')
        self.add_text_input('server_port', 'Robot Server Port', text='9999')
        
        # Robot pose settings
        self.add_text_input('position.x', 'position.x', text='0.3')
        self.add_text_input('position.y', 'position.y', text='0.3')
        self.add_text_input('position.z', 'position.z', text='0.3')
        self.add_text_input('orientation.x', 'orientation.x', text='0.0')
        self.add_text_input('orientation.y', 'orientation.y', text='0.0')
        self.add_text_input('orientation.z', 'orientation.z', text='0.0')
        self.add_text_input('orientation.w', 'orientation.w', text='1.0')
        self.add_combo_menu('gripper_state', 'gripper_state', items=['open', 'close'])
        
        self.text_out = ""

    def connect_to_server(self):
        """Connect to the robot control server"""
        host = self.get_property("server_host")
        port = int(self.get_property("server_port"))
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host, port))
            if hasattr(self, 'messageSignal'):
                self.messageSignal.emit(f"{self.NODE_NAME}: Connected to server at {host}:{port}")
            return sock
        except Exception as e:
            if hasattr(self, 'messageSignal'):
                self.messageSignal.emit(f"{self.NODE_NAME}: Failed to connect to server: {str(e)}")
            return None

    def get_robot_state(self):
        """Get current robot state from server"""
        sock = self.connect_to_server()
        if not sock:
            return
        
        try:
            # Send request for robot state
            request = {
                "command": "get_robot_state"
            }
            sock.sendall(json.dumps(request).encode('utf-8'))
            
            # Receive response
            response_data = sock.recv(4096)
            response = json.loads(response_data.decode('utf-8'))
            
            if response["status"] == "success":
                # Update properties with current values
                self.set_property('position.x', str(response['position.x']))
                self.set_property('position.y', str(response['position.y']))
                self.set_property('position.z', str(response['position.z']))
                self.set_property('orientation.x', str(response['orientation.x']))
                self.set_property('orientation.y', str(response['orientation.y']))
                self.set_property('orientation.z', str(response['orientation.z']))
                self.set_property('orientation.w', str(response['orientation.w']))
                self.set_property('gripper_state', response['gripper_state'])
                
                if hasattr(self, 'messageSignal'):
                    self.messageSignal.emit(f"{self.NODE_NAME}: Robot state updated")
            else:
                if hasattr(self, 'messageSignal'):
                    self.messageSignal.emit(f"{self.NODE_NAME}: Error getting robot state: {response.get('message', 'Unknown error')}")
        except Exception as e:
            if hasattr(self, 'messageSignal'):
                self.messageSignal.emit(f"{self.NODE_NAME}: Error in communication with server: {str(e)}")
        finally:
            sock.close()

    def execute(self):
        """Execute control command on the robot via server"""
        sock = self.connect_to_server()
        if not sock:
            self.text_out = "Failed to connect to robot server"
            return
            
        try:
            # Build command data from properties
            command_data = {
                "position.x": float(self.get_property('position.x')),
                "position.y": float(self.get_property('position.y')),
                "position.z": float(self.get_property('position.z')),
                "orientation.x": float(self.get_property('orientation.x')),
                "orientation.y": float(self.get_property('orientation.y')),
                "orientation.z": float(self.get_property('orientation.z')),
                "orientation.w": float(self.get_property('orientation.w')),
                "gripper_state": self.get_property('gripper_state')
            }
            
            # Send command request
            request = {
                "command": "execute_command",
                "data": command_data
            }
            sock.sendall(json.dumps(request).encode('utf-8'))
            
            # Receive response
            response_data = sock.recv(4096)
            response = json.loads(response_data.decode('utf-8'))
            
            if response["status"] == "success":
                pos_x = command_data["position.x"]
                pos_y = command_data["position.y"]
                pos_z = command_data["position.z"]
                
                if hasattr(self, 'messageSignal'):
                    self.messageSignal.emit(f"{self.NODE_NAME}: 位置 [{pos_x}, {pos_y}, {pos_z}] 执行成功")
                self.text_out = "执行成功！"
            else:
                if hasattr(self, 'messageSignal'):
                    self.messageSignal.emit(f"{self.NODE_NAME}: Error executing command: {response.get('message', 'Unknown error')}")
                self.text_out = f"执行失败: {response.get('message', '未知错误')}"
                
        except Exception as e:
            if hasattr(self, 'messageSignal'):
                self.messageSignal.emit(f"{self.NODE_NAME}: Error in communication with server: {str(e)}")
            self.text_out = f"错误: {str(e)}"
        finally:
            sock.close()
            
        if hasattr(self, 'messageSignal'):
            self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        """Set the message signal for logging"""
        self.messageSignal = messageSignal


