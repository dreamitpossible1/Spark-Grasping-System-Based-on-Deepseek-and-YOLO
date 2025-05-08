#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Client Node - Runs in Python 3.12 environment
This module handles the LLM processing and communicates with the robot control server
via TCP sockets.
"""
from NodeGraphQt import BaseNode
import socket
import json
import re
import os
from openai import OpenAI
from Qt import QtWidgets

class LLMClientNode(BaseNode):
    """Node for processing user input with LLM and controlling robot via socket connection"""
    __identifier__ = 'nodes.arm.llm_control'
    NODE_NAME = 'LLM Robot Control'

    def __init__(self):
        super(LLMClientNode, self).__init__()
        self.add_input('text_in')
        self.add_output('next_step')

        # Server connection settings
        self.add_text_input('server_host', 'Server Host', text='localhost')
        self.add_text_input('server_port', 'Server Port', text='9999')
        
        # LLM settings
        self.add_text_input('max_mem_len', label="Max Memory Length", text="20")
        
        # Try to load API key from config file
        api_key = ""
        base_url = "https://api.deepseek.com"
        model = "deepseek-chat"
        
        # 获取配置文件路径
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(script_dir, "settings", "api_keys.json")
        
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = json.load(f)
                    if 'deepseek' in config and 'api_key' in config['deepseek']:
                        api_key = config['deepseek']['api_key']
                    if 'deepseek' in config and 'base_url' in config['deepseek']:
                        base_url = config['deepseek']['base_url']
        except Exception as e:
            print(f"Error loading API key config: {e}")
            
        self.add_text_input('api_key', label="API Key", text=api_key)
        self.add_text_input('base_url', label="API Base URL", text=base_url)
        self.add_text_input('model', label="Model Name", text=model)
        
        # Initialize LLM client
        self.init_llm_client()
        
        # Message history for LLM conversation
        system_prompt = """
            你是一个六自由度机械臂控制助手，需要根据用户的意图，规划出机械臂末端的位置（position）和姿态（orientation），并决定吸盘状态（pump_state）。你的回答必须严格以 JSON 格式输出，并包括以下字段：
            
            - position.x: float，单位mm，正数为正方向，前后方向
            - position.y: float，单位mm，正数为正方向，左右方向
            - position.z: float，单位mm，正数为正方向，上下方向
            - orientation.x: float（四元数）
            - orientation.y: float
            - orientation.z: float
            - orientation.w: float
            - pump_state: "on" 或 "off"（控制吸盘开关状态）
            - answer: 拟人化地简述你理解的用户意图以及接下来的操作
            
            请仅返回 JSON 对象，格式示例如下：
            {
              "position.x": 100.0,
              "position.y": 100.0,
              "position.z": 100.0,
              "orientation.x": 0.0,
              "orientation.y": 0.0,
              "orientation.z": 0.0,
              "orientation.w": 1.0,
              "pump_state": "on",
              "answer": "目标位置已设定，准备开始动作执行。"
            }
            """
        system_message = {"role": "system", "content": system_prompt}
        self.messages = [system_message]

    def init_llm_client(self):
        """Initialize the LLM client with current settings"""
        api_key = self.get_property("api_key")
        base_url = self.get_property("base_url")
        
        self.client = OpenAI(api_key=api_key, base_url=base_url)

    def connect_to_server(self):
        """Connect to the robot control server via TCP socket"""
        host = self.get_property("server_host")
        port = int(self.get_property("server_port"))
        
        try:
            # Create socket and connect to server
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host, port))
            self.log_message(f"Connected to server at {host}:{port}")
            return sock
        except Exception as e:
            self.log_message(f"Failed to connect to server: {str(e)}")
            return None

    def get_robot_state(self):
        """Get the current robot state from the server"""
        sock = self.connect_to_server()
        if not sock:
            return None
        
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
                return response
            else:
                self.log_message(f"Error getting robot state: {response.get('message', 'Unknown error')}")
                return None
        except Exception as e:
            self.log_message(f"Error in communication with server: {str(e)}")
            return None
        finally:
            sock.close()

    def execute_robot_command(self, command_data):
        """Send a command to the robot control server"""
        sock = self.connect_to_server()
        if not sock:
            return False
        
        try:
            # 处理pump_state，转换为适合机械臂使用的格式
            if 'pump_state' in command_data:
                pump_state = 1 if command_data['pump_state'].lower() == 'on' else 0
                
                # 为了操作吸盘，需要发布pump_topic消息
                # 通过socket请求robot_control_server发布
                command_data['pump_value'] = pump_state
            
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
                self.log_message(f"Command executed successfully: {response.get('message', '')}")
                return True
            else:
                self.log_message(f"Error executing command: {response.get('message', 'Unknown error')}")
                return False
        except Exception as e:
            self.log_message(f"Error in communication with server: {str(e)}")
            return False
        finally:
            sock.close()

    def parse_llm_response(self, response_text):
        """Parse the JSON response from LLM"""
        try:
            # Extract JSON object from response using regex
            pattern = r'\{([^{}]*)\}'
            matches = re.findall(pattern, response_text)
            if matches:
                json_str = '{' + matches[0] + '}'
                return json.loads(json_str)
            else:
                self.log_message("No valid JSON found in LLM response")
                return None
        except Exception as e:
            self.log_message(f"Error parsing LLM response: {str(e)}")
            return None

    def log_message(self, message):
        """Log a message through the node's message signal"""
        if hasattr(self, 'messageSignal'):
            self.messageSignal.emit(f"{self.NODE_NAME}: {message}")
        
    def execute(self):
        """Execute the node's logic"""
        # Get input text from connected node
        text_in = self.input(0).connected_ports()[0].node().text_out
        
        # Get current robot state
        robot_state = self.get_robot_state()
        if not robot_state:
            self.log_message("Failed to get robot state. Check server connection.")
            self.text_out = "Failed to execute: Cannot connect to robot control server."
            return
        
        # Prepare prompt for LLM
        user_prompt = f"""
        当前机械臂末端的状态如下：
        - position.x: {robot_state['position.x']}
        - position.y: {robot_state['position.y']}
        - position.z: {robot_state['position.z']}
        - orientation.x: {robot_state['orientation.x']}
        - orientation.y: {robot_state['orientation.y']}
        - orientation.z: {robot_state['orientation.z']}
        - orientation.w: {robot_state['orientation.w']}
        - pump_state: {robot_state['pump_state']}  # 吸盘状态 on/off

        用户意图是：{text_in}

        请你根据以上信息，输出一个新的 JSON 动作指令。
        """
        self.messages.append({"role": "user", "content": user_prompt})
        self.log_message(f"LLM Input: {user_prompt}")
        
        # Initialize LLM client with latest settings (in case they were changed)
        self.init_llm_client()
        
        try:
            # Create chat completion request
            model = self.get_property("model")
            chat_completion = self.client.chat.completions.create(
                messages=self.messages, model=model)
            
            assistant_message = chat_completion.choices[0].message.content
            self.messages.append(chat_completion.choices[0].message)
            
            # Manage memory length
            max_mem_len = int(self.get_property("max_mem_len"))
            if len(self.messages) > max_mem_len:
                del self.messages[1:3]  # Remove oldest user-assistant message pair
            
            # Parse the response
            data_json = self.parse_llm_response(assistant_message)
            if not data_json:
                self.log_message("Failed to parse LLM response")
                self.text_out = "Failed to execute: Invalid response from LLM."
                return
            
            self.log_message(f"LLM Output: {data_json}")
            
            # Execute robot command
            success = self.execute_robot_command(data_json)
            if success:
                self.text_out = data_json['answer'] + " 执行完成！"
            else:
                self.text_out = "Failed to execute command on robot."
                
        except Exception as e:
            self.log_message(f"Error in LLM processing: {str(e)}")
            self.text_out = f"Error: {str(e)}"
            
        self.log_message(f"{self.NODE_NAME} executed.")

    def set_messageSignal(self, messageSignal):
        """Set the message signal for logging"""
        self.messageSignal = messageSignal


class PandaArmDeepSeekControlNode(BaseNode):
    """Legacy node for backward compatibility, redirects to LLMClientNode"""
    __identifier__ = 'nodes.arm.control'
    NODE_NAME = 'panda arm controlled by deepseek'

    def __init__(self):
        super(PandaArmDeepSeekControlNode, self).__init__()
        self.add_input('text_in')
        self.add_output('next_step')

        self.add_text_input('max_mem_len', label="Max Memory Length")
        self.set_property("max_mem_len", "20")
        
        # Create actual LLM client node instance
        self.llm_client = LLMClientNode()
        
    def execute(self):
        """Redirect to LLMClientNode execution"""
        # Transfer properties
        self.llm_client.set_property("max_mem_len", self.get_property("max_mem_len"))
        
        # Transfer input
        if hasattr(self, 'messageSignal'):
            self.llm_client.set_messageSignal(self.messageSignal)
            
        # Get input value
        text_in = self.input(0).connected_ports()[0].node().text_out
        
        # Set dummy property to pass to LLM client
        class DummyNode:
            def __init__(self, text):
                self.text_out = text
        
        class DummyPort:
            def __init__(self, text):
                self.dummy_node = DummyNode(text)
            
            def connected_ports(self):
                return [self]
            
            def node(self):
                return self.dummy_node
        
        # Mock input connection
        self.llm_client.input(0).set_connected_port = True
        self.llm_client.input(0).connected_ports = lambda: [DummyPort(text_in)]
        
        # Execute LLM client
        self.llm_client.execute()
        
        # Get output from LLM client
        self.text_out = self.llm_client.text_out
        
        self.messageSignal.emit(f'{self.NODE_NAME} executed (via LLMClientNode).')

    def set_messageSignal(self, messageSignal):
        """Set the message signal for logging"""
        self.messageSignal = messageSignal 