#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode, NodeBaseWidget
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from swiftpro.msg import SwiftproState
from swiftpro.msg import position
from swiftpro.msg import status
import json
from openai import OpenAI
import re
from Qt import QtCore, QtWidgets

__all__ = ['PandaArmControlNode','PandaArmDeepSeekControlNode']


class PandaArmDeepSeekControlNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.arm.control'
    NODE_NAME = 'panda arm controlled by deepseek'

    def __init__(self):
        super(PandaArmDeepSeekControlNode, self).__init__()
        self.add_input('text_in')
        self.add_output('next_step')

        self.add_text_input('max_mem_len', label="Max Memory Length")
        self.set_property("max_mem_len", "20")
        
        self.client = OpenAI(api_key="sk-bda36ce6acda4b50a00d327c32a48f80", base_url="https://api.deepseek.com")
        system_prompt = """
                        你是一个六自由度的机械臂，其中的answer答复要有拟人性。

                        请按照用户的意图给出机械臂末端的位置和姿态。

                        json输出示例:
                        {
                            "position.x": 0.3,
                            "position.y": 0.3,
                            "position.z": 0.2,
                            "orientation.x": 0.0,
                            "orientation.y": 0.0,
                            "orientation.z": 0.0,
                            "orientation.w": 1.0,
                            "gripper_state": "open", #只有open和close
                            "answer": "已给出目标位置和姿态，后续将进行规划执行动作。"
                        }
                        """
        system_message = {"role": "system", "content": system_prompt}

        self.messages = [system_message]

        self.is_created_node = False

    def create_ros_node(self,):
        """创建ROS节点和相关的发布者、订阅者"""
        # 初始化ROS节点
        if not rospy.get_node_uri():
            rospy.init_node(self.NODE_NAME.replace(' ', '_'), anonymous=True)
        
        # 创建发布者，用于发送位置控制命令
        self.pub_position = rospy.Publisher('position_write_topic', position, queue_size=10)
        self.pub_gripper = rospy.Publisher('gripper_topic', status, queue_size=10)
        
        # 创建订阅者，接收SwiftproState信息
        self.sub_end_pose = rospy.Subscriber(
            'SwiftproState_topic',
            SwiftproState,
            self.end_pose_listener_callback,
            queue_size=10)
        
        self.end_pose = None
        self.is_get_end_pose = False
        self.is_created_node = True

    def delete_ros_node(self,):
        """清理ROS节点资源"""
        if self.is_created_node:
            self.sub_end_pose.unregister()
            self.is_created_node = False

    def end_pose_listener_callback(self, msg):
        """接收机械臂末端位置的回调函数"""
        self.end_pose = [msg.x, msg.y, msg.z, 0.0, 0.0, 0.0, 1.0]  # 使用默认方向值
        self.is_get_end_pose = True

    def execute(self):
        """执行节点逻辑"""
        if not self.is_created_node:
            self.create_ros_node()

        # 获取语音节点的文本输入
        text_in = self.input(0).connected_ports()[0].node().text_out
        
        # 通过话题获取当前的机械臂位置状态
        timeout_counter = 0
        while not self.is_get_end_pose and timeout_counter < 50:
            rospy.sleep(0.1)
            timeout_counter += 1
            
        if not self.is_get_end_pose:
            self.messageSignal.emit(f'{self.NODE_NAME}获取机械臂位置超时!')
            self.text_out = "获取机械臂位置失败!"
            self.delete_ros_node()
            return
            
        end_pose = self.end_pose[:]
        # 重置标志位
        self.is_get_end_pose = False

        user_prompt = """
                        机械臂末端的当前位置和姿态是： "position.x": %s,"position.y": %s,"position.z": %s,"orientation.x": %s,"orientation.y": %s,"orientation.z": %s,"orientation.w": %s,
                        用户的意图是：%s
                    """ % (end_pose[0], end_pose[1], end_pose[2], end_pose[3], end_pose[4], end_pose[5], end_pose[6], text_in)

        self.messages.append({"role": "user", "content": user_prompt})

        self.messageSignal.emit(f'{self.NODE_NAME}的LLM输入： "role": "user", "content": {user_prompt}')

        # 创建聊天请求
        chat_completion = self.client.chat.completions.create(
            messages=self.messages, model="deepseek-chat", )
        assistant_message = chat_completion.choices[0].message.content

        pattern = r'\{([^{}]*)\}'
        matches = re.findall(pattern, assistant_message)
        assistant_message = '{' + matches[0] + '}'

        self.messages.append(chat_completion.choices[0].message)

        # 如果超出最长记录长度，删除第二个消息
        if len(self.messages) > int(self.get_property("max_mem_len")):
            del self.messages[1:3]

        # 加判断，解析返回的json数据
        data_json = json.loads(assistant_message)
        self.messageSignal.emit(f'{self.NODE_NAME}的LLM输出：{data_json}')
        
        # 创建控制消息
        pos_msg = position()
        pos_msg.x = float(data_json['position.x'])
        pos_msg.y = float(data_json['position.y'])
        pos_msg.z = float(data_json['position.z'])
        pos_msg.speed = 1000  # 设置默认速度
        
        # 发送位置控制消息
        self.pub_position.publish(pos_msg)
        
        # 创建夹爪控制消息
        gripper_msg = status()
        gripper_msg.status = 1 if data_json['gripper_state'] == 'close' else 0
        
        # 发送夹爪控制消息
        self.pub_gripper.publish(gripper_msg)
        
        # 等待执行完成
        rospy.sleep(3.0)  # 给机械臂足够的时间来执行动作
        
        self.messageSignal.emit(f'{self.NODE_NAME}命令已发送')
        self.text_out = data_json['answer'] + "执行完成！"

        self.delete_ros_node()
        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

class MyCustomWidget(QtWidgets.QWidget):
    """
    Custom widget to be embedded inside a node.
    """
    def __init__(self, parent=None):
        super(MyCustomWidget, self).__init__(parent)
        
        self.btn = QtWidgets.QPushButton("get_end_effector_pose")

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.btn)

        self.btn.clicked.connect(self.get_end_pose)

    def set_node_obj(self, obj):
        """"""
        self.node_obj = obj

    def get_end_pose(self,):
        """"""
        if not self.node_obj.is_created_node:
            self.node_obj.create_ros_node()
        self.node_obj.get_end_pose()
        self.node_obj.get_joint_states()

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
        """"""
    def get_value(self):
        """"""
    def set_value(self, value):
        """"""

class PandaArmControlNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.arm.control'
    NODE_NAME = 'panda arm control'

    def __init__(self):
        super(PandaArmControlNode, self).__init__()
        self.add_input('in')
        self.add_output('next_step')

        node_widget = NodeWidgetWrapper(self.view)
        node_widget.get_custom_widget().set_node_obj(self)
        self.add_custom_widget(node_widget, tab='Custom')
        
        self.add_text_input('position.x', 'position.x', text='0.3')
        self.add_text_input('position.y', 'position.y', text='0.3')
        self.add_text_input('position.z', 'position.z', text='0.3')
        self.add_text_input('orientation.x', 'orientation.x', text='0.0')
        self.add_text_input('orientation.y', 'orientation.y', text='0.0')
        self.add_text_input('orientation.z', 'orientation.z', text='0.0')
        self.add_text_input('orientation.w', 'orientation.w', text='1.0')

        self.add_combo_menu('gripper_state', 'gripper_state', items=['open', 'close'])

        self.is_created_node = False

    def create_ros_node(self,):
        """创建ROS节点和相关的发布者、订阅者"""
        if not rospy.get_node_uri():
            rospy.init_node(self.NODE_NAME.replace(' ', '_'), anonymous=True)
        
        # 创建发布者，用于发送位置控制命令
        self.pub_position = rospy.Publisher('position_write_topic', position, queue_size=10)
        self.pub_gripper = rospy.Publisher('gripper_topic', status, queue_size=10)
        
        # 创建订阅者，接收SwiftproState信息
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
            
        self.end_pose = None
        self.is_get_end_pose = False
        self.joint_states = None
        self.is_get_joint_states = False
        
        self.is_created_node = True
    
    def delete_ros_node(self,):
        """清理ROS节点资源"""
        if self.is_created_node:
            self.sub_end_pose.unregister()
            self.sub_joint_states.unregister()
            self.is_created_node = False
    
    def joint_states_listener_callback(self, msg):
        """接收关节状态的回调函数"""
        self.joint_states = msg
        self.is_get_joint_states = True

    def end_pose_listener_callback(self, msg):
        """接收机械臂末端位置的回调函数"""
        self.end_pose = [msg.x, msg.y, msg.z, 0.0, 0.0, 0.0, 1.0]  # 使用默认方向值
        self.is_get_end_pose = True

    def get_joint_states(self,):
        """获取关节状态"""
        timeout_counter = 0
        while not self.is_get_joint_states and timeout_counter < 50:
            rospy.sleep(0.1)
            timeout_counter += 1
            
        if not self.is_get_joint_states:
            return
            
        joint_states = self.joint_states
        self.is_get_joint_states = False
        
        if joint_states and len(joint_states.position) > 0:
            finger_joint_posi = joint_states.position[-1]
            if finger_joint_posi < 0.01:
                self.set_property('gripper_state', 'close')
            else:
                self.set_property('gripper_state', 'open')
    
    def get_end_pose(self,):
        """获取机械臂末端位置"""
        timeout_counter = 0
        while not self.is_get_end_pose and timeout_counter < 50:
            rospy.sleep(0.1)
            timeout_counter += 1
            
        if not self.is_get_end_pose:
            return None
            
        end_pose = self.end_pose[:]
        self.is_get_end_pose = False

        # 设置属性
        self.set_property('position.x', str(end_pose[0]))
        self.set_property('position.y', str(end_pose[1]))
        self.set_property('position.z', str(end_pose[2]))
        self.set_property('orientation.x', str(end_pose[3]))
        self.set_property('orientation.y', str(end_pose[4]))
        self.set_property('orientation.z', str(end_pose[5]))
        self.set_property('orientation.w', str(end_pose[6]))

        return end_pose

    def execute(self):
        """执行节点逻辑"""
        if not self.is_created_node:
            self.create_ros_node()
            
        # 创建位置控制消息
        pos_msg = position()
        pos_msg.x = float(self.get_property('position.x'))
        pos_msg.y = float(self.get_property('position.y'))
        pos_msg.z = float(self.get_property('position.z'))
        pos_msg.speed = 1000  # 设置默认速度
        
        # 发送位置控制消息
        self.pub_position.publish(pos_msg)
        
        # 创建夹爪控制消息
        gripper_msg = status()
        gripper_msg.status = 1 if self.get_property('gripper_state') == 'close' else 0
        
        # 发送夹爪控制消息
        self.pub_gripper.publish(gripper_msg)
        
        # 等待执行完成
        rospy.sleep(3.0)  # 给机械臂足够的时间来执行动作
        
        self.messageSignal.emit(f'位置 [{pos_msg.x}, {pos_msg.y}, {pos_msg.z}] 执行成功.')
        self.text_out = "执行成功！"
        
        self.delete_ros_node()
        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal


