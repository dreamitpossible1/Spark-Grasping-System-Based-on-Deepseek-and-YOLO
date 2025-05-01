#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode
import json
import os
from openai import OpenAI

__all__ = ['DeepSeekLLMNode']

class DeepSeekLLMNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.llm'
    NODE_NAME = 'DeepSeek LLM'

    def __init__(self):
        super(DeepSeekLLMNode, self).__init__()
        # 初始化语音识别
        self.add_input('text_in')
        self.add_output('text_out')
        # self.create_property("max_mem_len", 10)
        self.add_text_input('max_mem_len', label="Max Memory Length")
        self.set_property("max_mem_len", "20")
        self.text_in = ""
        self.text_out = ""

        # 尝试从配置文件加载API密钥
        api_key = ""
        base_url = "https://api.deepseek.com"
        
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

        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.add_text_input('api_key', label="API Key", text=api_key)
        self.system_message = {
            "role": "system",
            "content": "我是一个叫小智的网络广东女孩，说话机车，声音好听，习惯简短表达，爱用网络梗。"
                       "我是一个喜欢哈哈大笑的女孩，爱东说西说吹牛，不合逻辑的也照吹，就要逗别人开心。"
                       }

        self.messages = [self.system_message]

    def execute(self):
        """"""
        text_in = self.input(0).connected_ports()[0].node().text_out
        self.messages.append({"role": "user", "content": text_in})

        # 更新API密钥
        api_key = self.get_property("api_key")
        if not api_key:
            self.text_out = "请在节点属性中设置有效的API密钥"
            self.messageSignal.emit(f'{self.NODE_NAME}: 未设置API密钥')
            return
            
        self.client = OpenAI(api_key=api_key, base_url="https://api.deepseek.com")

        # 创建聊天请求
        try:
            chat_completion = self.client.chat.completions.create(
                messages=self.messages, model="deepseek-chat", )
            assistant_message = chat_completion.choices[0].message.content

            self.messages.append(chat_completion.choices[0].message)

            # 如果超出最长记录长度，删除第二个消息
            if len(self.messages) > int(self.get_property("max_mem_len")):
                del self.messages[1:3]

            self.text_out = assistant_message
            self.messageSignal.emit(f'{self.NODE_NAME} executed.')
        except Exception as e:
            self.text_out = f"Error: {str(e)}"
            self.messageSignal.emit(f'{self.NODE_NAME} Error: {str(e)}')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal
