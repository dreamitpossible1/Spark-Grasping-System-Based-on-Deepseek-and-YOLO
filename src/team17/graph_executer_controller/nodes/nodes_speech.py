#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode
import pyttsx3, os
from vosk import Model, KaldiRecognizer, SetLogLevel
import pyaudio
import json
import numpy as np
from src.team17.graph_executer_controller.utils.general import download_and_extract_zip
import wave
import edge_tts
import asyncio
import tempfile
from playsound import playsound
import time
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

__all__ = ['Pyttsx3SpeakNode', 'VOSKRecognitionNode', 'EdgeTTSSpeakNode', 'TextInputNode']


class TextInputNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.speech'
    NODE_NAME = 'Text input'

    def __init__(self):
        super(TextInputNode, self).__init__()
        self.add_text_input('text_in')
        self.add_output('text_out')
        self.text_out = ""

    def execute(self):
        self.text_out = self.get_property('text_in')
        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal


class EdgeTTSSpeakNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.speech'
    NODE_NAME = 'Speak by edge-tts '

    def __init__(self):
        super(EdgeTTSSpeakNode, self).__init__()
        # 初始化语音引擎
        self.engine = pyttsx3.init()
        # 设置语音属性（可选）
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[0].id)  # 0通常是英文男声，1可能是英文女声，中文可能需要其他设置
        self.engine.setProperty('rate', 150)  # 语速
        self.add_input('text_in')

        self.add_checkbox('isConversationLoop', text='是否开启对话循环')

    def execute(self):
        """"""
        text = self.input(0).connected_ports()[0].node().text_out
        self.messageSignal.emit(f"{self.name()} Output result: {text}")
        asyncio.run(self.play_with_playsound(text))

        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

        if self.get_property('isConversationLoop'):
            # 遍历前置节点，并保存到一个列表中
            self.get_execution_order(self) # 遍历前置节点，再执行它们

    async def play_with_playsound(self,text):
        voice = 'zh-CN-XiaoxiaoNeural'
        # text = "你好，这是一个使用playsound播放的测试。"

        # 创建临时文件
        with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as tmp_file:
            tmp_path = tmp_file.name

        # 保存到临时文件
        communicate = edge_tts.Communicate(text, voice)
        await communicate.save(tmp_path)

        # 播放
        playsound(tmp_path)

        # 删除临时文件
        os.unlink(tmp_path)


    def get_execution_order(self, obj_node):
        """获取从指定节点开始的下游节点执行顺序（拓扑排序）"""
        visited = set()
        # execution_order = []

        def visit_up(node):
            if node in visited:
                return
            # 首先处理所有上游节点
            for port in node.inputs().values():
                for connected_port in port.connected_ports():
                    visit_up(connected_port.node())
            visited.add(node)
            node.set_messageSignal(self.messageSignal)
            if hasattr(node, 'execute'):
                 node.execute() # 运行节点
        visit_up(obj_node)

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal


class VOSKRecognitionNode(BaseNode):
    """科学上网下载速度才快，否则只有几十kb/s"""
    __identifier__ = 'nodes.speech'
    NODE_NAME = 'Speech recognition by vosk'

    def __init__(self):
        super(VOSKRecognitionNode, self).__init__()
        # 加载模型
        # https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip         41.87 M
        # https://alphacephei.com/vosk/models/vosk-model-cn-0.15.zip                1.67 G
        models_dir = os.path.join(BASE_DIR, 'res', 'models', 'VOSK')
        if not os.path.exists(models_dir):
            os.makedirs(models_dir)

        model_dir = os.path.join(BASE_DIR, 'res', 'models', 'VOSK', 'vosk-model-small-cn-0.22')
        if os.path.exists(model_dir):
            self.vosk_model = Model(model_dir)
        else:
            download_and_extract_zip('https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip',os.path.dirname(model_dir), os.path.dirname(model_dir))
            try:
                os.remove(os.path.join(os.path.dirname(model_dir), 'vosk-model-small-cn-0.22'))
            except OSError:
                pass
            self.vosk_model = Model(model_dir)
        SetLogLevel(-1)

        self.add_output('text_out')
        self.text_out = ""

    def execute(self):
        while True:
            text = self.save_wave(self.vosk_model)
            if text != "" and text != None:
                self.messageSignal.emit(f"{self.name()} Output result: {text}")
                self.text_out = text
                break
        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def save_wave(self, model):
        # 修改音频参数，参考spark_voice/scripts/lib/microphone.py的配置
        FORMAT = pyaudio.paInt16  # 音频流的格式
        RATE = 16000  # 采样率调整为16000Hz，与Microphone类一致
        CHUNK = 512  # 单位帧，与Microphone类一致
        THRESHOLDNUM = 30  # 静默时间，超过这个个数就保存文件
        THRESHOLD = 7000  # 提高停止采集阈值，适应嘈杂环境 (原先是50)
        MAX_RECORDING_TIME = 10  # 最长录音时间（秒）
        
        # 尝试禁止ALSA错误消息显示
        # 重定向stderr到/dev/null来抑制ALSA错误消息
        old_stderr = os.dup(2)
        os.close(2)
        os.open(os.devnull, os.O_WRONLY)
        
        try:
            audio = pyaudio.PyAudio()
            self.messageSignal.emit(f"调试: PyAudio初始化成功")
            
            # 打印可用的输入设备信息
            info = audio.get_host_api_info_by_index(0)
            num_devices = info.get('deviceCount')
            self.messageSignal.emit(f"调试: 检测到 {num_devices} 个音频设备")
            
            # 收集所有输入设备信息
            input_devices = []
            respeaker_index = None  # 优先查找ReSpeaker设备
            
            for i in range(num_devices):
                device_info = audio.get_device_info_by_index(i)
                if device_info.get('maxInputChannels') > 0:
                    device_name = device_info.get('name')
                    self.messageSignal.emit(f"调试: 输入设备 {i}: {device_name}")
                    input_devices.append((i, device_name))
                    
                    # 检查是否为ReSpeaker设备（参考Microphone类的逻辑）
                    if 'respeaker' in device_name.lower() and device_info.get('maxInputChannels') > 0:
                        self.messageSignal.emit(f"调试: 发现ReSpeaker设备: {device_name}")
                        respeaker_index = i
            
            # 设备选择优先级：ReSpeaker > 明确指定的设备 > 非ASTRA设备 > 默认设备
            input_device_index = None
            
            # 1. 优先使用ReSpeaker设备
            if respeaker_index is not None:
                input_device_index = respeaker_index
                self.messageSignal.emit(f"调试: 将使用ReSpeaker设备 (index: {respeaker_index})")
                
            # 2. 其次使用首选设备列表中的设备
            if input_device_index is None:
                preferred_device_names = ["pulse", "default", "HDA Intel PCH", "PCH"]
                for preferred_name in preferred_device_names:
                    for idx, name in input_devices:
                        # 排除ASTRA设备，因为它是机械臂而非麦克风
                        if preferred_name in name and "ASTRA" not in name:
                            input_device_index = idx
                            self.messageSignal.emit(f"调试: 选择首选设备: {name}")
                            break
                    if input_device_index is not None:
                        break
            
            # 3. 如果没有找到首选设备，尝试任何非ASTRA设备
            if input_device_index is None:
                for idx, name in input_devices:
                    if "ASTRA" not in name:
                        input_device_index = idx
                        self.messageSignal.emit(f"调试: 未找到首选设备，使用非ASTRA设备: {name}")
                        break
            
            # 4. 如果还是没有找到，则使用默认设备
            if input_device_index is None:
                input_device_index = audio.get_default_input_device_info()['index']
                self.messageSignal.emit(f"调试: 使用默认输入设备")
            
            # 获取选定设备的能力
            device_info = audio.get_device_info_by_index(input_device_index)
            device_name = device_info.get('name')
            
            # 根据设备调整采样率
            if 'respeaker' in device_name.lower():
                # ReSpeaker通常使用16000Hz
                RATE = 16000
            else:
                # 其他设备使用其默认采样率
                sample_rate = int(device_info.get('defaultSampleRate'))
                if sample_rate != RATE:
                    self.messageSignal.emit(f"调试: 设备默认采样率为 {sample_rate}Hz，将使用此采样率")
                    RATE = sample_rate
            
            self.messageSignal.emit(f"调试: 将使用输入设备: {device_name} (采样率: {RATE}Hz)")
            
            # 打开音频流
            stream = audio.open(
                format=FORMAT,
                channels=1,
                rate=RATE,
                input=True,
                input_device_index=input_device_index,
                frames_per_buffer=CHUNK
            )
                                
            self.messageSignal.emit(f"调试: 音频流打开成功")
        except Exception as e:
            # 恢复stderr
            os.dup2(old_stderr, 2)
            os.close(old_stderr)
            self.messageSignal.emit(f"错误: 无法初始化录音设备: {str(e)}")
            return ""
            
        # 恢复stderr
        os.dup2(old_stderr, 2)
        os.close(old_stderr)
        
        frames = []
        # print("开始录音...")
        self.messageSignal.emit(f"{self.name()} 开始录音...")
        self.messageSignal.emit(f"调试信息: 阈值设置为 {THRESHOLD}，需要 {THRESHOLDNUM} 帧低于阈值才会停止录音")
        self.messageSignal.emit(f"调试信息: 最长录音时间为 {MAX_RECORDING_TIME} 秒")
        
        count = 0
        frame_count = 0
        start_time = time.time()
        
        try:
            # 添加时间限制条件
            while count < THRESHOLDNUM and (time.time() - start_time) < MAX_RECORDING_TIME:
                try:
                    data = stream.read(CHUNK, exception_on_overflow=False)
                    np_data = np.frombuffer(data, dtype=np.int16)
                    frame_energy = np.mean(np.abs(np_data))
                    
                    frame_count += 1
                    if frame_count % 20 == 0:  # 由于帧大小减小，调整打印频率
                        elapsed_time = time.time() - start_time
                        self.messageSignal.emit(f"调试: 帧 {frame_count}, 能量 {frame_energy:.2f}, 静默计数 {count}/{THRESHOLDNUM}, 已录音 {elapsed_time:.1f}秒")
                    
                    # 如果能量低于阈值持续时间过长，则停止录音
                    if frame_energy < THRESHOLD:
                        count += 1
                        if count % 5 == 0:  # 每累积5个静默帧打印一次
                            self.messageSignal.emit(f"调试: 检测到静默 {count}/{THRESHOLDNUM}")
                    elif count > 0:
                        if count > 10:  # 如果有明显的计数减少，打印提示
                            self.messageSignal.emit(f"调试: 检测到声音，静默计数重置从 {count} ")
                        count -= 1

                    frames.append(data)
                except IOError as e:
                    self.messageSignal.emit(f"警告: 录音过程中发生IO错误: {str(e)}")
                    # 尝试继续录音
                    continue
                    
            # 录音结束原因
            if time.time() - start_time >= MAX_RECORDING_TIME:
                self.messageSignal.emit(f"调试: 达到最大录音时间限制 {MAX_RECORDING_TIME} 秒，停止录音")
            else:
                self.messageSignal.emit(f"调试: 检测到足够的静默，停止录音")
                
        except Exception as e:
            self.messageSignal.emit(f"错误: 录音过程中发生异常: {str(e)}")
            # 清理资源
            stream.stop_stream()
            stream.close()
            audio.terminate()
            return ""
            
        # print("停止录音!")
        self.messageSignal.emit(f"{self.name()} 停止录音!")
        self.messageSignal.emit(f"调试: 总共录制了 {frame_count} 帧音频，时长约 {(time.time() - start_time):.2f} 秒")
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # 检查是否有足够的音频数据
        if len(frames) < 10:  # 至少需要一些帧才能进行识别
            self.messageSignal.emit(f"警告: 录制的音频数据太少，无法进行识别")
            return ""

        # 添加识别开始的打印
        self.messageSignal.emit(f"调试: 开始识别语音...")
        rec = KaldiRecognizer(model, RATE)
        rec.SetWords(True)
        str_ret = ""
        
        # 分批处理音频数据以提高识别效果
        buffer_size = 50  # 每批处理的帧数
        for i in range(0, len(frames), buffer_size):
            chunk = b''.join(frames[i:i+buffer_size])
            if rec.AcceptWaveform(chunk):
                result = json.loads(rec.Result())
                if 'text' in result and result['text']:
                    str_ret += result['text'] + " "
                    self.messageSignal.emit(f"调试: 部分识别结果: {result['text']}")

        # 处理最终结果
        result = json.loads(rec.FinalResult())
        if 'text' in result and result['text']:
            str_ret += result['text']
            self.messageSignal.emit(f"调试: 最终识别结果: {result['text']}")
        
        if not str_ret.strip():
            self.messageSignal.emit(f"警告: 未能识别出任何文本")
            
        # 清理结果字符串
        str_ret = " ".join(str_ret.split())
        return str_ret

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

class Pyttsx3SpeakNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.speech'
    NODE_NAME = 'Speak by pyttsx3 '

    def __init__(self):
        super(Pyttsx3SpeakNode, self).__init__()
        # 初始化语音引擎
        self.engine = pyttsx3.init()
        # 设置语音属性（可选）
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[0].id)  # 0通常是英文男声，1可能是英文女声，中文可能需要其他设置
        self.engine.setProperty('rate', 150)  # 语速
        self.add_input('text_in')

        self.add_checkbox('isConversationLoop', text='是否开启对话循环')

    def execute(self):
        """"""
        text = self.input(0).connected_ports()[0].node().text_out
        self.messageSignal.emit(f"{self.name()} Output result: {text}")
        self.engine.say(text)
        self.engine.runAndWait()

        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

        if self.get_property('isConversationLoop'):
            # 遍历前置节点，并保存到一个列表中
            self.get_execution_order(self) # 遍历前置节点，再执行它们


    def get_execution_order(self, obj_node):
        """获取从指定节点开始的下游节点执行顺序（拓扑排序）"""
        visited = set()
        # execution_order = []

        def visit_up(node):
            if node in visited:
                return
            # 首先处理所有上游节点
            for port in node.inputs().values():
                for connected_port in port.connected_ports():
                    visit_up(connected_port.node())
            visited.add(node)
            node.set_messageSignal(self.messageSignal)
            if hasattr(node, 'execute'):
                 node.execute() # 运行节点
        visit_up(obj_node)

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal