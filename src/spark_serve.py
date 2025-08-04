from openai import OpenAI
from prompt_toolkit import prompt
import time
import socket
import threading
import sys
import select
import queue

# 初始化客户端
client = OpenAI(
    api_key="sk-b765ec4ae6e64e8b8e754ffa4abfe957",
    base_url="https://api.deepseek.com"
)

# 物品中英文对照表
OBJECT_TRANSLATIONS = {
    # 常见物品
    "杯子": "cup",
    "水杯": "cup",
    "茶杯": "cup",
    "碗": "bowl",
    "盒子": "box",
    "鼠标": "mouse",
    "键盘": "keyboard",
    "手机": "cell.phone",
    "笔": "pen",
    "铅笔": "pencil",
    "书": "book",
    "瓶子": "bottle",
    "水瓶": "bottle",
    "眼镜": "glasses",
    "钥匙": "key",
    "遥控器": "remote",
    "纸": "paper",
    "剪刀": "scissors",
    "尺子": "ruler",
    "包": "bag",
    "钱包": "wallet",
    "手表": "watch",
    "充电器": "charger",
    "耳机": "headphones",
    "相机": "camera",
    "餐盘": "plate",
    "叉子": "fork",
    "勺子": "spoon",
    "刀": "knife",
    # 反向映射（英文到中文）
    "cup": "杯子",
    "bowl": "碗",
    "box": "盒子",
    "mouse": "鼠标",
    "keyboard": "键盘",
    "phone": "手机",
    "pen": "笔",
    "pencil": "铅笔",
    "book": "书",
    "bottle": "瓶子",
    "glasses": "眼镜",
    "key": "钥匙",
    "remote": "遥控器",
    "paper": "纸",
    "scissors": "剪刀",
    "ruler": "尺子",
    "bag": "包",
    "wallet": "钱包",
    "watch": "手表",
    "charger": "充电器",
    "headphones": "耳机",
    "camera": "相机",
    "plate": "餐盘",
    "fork": "叉子",
    "spoon": "勺子",
    "knife": "刀"
}

# UDP 服务器设置
UDP_IP = "192.168.1.26"
UDP_PORT = 9090
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((UDP_IP, UDP_PORT))
print(f"UDP 服务器启动，等待客户端消息... ({UDP_IP}:{UDP_PORT})")
client_address = None

# 等待客户端连接
while client_address is None:
    print("等待客户端连接...")
    data, client_address = server_socket.recvfrom(1024)
    print(f"收到客户端 {client_address} 的消息：{data.decode()}")

# 初始化对话历史
messages = [
    {"role": "system", "content": """你是一个智能助手，负责帮助用户找寻和拿取物品。你需要：
1. 当用户要求拿取某个物品时，回复表示正在为用户寻找该物品
2. 回答要自然、友好，像一个真实的助手
3. 不要提及detected_objects.txt文件或检测状态
4. 直接告诉用户正在寻找，并会尽力帮助拿取

请用中文对话。"""}
]

def translate_to_english(chinese_word):
    """将中文物品名转换为英文"""
    return OBJECT_TRANSLATIONS.get(chinese_word)

def translate_to_chinese(english_word):
    """将英文物品名转换为中文"""
    return OBJECT_TRANSLATIONS.get(english_word, english_word)

def read_detected_objects():
    """读取当前检测到的物品列表，包含坐标信息"""
    try:
        with open("detected_objects.txt", "r") as f:
            lines = [line.strip() for line in f.readlines()]
            objects_with_coords = []
            for line in lines:
                if ": Center(" in line:
                    # 解析格式: "mouse: Center(713, 132)"
                    parts = line.split(": Center(")
                    if len(parts) == 2:
                        obj_name = parts[0]
                        coord_part = parts[1].rstrip(")")
                        objects_with_coords.append(f"{obj_name}: Center({coord_part})")
                    else:
                        objects_with_coords.append(line)
                else:
                    objects_with_coords.append(line)
            return objects_with_coords
    except FileNotFoundError:
        return []

def find_object_in_detected(chinese_name, detected_objects_english):
    """检查用户要找的物品是否在检测列表中"""
    if not chinese_name:
        return False
    english_name = translate_to_english(chinese_name)
    if not english_name:
        return False
    return english_name.lower() in [obj.lower() for obj in detected_objects_english]

# 全局变量用于跟踪已找到的物品
found_items = set()
searching_items = set()

def monitor_detected_objects():
    """持续监控detected_objects.txt文件"""
    global found_items, searching_items
    
    while True:
        try:
            with open("detected_objects.txt", "r") as f:
                current_lines = [line.strip() for line in f.readlines()]
            
            # 提取物品名称（不包含坐标）
            current_objects_english = []
            for line in current_lines:
                if ": Center(" in line:
                    obj_name = line.split(": Center(")[0]
                    current_objects_english.append(obj_name)
                else:
                    current_objects_english.append(line)
            
            # 检查正在寻找的物品是否被检测到
            for chinese_item in list(searching_items):
                english_item = translate_to_english(chinese_item)
                if english_item and english_item in current_objects_english:
                    if chinese_item not in found_items:
                        # 查找对应物品的完整坐标信息
                        coord_info = None
                        for line in current_lines:
                            if line.startswith(english_item + ": Center("):
                                coord_info = line
                                break
                        
                        # 打印找到的物品和坐标
                        if coord_info:
                            print(f"\n✓ 寻找到 {chinese_item}！坐标: {coord_info}")
                            # 重新发送完整指令给客户端
                            if client_address:
                                server_socket.sendto(coord_info.encode(), client_address)
                                print(f"已重新发送完整指令 '{coord_info}' 给客户端 {client_address}")
                        else:
                            print(f"\n✓ 寻找到 {chinese_item}！")
                        
                        sys.stdout.flush()
                        found_items.add(chinese_item)
                        searching_items.remove(chinese_item)
                else:
                    if chinese_item in found_items:
                        found_items.remove(chinese_item)
                        searching_items.add(chinese_item)
                        
        except FileNotFoundError:
            pass
        
        time.sleep(0.5)

# 启动监控线程
monitor_thread = threading.Thread(target=monitor_detected_objects, daemon=True)
monitor_thread.start()

# 新增：用于ready请求的队列
ready_request_queue = queue.Queue()

def udp_ready_listener():
    """监听客户端ready:xxx请求，收到后查找并发送坐标"""
    while True:
        data, addr = server_socket.recvfrom(1024)
        msg = data.decode()
        if msg.startswith("ready:"):
            object_name = msg.split(":", 1)[1]
            print(f"收到客户端到达目标点请求: {object_name}")
            # 查找detected_objects.txt
            found = False
            while not found:
                detected = read_detected_objects()
                for line in detected:
                    if line.startswith(object_name + ": Center("):
                        server_socket.sendto(line.encode(), addr)
                        print(f"已发送带坐标指令 '{line}' 给客户端 {addr}")
                        found = True
                        break
                if not found:
                    time.sleep(0.2)

# 启动UDP ready监听线程
udp_ready_thread = threading.Thread(target=udp_ready_listener, daemon=True)
udp_ready_thread.start()

# 用户输入主循环不再主动发指令，只用于对话
while True:
    # 获取用户输入
    try:
        user_input = input("\n用户：")
    except KeyboardInterrupt:
        print("\n对话结束。")
        break
    # 退出机制
    if user_input.lower() in ["退出", "exit"]:
        print("对话结束。")
        break
    # 读取当前检测到的物品（包含坐标）
    try:
        with open("detected_objects.txt", "r") as f:
            current_lines = [line.strip() for line in f.readlines()]
        # 提取物品名称用于显示
        current_objects_english = []
        for line in current_lines:
            if ": Center(" in line:
                obj_name = line.split(": Center(")[0]
                current_objects_english.append(obj_name)
            else:
                current_objects_english.append(line)
    except FileNotFoundError:
        current_lines = []
        current_objects_english = []
    # 转换为中文显示
    current_objects_chinese = [translate_to_chinese(obj) for obj in current_objects_english]
    # 构建包含当前物品信息的用户消息
    context_message = f"""用户的请求是：{user_input}\n当前检测到的物品有：{', '.join(current_objects_chinese) if current_objects_chinese else '没有检测到任何物品'}"""
    # 添加用户消息到历史
    messages.append({"role": "user", "content": context_message})
    try:
        # 获取模型响应
        response = client.chat.completions.create(
            model="deepseek-chat",
            messages=messages,
            stream=False
        )
        # 解析回复内容
        assistant_response = response.choices[0].message.content
        # 添加助手回复到历史
        messages.append({"role": "assistant", "content": assistant_response})
        # 打印回复
        print("\n助手：", assistant_response)

        # 新增：用户输入时立即向客户端发送物品名（如mouse、cup等）
        found_item = None
        for name in OBJECT_TRANSLATIONS:
            if name in user_input:
                found_item = name
                break
        if found_item:
            english_name = translate_to_english(found_item)
            if client_address:
                server_socket.sendto(english_name.encode(), client_address)
                print(f"已发送指令 '{english_name}' 给客户端 {client_address}")

        # 不再主动发带坐标指令，等待user端ready:xxx请求
    except Exception as e:
        print(f"发生错误：{str(e)}")
        break
