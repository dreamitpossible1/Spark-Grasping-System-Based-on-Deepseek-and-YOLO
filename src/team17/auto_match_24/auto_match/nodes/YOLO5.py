import torch
import cv2
import numpy as np
from PIL import Image
import os
import rospy
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge

# 加载模型
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# 查看模型支持的类别
print("模型支持的类别:")
print(model.names)

# 设置固定的结果保存路径
RESULTS_DIR = 'D:/CUHKSZ/course/Robot Manipulation/results'

# 创建结果目录（如果不存在）
def setup_results_dir():
    if not os.path.exists(RESULTS_DIR):
        # 如果目录不存在，创建它
        os.makedirs(RESULTS_DIR)
    print(f"结果将保存到: {RESULTS_DIR}")

# 显示检测结果的详细信息
def display_detection_details(results):
    # 获取检测结果
    detections = results.xyxy[0]  # 第一张图像的检测结果
    
    if len(detections) == 0:
        print("未检测到任何物体")
        return
    
    print(f"\n检测到 {len(detections)} 个物体:")
    print("-" * 50)
    print(f"{'类别':<15}{'置信度':<10}{'坐标 (x1, y1, x2, y2)'}")
    print("-" * 50)
    
    for det in detections:
        x1, y1, x2, y2, conf, cls = det.tolist()
        class_name = model.names[int(cls)]
        print(f"{class_name:<15}{conf:.2f}      ({int(x1)}, {int(y1)}, {int(x2)}, {int(y2)})")

# 从ROS摄像头进行实时检测
def detect_from_ros_camera(topic_name="/camera/rgb/image_raw"):
    print(f"正在订阅ROS话题: {topic_name}")
    
    # 初始化ROS节点
    rospy.init_node('yolo_detector', anonymous=True)
    
    # 创建CvBridge对象，用于将ROS图像转换为OpenCV图像
    bridge = CvBridge()
    
    # 创建结果发布者（可选，用于发布检测结果）
    result_pub = rospy.Publisher('/yolo/detection_result', RosImage, queue_size=1)
    
    # 帧计数器
    frame_count = 0
    
    # 最新图像
    latest_image = None
    
    # 回调函数，处理接收到的图像
    def image_callback(ros_image):
        nonlocal latest_image, frame_count
        frame_count += 1
        
        try:
            # 将ROS图像转换为OpenCV图像
            cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
            latest_image = cv_image
        except Exception as e:
            print(f"转换图像时出错: {e}")
    
    # 订阅摄像头话题
    rospy.Subscriber(topic_name, RosImage, image_callback)
    
    print("已开始接收ROS摄像头数据，按Ctrl+C退出...")
    
    # 创建窗口
    cv2.namedWindow('YOLOv5 ROS Detection', cv2.WINDOW_NORMAL)
    
    rate = rospy.Rate(10)  # 10Hz
    
    try:
        while not rospy.is_shutdown():
            if latest_image is not None:
                # 将OpenCV图像转换为PIL图像
                img = Image.fromarray(cv2.cvtColor(latest_image, cv2.COLOR_BGR2RGB))
                
                # 执行推理
                results = model(img)
                
                # 将结果渲染到图像上
                img_with_boxes = np.array(results.render()[0])
                
                # 转回OpenCV格式以显示
                img_with_boxes = cv2.cvtColor(img_with_boxes, cv2.COLOR_RGB2BGR)
                
                # 显示结果
                cv2.imshow('YOLOv5 ROS Detection', img_with_boxes)
                
                # 每隔30帧保存一次图像
                if frame_count % 30 == 0:
                    save_path = os.path.join(RESULTS_DIR, f"ros_camera_frame_{frame_count}.jpg")
                    cv2.imwrite(save_path, img_with_boxes)
                    print(f"ROS摄像头帧已保存到: {save_path}")
                    
                    # 显示检测结果的详细信息
                    display_detection_details(results)
                
                # 尝试发布结果图像（可选）
                try:
                    result_msg = bridge.cv2_to_imgmsg(img_with_boxes, "bgr8")
                    result_pub.publish(result_msg)
                except Exception as e:
                    print(f"发布结果图像时出错: {e}")
            
            # 按'q'退出
            if cv2.waitKey(1) == ord('q'):
                break
                
            rate.sleep()
    
    except KeyboardInterrupt:
        print("用户中断，退出程序")
    
    finally:
        cv2.destroyAllWindows()
        print("已关闭ROS摄像头检测")

# 主函数
if __name__ == "__main__":
    # 设置结果目录
    setup_results_dir()
    
    # 直接使用ROS摄像头
    print("启动ROS摄像头检测...")
    topic_name = input("请输入ROS摄像头话题名称 (默认为/camera/rgb/image_raw): ").strip()
    if not topic_name:
        topic_name = "/camera/rgb/image_raw"
    detect_from_ros_camera(topic_name)