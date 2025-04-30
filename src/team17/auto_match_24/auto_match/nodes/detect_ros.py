#!/usr/bin/env python3
# -*- coding: utf-8 -*-

## Author: GentsunCheng
## Date: July, 16, 2024
# Purpose: Ros node to detect objects using yolov5
import os
import cv2
import yolov5
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import platform
import pathlib
import threading
plt = platform.system()
if plt != 'Windows':
    pathlib.WindowsPath = pathlib.PosixPath

class SparkDetect:
    class __results__:
        def __init__(self):
            self.name = []
            self.x = []
            self.y = []
            self.size_x = []
            self.size_y = []
            self.confidence = []
            self.image = None

    def __init__(self, model_path):
        '''
        初始化YOLOv5检测器
        :param model_path: YOLOv5模型文件路径
        '''
        self.model = None
        # COCO数据集的80个类别
        self.coco_names = {
            0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 
            6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 10: 'fire hydrant', 
            11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat', 
            16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear', 
            22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 
            27: 'tie', 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard', 
            32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 
            36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 
            40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 
            46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 
            51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 
            56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table', 
            61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 
            66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven', 70: 'toaster', 
            71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase', 
            76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'
        }
        
        try:
            rospy.loginfo(f"尝试加载模型: {model_path}")
            self.model = yolov5.load(model_path)
            rospy.loginfo("模型加载成功")
            
            # 检查并修正模型的类别映射
            if hasattr(self.model.model, 'names') and self.model.model.names:
                if len(self.model.model.names) != len(self.coco_names):
                    rospy.logwarn(f"模型类别数量({len(self.model.model.names)})与COCO类别数量({len(self.coco_names)})不匹配")
                    rospy.loginfo("使用标准COCO类别映射替换模型类别映射")
                    self.model.model.names = self.coco_names
            else:
                rospy.logwarn("模型没有类别映射，使用标准COCO类别映射")
                self.model.model.names = self.coco_names
            
            # 打印可用的类别ID信息
            available_classes = self.model.model.names
            class_ids = list(available_classes.keys())
            min_id, max_id = min(class_ids), max(class_ids)
            rospy.loginfo(f"模型支持的类别ID范围: [{min_id}, {max_id}]")
            rospy.loginfo("类别ID映射:")
            for class_id, class_name in sorted(available_classes.items()):
                rospy.loginfo(f"  {class_id}: {class_name}")
                
        except Exception as e:
            rospy.logerr(f"加载模型失败:{e}, 开始下载")
            # 下载模型
            url = "https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt"
            stat = os.system("wget " + url + " -O " + model_path)
            if not (os.path.exists(model_path) or stat == 0):
                rospy.logerr("下载模型失败")
                if os.path.exists(model_path):
                    os.remove(model_path)
                raise RuntimeError("模型加载和下载都失败")
            rospy.loginfo("下载模型成功")
            try:
                self.model = yolov5.load(model_path)
                rospy.loginfo("下载的模型加载成功")
            except Exception as e:
                rospy.logerr(f"下载的模型加载失败: {e}")
                raise

    def detect(self, image):
        if self.model is None:
            rospy.logerr("模型未初始化，无法进行检测")
            return None

        # 输入图像合法性检查
        if image is None or not hasattr(image, 'shape'):
            rospy.logerr("输入图像无效: 图像为空或没有shape属性")
            return self.__results__()

        # 检查图像维度和类型
        if len(image.shape) != 3:
            rospy.logerr(f"输入图像维度错误: 期望3维(HxWxC), 实际为{len(image.shape)}维")
            return self.__results__()

        if image.shape[2] != 3:
            rospy.logerr(f"输入图像通道数错误: 期望3通道(RGB), 实际为{image.shape[2]}通道")
            return self.__results__()

        try:
            # 确保图像数据类型正确
            if image.dtype != 'uint8':
                rospy.logwarn(f"输入图像类型为{image.dtype}, 尝试转换为uint8")
                image = image.astype('uint8')
        except Exception as e:
            rospy.logerr(f"图像类型转换失败: {str(e)}")
            return self.__results__()

        '''
        检测图像中的物体
        :param image: 输入图像
        :return: 结果类结构
                  result.name: 物体名称列表
                  result.x: 物体中心点x坐标列表
                  result.y: 物体中心点y坐标列表
                  result.confidence: 物体置信度列表
                  result.image: 检测后的图像
        '''
        # 存储检测结果的列表
        result = self.__results__()
        result.image = image.copy()  # 保存原始图像的副本

        try:
            rospy.logdebug(f"开始推理, 输入图像形状: {image.shape}, 类型: {image.dtype}")
            results = self.model(image, augment=True)
            detections = results.xyxy[0]  # 获取检测结果
            rospy.logdebug(f"模型推理完成, 检测到{len(detections)}个目标")
            
            # 使用YOLOv5内置的渲染功能获取带边界框的图像
            result.image = results.render()[0]
            
            # 遍历检测结果
            for *xyxy, conf, cls in detections:
                try:
                    cls_id = int(cls)
                    # 检查类别ID是否在COCO类别范围内
                    if cls_id not in self.coco_names:
                        rospy.logwarn(f"跳过未知类别ID: {cls_id} (置信度: {conf:.2f})")
                        continue
                    
                    if conf < 0.25:  # 添加置信度阈值
                        rospy.logdebug(f"跳过低置信度检测: 类别={cls_id}, 置信度={conf:.2f}")
                        continue
                        
                    # 计算中心点坐标
                    center_x = int((xyxy[0] + xyxy[2]) / 2)
                    center_y = int((xyxy[1] + xyxy[3]) / 2)
                    # 计算大小
                    size_x = int(xyxy[2] - xyxy[0])
                    size_y = int(xyxy[3] - xyxy[1])
                    
                    # 获取类别名称
                    class_name = self.model.model.names[cls_id]
                    
                    # 存储中心点坐标,物体名称,置信度和图像
                    result.size_x.append(size_x)
                    result.size_y.append(size_y)
                    result.name.append(class_name)
                    result.x.append(center_x)
                    result.y.append(center_y)
                    result.confidence.append(float(conf))
                except Exception as e:
                    rospy.logwarn(f"处理检测结果时出错: {str(e)}, 跳过此检测框")
                    continue

        except Exception as e:
            rospy.logerr(f"检测过程发生错误: {type(e).__name__}")
            rospy.logerr(f"错误详细信息: {str(e)}")
            rospy.logerr(f"图像形状: {image.shape if image is not None else 'None'}")
            import traceback
            rospy.logerr(f"堆栈跟踪:\n{traceback.format_exc()}")
            # 即使发生错误，也返回带有原始图像的结果对象
            pass

        return result


class Detector:
    def __init__(self):
        rospy.loginfo("初始化检测器...")
        self.image_pub = rospy.Publisher("result_image",Image, queue_size=1)
        self.object_pub = rospy.Publisher("/objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()
        
        try:
            self.detector = SparkDetect(os.environ['HOME'] + "/yolov5s.pt")
        except Exception as e:
            rospy.logerr(f"初始化检测器失败: {e}")
            raise
            
        # 等待一段时间确保相机已经准备好
        rospy.sleep(2.0)
        
        # 创建显示窗口
        cv2.namedWindow('YOLOv5 物体检测', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('YOLOv5 物体检测', 640, 480)
        
        # 设置最新检测图像
        self.latest_detection_image = None
        self.image_lock = threading.Lock()
        
        # 开启显示线程
        self.display_thread = threading.Thread(target=self.display_detection_results)
        self.display_thread.daemon = True
        self.display_thread.start()
        
        # 订阅相机话题
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24
        )
        rospy.loginfo("检测器初始化完成")
        
        self.obj_id = {'book': 73, 'bowl': 45}  # 反转键值对以便查找
        self.items = ['book', 'bowl'] 
        
        # 跟踪已经检测到的bowl的位置
        self.detected_bowls = []
        # bowl位置差异阈值，如果超过这个值则认为是新的bowl
        self.bowl_position_threshold = 20

    def display_detection_results(self):
        """单独的线程用于显示检测结果"""
        rate = rospy.Rate(15)  # 15Hz的显示更新速率
        
        while not rospy.is_shutdown():
            try:
                with self.image_lock:
                    if self.latest_detection_image is not None:
                        cv2.imshow('YOLOv5 物体检测', self.latest_detection_image)
                
                key = cv2.waitKey(1)
                if key == 27 or key == ord('q'):  # ESC或q键退出
                    rospy.loginfo("用户关闭了图像显示窗口")
                    break
                    
                rate.sleep()
            except Exception as e:
                rospy.logerr(f"显示线程发生错误: {e}")
        
        cv2.destroyAllWindows()

    def image_cb(self, data):
        objArray = Detection2DArray()
        try:
            rospy.logdebug(f"收到图像消息，时间戳: {data.header.stamp}")
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if cv_image is None:
                rospy.logerr("Failed to convert image message to OpenCV image")
                return
            
            image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            if image is None:
                rospy.logerr("Failed to convert BGR to RGB")
                return

            objArray.header = data.header
            try:
                results = self.detector.detect(image)
                if results is None:
                    rospy.logerr("Detection failed - no results returned")
                    return
                    
                img_bgr = cv2.cvtColor(results.image, cv2.COLOR_RGB2BGR)
                if img_bgr is None:
                    rospy.logerr("Detection returned None image")
                    return

                # 添加检测结果的详细信息打印
                if len(results.name) == 0:
                    rospy.loginfo("未检测到任何物体")
                else:
                    rospy.loginfo(f"\n检测到 {len(results.name)} 个物体:")
                    rospy.loginfo("-" * 50)
                    rospy.loginfo(f"{'类别':<15}{'置信度':<10}{'坐标 (x, y)':<20}{'大小 (w, h)'}")
                    rospy.loginfo("-" * 50)
                    
                    for i in range(len(results.name)):
                        name = results.name[i]
                        conf = results.confidence[i]
                        x, y = results.x[i], results.y[i]
                        w, h = results.size_x[i], results.size_y[i]
                        rospy.loginfo(f"{name:<15}{conf:.2f}      ({x:>4}, {y:>4})      ({w:>4}, {h:>4})")

                # 更新最新检测图像用于显示
                with self.image_lock:
                    self.latest_detection_image = img_bgr.copy()

                # 记录发布到话题的物体数量
                published_objects = 0
                
                # 临时保存当前帧检测到的bowl位置，用于更新self.detected_bowls
                current_frame_bowls = []
                
                for i in range(len(results.name)):
                    name = results.name[i]
                    
                    # 跳过非目标物体
                    if name not in self.items:
                        continue
                    
                    # 应用置信度阈值
                    if name == 'bowl' and results.confidence[i] < 0.2:
                        continue
                    if name != 'bowl' and results.confidence[i] < 0.5:
                        continue
                    
                    # 获取当前物体的位置
                    center_x = int(results.x[i])
                    center_y = int(results.y[i])
                    
                    # 特殊处理bowl物体
                    if name == 'bowl':
                        # 检查是否为新的bowl
                        is_new_bowl = True
                        
                        for existing_bowl in self.detected_bowls:
                            # 计算与已存在bowl的距离
                            dx = abs(center_x - existing_bowl[0])
                            dy = abs(center_y - existing_bowl[1])
                            
                            # 如果距离小于阈值，认为是同一个bowl
                            if dx <= self.bowl_position_threshold and dy <= self.bowl_position_threshold:
                                is_new_bowl = False
                                # 将当前检测到的bowl位置记录到当前帧bowls列表中
                                current_frame_bowls.append((center_x, center_y))
                                rospy.logdebug(f"检测到已知bowl: ({center_x}, {center_y}), 与已存在bowl({existing_bowl[0]}, {existing_bowl[1]})距离: dx={dx}, dy={dy}")
                                break
                        
                        # 如果是新的bowl，添加到已检测bowl列表
                        if is_new_bowl:
                            self.detected_bowls.append((center_x, center_y))
                            current_frame_bowls.append((center_x, center_y))
                            rospy.loginfo(f"检测到新的bowl: ({center_x}, {center_y})")
                    
                    # 为所有符合条件的物体创建Detection2D消息
                    published_objects += 1
                    obj = Detection2D()
                    obj.header = data.header
                    obj_hypothesis = ObjectHypothesisWithPose()
                    
                    # 对于自定义物体ID使用手动映射，对于其他类别使用COCO ID
                    if name in self.obj_id:
                        obj_hypothesis.id = int(self.obj_id[name])
                    else:
                        # 遍历模型的类别名称找到对应的ID
                        for cls_id, cls_name in self.detector.model.model.names.items():
                            if cls_name == name:
                                obj_hypothesis.id = int(cls_id)
                                break
                    
                    obj_hypothesis.score = results.confidence[i]
                    obj.results.append(obj_hypothesis)
                    obj.bbox.size_y = int(results.size_y[i])
                    obj.bbox.size_x = int(results.size_x[i])
                    obj.bbox.center.x = center_x
                    obj.bbox.center.y = center_y
                    objArray.detections.append(obj)
                    rospy.loginfo(f"发布物体到话题: {name}, 置信度: {results.confidence[i]:.2f}, 位置: ({center_x}, {center_y})")
                
                # 更新已检测bowl列表，只保留当前帧中存在的bowl
                if len(current_frame_bowls) > 0:
                    self.detected_bowls = current_frame_bowls
                
                if published_objects > 0:
                    rospy.loginfo(f"总共发布了 {published_objects} 个物体到 /objects 话题")
                    rospy.loginfo(f"当前跟踪的bowl数量: {len(self.detected_bowls)}")
                else:
                    rospy.loginfo("没有物体被发布到话题 (可能是置信度低于阈值或不在目标列表中)")

            except Exception as e:
                rospy.logerr(f"Detection error: {str(e)}")
                img_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                
            if img_bgr is not None:  # Only process if we have a valid image
                try:
                    image_out = self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")
                    image_out.header = data.header
                    self.image_pub.publish(image_out)
                except CvBridgeError as e:
                    rospy.logerr(f"Failed to convert image back to ROS message: {str(e)}")
            
            self.object_pub.publish(objArray)
            
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert ROS message to OpenCV image: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Unexpected error in image callback: {str(e)}")


if __name__=='__main__':
    rospy.init_node('detector_node')
    rospy.loginfo("====================================================")
    rospy.loginfo("              启动YOLOv5物体检测节点                 ")
    rospy.loginfo("====================================================")
    rospy.loginfo("窗口操作:")
    rospy.loginfo("  - 按ESC或q键关闭窗口")
    rospy.loginfo("  - 检测结果同时发布到/objects话题")
    rospy.loginfo("====================================================")
    
    obj=Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    
    # 确保窗口关闭
    cv2.destroyAllWindows()