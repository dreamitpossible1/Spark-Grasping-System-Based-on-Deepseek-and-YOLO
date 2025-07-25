## Project Title
Spark Grasping System for RB3 Gen2 Based on Deepseek and YOLO


## Overview
This project is dedicated to developing a visual grasping system for RB3 Gen2, leveraging the capabilities of Deepseek and YOLO. RB3 Gen2 serves as the core hardware platform, providing advanced motion control and sensor features. Spark is utilized for efficient data processing and distributed computing, enabling real - time analysis of depth images. YOLO is employed for accurate object detection and recognition, allowing the robot arm to perceive and interact with 3D environments effectively. This combination enhances the robot's ability to navigate and perform tasks in complex environments.

## Quick Start with QualComm RB3 gen2
Download the precompiled package for RB3 Gen2：

wget https://artifacts.codelinaro.org/artifactory/qli-ci/flashable-binaries/qirpsdk/qcs6490-rb3gen2-vision-kit/arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip

Use the following command to unzip the package:

unzip arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip

The specific content is as follows:[QualComm Intelligent Robotics Product SDK Quick Start]([QIRP User Guide - Qualcomm® Linux Documentation](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/quick-start_3.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm Intelligent Robotics Product (QIRP) SDK)

## 功能包说明

* ***src*** : Spark的源代码，包括底层配置，硬件驱动，和各个应用功能包等。
* ***doc*** : 软硬件依赖包。

## 系统要求

* System:	Ubuntu 20.04+
* ROS Version:	noetic(Desktop-Full Install)

## 技术方案概述

本项目采用双环境架构设计：

### ROS环境 (Python 3.8)
- **机械臂驱动**：修改原代码为可异步实现，支持参数写入和机械臂状态的接收（共用串口，异步收发）
- **导航与SLAM建图**：建图期间可标定工作区域，本项目设定了两个工作区域
- **视觉识别**：使用YOLOv5s实现对物体的检测、分割以及抓取点的获取
- **标定系统**：机械臂和相机的标定采用两层标定法（在两个不同的z轴高度，提高精度）

### LLM控制环境 (Python 3.12)
- 基于NodeGraphQt开发，实现可视化节点编辑和工作流搭建
- 使用LLM模型(deepseek)，根据设定的prompt完成用户命令的传达和指令的生成
- 支持文本输入和语音输入两种交互方式

### 系统集成
- 两个环境通过socket建立本机通信，规避环境差异带来的代码兼容性问题

## 使用说明

### 机械臂控制
仅启动机械臂控制：
```bash
roslaunch swiftpro swiftpro_unified.launch
```

机械臂位置控制：
```bash
rostopic pub /position_write_topic swiftpro/position "x: 100.0
y: 100.0
z: 100.0
speed: 1000" -1
```

控制吸盘：
```bash
rostopic pub /pump_topic swiftpro/status "status: 1" -1
```

### 两点间导航功能
启动导航功能：
```bash
roslaunch auto_match robot_nav.launch
```

导航到指定区域：
```bash
# 导航到放置区
rostopic pub /navigation_command std_msgs/String "data: 'goto placement_area'" -1

# 导航到分拣区
rostopic pub /navigation_command std_msgs/String "data: 'goto sorting_area'" -1
```

注意：启动导航后需先通过2D pose estimation标定机器人所在位置，然后系统将自动避障导航到工作区域。

### 识别与抓放流程
启动抓取功能：
```bash
roslaunch auto_match grasp_standalone.launch
```
注意：用手电打光，当识别到三个物体时开始。

开始抓取：
```bash
rostopic pub /grasp_cmd std_msgs/String "grasp" -1
```

Object列表更新：
```bash
rostopic pub /reset_bowl_list std_msgs/String "data: 'reset'" -1
```

### 辅助控制
手动操作：
```bash
rosrun spark_teleop keyboard_control.sh
```

## LLMspark 使用说明

### 创建与管理虚拟环境
创建虚拟环境：
```bash
python3.12 -m venv ~/LLMspark
```

激活虚拟环境：
```bash
source ~/LLMspark/bin/activate
```

退出虚拟环境：
```bash
deactivate
```

### Qt环境配置
```bash
export QT_QPA_PLATFORM_PLUGIN_PATH=/path/to/qt6/plugins
```

### 启动服务
启动ROS服务端：
```bash
python3 src/team17/auto_match_24/auto_match/nodes/start_robot_server.py
```

启动LLM控制端：
```bash
python -m src.team17.graph_executer_controller.main
```

### LLM界面选择
- **Text input**：文本输入方式
- **LLM Robot Control**：机器人控制接口
- **Speach recognition by VOSK**：使用本地中文语音转文字识别的小模型

注：上述topic命令可通过本地语音识别或文本输入传达给LLM，由LLM解析后执行相应操作。
