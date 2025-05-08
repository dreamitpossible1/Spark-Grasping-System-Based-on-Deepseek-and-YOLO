# NXROBO Spark
![Spark机器人](http://wiki.ros.org/Robots/Spark?action=AttachFile&do=get&target=spark1.png "Spark Robot")

## 说明

本项目为港中深25春季robot manipulation team17课程项目，此代码基于[***NXROBO Spark***](https://github.com/NXROBO/spark_noetic "NXROBO Spark")官方仓库进行修改，本项目使用 ***GPL3.0*** 协议，***使用本项目时请遵守相关协议***

Spark控制程序[***Spark Control***](https://github.com/GentsunCheng/spark-control "Spark Control")

## 功能包说明

* ***src*** : Spark的源代码，包括底层配置，硬件驱动，和各个应用功能包等。
* ***doc*** : 软硬件依赖包。

## 系统要求

* System:	Ubuntu 20.04+
* ROS Version:	noetic(Desktop-Full Install)

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
```bash
python3 src/team17/auto_match_24/auto_match/nodes/start_robot_server.py
```

```bash
python -m src.team17.graph_executer_controller.main