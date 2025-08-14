## Project Title
Spark Grasping System for RB3 Gen2 Based on Deepseek and YOLO


## Overview
This project delivers a Spark grasping system for the RB3 Gen2, powered by Deepseek and YOLO. Leveraging the RB3 Gen2’s advanced motion control and sensor technologies, the platform establishes a robust foundation. YOLO handles real-time object detection, rapidly identifying target objects within the environment, while Deepseek drives decision-making by analyzing data to formulate optimal grasping strategies. Working in concert, these components enable the robotic arm to execute grasping tasks with both high efficiency and precision. Integration of the Qualcomm® Intelligent Robotics Product SDK ensures seamless interoperability among all system modules and further elevates overall reliability.

<p align="center"> <img src="https://github.com/dreamitpossible1/Spark-Grasping-System-Based-on-Deepseek-and-YOLO/blob/main/pictures/Technical%20flowchart.png" alt="Script Step 1" /> </p>


## Quick Start with QualComm RB3 gen2
Download the precompiled package for RB3 Gen2：

```bash
wget https://artifacts.codelinaro.org/artifactory/qli-ci/flashable-binaries/qirpsdk/qcs6490-rb3gen2-vision-kit/arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip
```

Use the following command to unzip the package:
```bash
unzip arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip
```
 Run Flashing Procedure

```bash
cd <extracted zip directory>/target/qcs6490-rb3gen2-vision-kit/qcom-multimedia-image
<qdl_tool_path>/qdl_2.3.1/QDL_Linux_x64/qdl prog_firehose_ddr.elf rawprogram*.xml patch*.xml
```

Before you start, make sure finish [QualComm Intelligent Robotics Product SDK Quick Start]([QIRP User Guide - Qualcomm® Linux Documentation](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/quick-start_3.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm Intelligent Robotics Product (QIRP) SDK)

## Quick Start with uArm Swift Pro

1. Download [uArm-Python-SDK](https://github.com/uArm-Developer/uArm-Python-SDK.git)

```bash
git clone https://github.com/uArm-Developer/uArm-Python-SDK.git
```
2. Installation
```bash
   python setup.py install
```

## Run the System
Follow these steps to deploy and launch the robot arm system based on depth images:
1. Clone the Repository
```bash
git clone https://github.com/dreamitpossible1/Spark-Grasping-System-Based-on-Deepseek-and-YOLO.git
```

2. Set Permissions and Run Script
```bash
cd Spark-Grasping-System-Based-on-Deepseek-and-YOLO/src
chmod +x run_label.sh
./run_label.sh
```

3. Set Permissions and Launch the Spark
```bash
chmod +x onekey.sh
./onekey.sh
```

4. Launch the DeepSeek
```bash
python spark_serve.py
```

5. Control the grabbing of the robotic arm
```bash
python spark_uarm.py
python spark_user.py
```

<p align="center"> <img src="https://github.com/dreamitpossible1/Spark-Grasping-System-Based-on-Deepseek-and-YOLO/blob/main/pictures/1.jpg" alt="Script Step 1" /> </p> <p align="center"> <img src="https://github.com/dreamitpossible1/Spark-Grasping-System-Based-on-Deepseek-and-YOLO/blob/main/pictures/9.png" alt="Script Step 2" /> </p>

📹 Results Display

[Display](https://vcnkc4612xpy.feishu.cn/file/EmUZbfb5Lo0DyTxsHyPc8y3dn0f?from=from_copylink)

## Reference

- [Qualcomm Linux](https://www.qualcomm.com/developer/software/qualcomm-linux)

- [QualComm Intelligent Robotics Product SDK](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/introduction_1.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm Intelligent Robotics Product (QIRP) SDK)
