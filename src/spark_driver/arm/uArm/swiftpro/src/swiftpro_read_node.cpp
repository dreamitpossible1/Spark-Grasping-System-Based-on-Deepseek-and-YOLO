/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 *		   David Long <xiaokun.long@ufactory.cc>	   
 */
 
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <string>
#include <algorithm> // 用于 std::remove_if
#include <swiftpro/SwiftproState.h>

serial::Serial _serial;				// serial object
float position[4] = {0.0};			// 3 cartesian coordinates: x, y, z(mm) and 1 angle(degree)
char  strdata[2048];				// global variables for handling string

// 函数前向声明
void requestCurrentPosition();
bool parseJointAngles(const std::string &raw_data, float angles[4]);
void calculatePositionFromAngles(float angle[4], float position[3]);
void requestJointAngles();

// 请求并打印当前位置
void requestCurrentPosition()
{
    ROS_INFO("Requesting current position directly...");
    _serial.write("M2200\r\n"); // 请求当前位置
    ros::Duration(0.1).sleep(); // 等待响应
    
    if (_serial.available()) {
        std::string raw_position = _serial.read(_serial.available());
        ROS_INFO("Raw position response: [%s]", raw_position.c_str());
    } else {
        ROS_WARN("No response from position request");
    }
}

void handlestr()
{
	ROS_INFO("Handling string: %s", strdata);
	char  *pch = strtok(strdata, " ");
	float value[8];
	int   index = 0;

	while (pch != NULL && index < 5)
	{
		ROS_INFO("Token: %s", pch);
		value[index] = atof(pch+1);
		ROS_INFO("Parsed value[%d] = %f", index, value[index]);
		pch = strtok(NULL, " ");
		index++;
	}
	
	ROS_INFO("Before assignment: position = [%f, %f, %f, %f]", position[0], position[1], position[2], position[3]);
	position[0] = value[1];
	position[1] = value[2];
	position[2] = value[3];
	position[3] = value[4];
	ROS_INFO("After assignment: position = [%f, %f, %f, %f]", position[0], position[1], position[2], position[3]);
}


void handlechar(char c)
{
	static int index = 0;

	switch(c)
	{
		case '\r':
			ROS_DEBUG("Received carriage return");
			break;	

		case '\n':
			ROS_DEBUG("Received newline, processing buffer...");
			strdata[index] = '\0';
			handlestr();
			index = 0;
			break;

		default:
			strdata[index++] = c;
			if (index == 1 || index % 50 == 0) {
				ROS_DEBUG("Adding to buffer: %c (index=%d)", c, index);
			}
			break;
	}
}

// 新增：直接从原始返回字符串解析位置信息
void parsePositionFromRaw(const std::string &raw_data)
{
    ROS_INFO("Attempting to parse position from raw data: [%s]", raw_data.c_str());
    
    // 常见格式: "ok X:123.45 Y:67.89 Z:10.11 A:45"
    // 或者: "@3 X:123.45 Y:67.89 Z:10.11 A:45"
    
    float temp_position[4] = {0.0, 0.0, 0.0, 0.0};
    bool position_updated = false;
    
    size_t x_pos = raw_data.find("X:");
    size_t y_pos = raw_data.find("Y:");
    size_t z_pos = raw_data.find("Z:");
    size_t a_pos = raw_data.find("A:");
    
    if (x_pos != std::string::npos && y_pos != std::string::npos && 
        z_pos != std::string::npos) {
        
        try {
            // 从 "X:123.45" 中提取 "123.45"
            std::string x_str = raw_data.substr(x_pos + 2, y_pos - (x_pos + 2));
            std::string y_str = raw_data.substr(y_pos + 2, z_pos - (y_pos + 2));
            
            // Z后面可能是A或者是结尾
            std::string z_str;
            if (a_pos != std::string::npos) {
                z_str = raw_data.substr(z_pos + 2, a_pos - (z_pos + 2));
            } else {
                z_str = raw_data.substr(z_pos + 2);
            }
            
            // 移除可能的空格
            x_str.erase(std::remove_if(x_str.begin(), x_str.end(), ::isspace), x_str.end());
            y_str.erase(std::remove_if(y_str.begin(), y_str.end(), ::isspace), y_str.end());
            z_str.erase(std::remove_if(z_str.begin(), z_str.end(), ::isspace), z_str.end());
            
            ROS_INFO("Extracted values: X=[%s], Y=[%s], Z=[%s]", 
                     x_str.c_str(), y_str.c_str(), z_str.c_str());
            
            // 转换为浮点数
            temp_position[0] = std::stof(x_str);
            temp_position[1] = std::stof(y_str);
            temp_position[2] = std::stof(z_str);
            
            // 如果有角度值，也解析
            if (a_pos != std::string::npos) {
                std::string a_str = raw_data.substr(a_pos + 2);
                a_str.erase(std::remove_if(a_str.begin(), a_str.end(), ::isspace), a_str.end());
                temp_position[3] = std::stof(a_str);
            }
            
            position_updated = true;
        } catch (const std::exception& e) {
            ROS_ERROR("Error parsing position values: %s", e.what());
        }
    }
    
    // 如果解析成功，更新位置数组
    if (position_updated) {
        ROS_INFO("Successfully parsed position: x=%.2f, y=%.2f, z=%.2f, angle=%.2f", 
                 temp_position[0], temp_position[1], temp_position[2], temp_position[3]);
        
        // 只有当解析出的值不全为0时才更新位置
        if (temp_position[0] != 0.0 || temp_position[1] != 0.0 || temp_position[2] != 0.0) {
            position[0] = temp_position[0];
            position[1] = temp_position[1];
            position[2] = temp_position[2];
            position[3] = temp_position[3];
            ROS_INFO("UPDATED position array with parsed values");
        }
    }
}

// 请求关节角度
void requestJointAngles()
{
    ROS_INFO("Requesting current joint angles...");
    _serial.write("M2201\r\n"); // 请求当前关节角度
    ros::Duration(0.1).sleep(); // 等待响应
    
    if (_serial.available()) {
        std::string raw_angles = _serial.read(_serial.available());
        ROS_INFO("Raw joint angles response: [%s]", raw_angles.c_str());
        
        // 尝试解析关节角度数据
        // 机械臂可能返回类似 "ok N0:90.00 N1:90.00 N2:0.00 N3:0.00" 的数据
        float joint_angles[4] = {0.0};
        bool parsed = parseJointAngles(raw_angles, joint_angles);
        
        if (parsed) {
            ROS_INFO("Parsed joint angles: [%.2f, %.2f, %.2f, %.2f]", 
                     joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3]);
            
            // 尝试使用关节角度通过正运动学计算笛卡尔坐标
            float fk_position[3] = {0.0};
            calculatePositionFromAngles(joint_angles, fk_position);
            
            ROS_INFO("Position from FK: [%.2f, %.2f, %.2f]", 
                     fk_position[0], fk_position[1], fk_position[2]);
            
            // 如果当前位置全是0，并且正运动学计算出的位置有效，则使用计算结果
            if (position[0] == 0.0 && position[1] == 0.0 && position[2] == 0.0 &&
                (fk_position[0] != 0.0 || fk_position[1] != 0.0 || fk_position[2] != 0.0)) {
                
                ROS_INFO("Updating position from FK calculation");
                position[0] = fk_position[0];
                position[1] = fk_position[1];
                position[2] = fk_position[2];
            }
        }
    } else {
        ROS_WARN("No response from joint angles request");
    }
}

// 从原始数据中解析关节角度
bool parseJointAngles(const std::string &raw_data, float angles[4])
{
    size_t n0_pos = raw_data.find("N0:");
    size_t n1_pos = raw_data.find("N1:");
    size_t n2_pos = raw_data.find("N2:");
    size_t n3_pos = raw_data.find("N3:");
    
    if (n0_pos != std::string::npos && n1_pos != std::string::npos && 
        n2_pos != std::string::npos && n3_pos != std::string::npos) {
        
        try {
            // 提取角度值
            std::string n0_str = raw_data.substr(n0_pos + 3, n1_pos - (n0_pos + 3));
            std::string n1_str = raw_data.substr(n1_pos + 3, n2_pos - (n1_pos + 3));
            std::string n2_str = raw_data.substr(n2_pos + 3, n3_pos - (n2_pos + 3));
            std::string n3_str = raw_data.substr(n3_pos + 3);
            
            // 移除空格
            n0_str.erase(std::remove_if(n0_str.begin(), n0_str.end(), ::isspace), n0_str.end());
            n1_str.erase(std::remove_if(n1_str.begin(), n1_str.end(), ::isspace), n1_str.end());
            n2_str.erase(std::remove_if(n2_str.begin(), n2_str.end(), ::isspace), n2_str.end());
            n3_str.erase(std::remove_if(n3_str.begin(), n3_str.end(), ::isspace), n3_str.end());
            
            // 转换为浮点数
            angles[0] = std::stof(n0_str);
            angles[1] = std::stof(n1_str);
            angles[2] = std::stof(n2_str);
            angles[3] = std::stof(n3_str);
            
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("Error parsing joint angles: %s", e.what());
        }
    }
    
    return false;
}

// 使用正运动学计算位置
// 这里使用 swiftpro_moveit_node.cpp 中的正运动学算法
#define MATH_PI                3.141592653589793238463
#define MATH_TRANS             57.2958    
#define MATH_L1                106.6
#define MATH_L2                13.2
#define MATH_LOWER_ARM         142.07
#define MATH_UPPER_ARM         158.81

void calculatePositionFromAngles(float angle[4], float position[3])
{
    double stretch = MATH_LOWER_ARM * cos(angle[1] / MATH_TRANS) 
                   + MATH_UPPER_ARM * cos(angle[2] / MATH_TRANS) + MATH_L2 + 56.55;

    double height = MATH_LOWER_ARM * sin(angle[1] / MATH_TRANS) 
                  - MATH_UPPER_ARM * sin(angle[2] / MATH_TRANS) + MATH_L1;
    
    position[0] = stretch * sin(angle[0] / MATH_TRANS);
    position[1] = -stretch * cos(angle[0] / MATH_TRANS);
    position[2] = height - 74.55;
}

/* 
 * Node name:
 *	 swiftpro_read_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *   position_read_topic
 */
int main(int argc, char** argv)
{	
	ros::init(argc, argv, "swiftpro_read_node");
	ros::NodeHandle nh;
	swiftpro::SwiftproState swiftpro_state;
	std_msgs::String result;

	ros::Publisher pub = nh.advertise<swiftpro::SwiftproState>("SwiftproState_topic", 1);
	ros::Rate loop_rate(20);

	try
	{
		_serial.setPort("/dev/ttyACM0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		ROS_INFO_STREAM("Port has been open successfully");
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}
	
	if (_serial.isOpen())
	{
		ros::Duration(3.0).sleep();				// wait 3s
		_serial.write("M2019\r\n");				// detach
		ros::Duration(0.5).sleep();				// wait 0.5s
		
		// 测试串口通信
		ROS_INFO("Sending test command to check communication...");
		_serial.write("M2122\r\n");			// Get device information
		ros::Duration(0.5).sleep();
		if (_serial.available()) {
			std::string test_result = _serial.read(_serial.available());
			ROS_INFO("Device information: %s", test_result.c_str());
		} else {
			ROS_WARN("No response from device test command");
		}
		
		// 直接获取当前位置数据
		requestCurrentPosition();
		
		// 获取关节角度并尝试通过正运动学计算位置
		requestJointAngles();
		
		// 尝试取消原有的自动上报功能，再重新启用
		_serial.write("M2120 V0\r\n");		// 停止自动上报
		ros::Duration(0.5).sleep();
		_serial.write("M2120 V0.05\r\n");		// report position per 0.05s
		ROS_INFO_STREAM("Start to report data");
	}
	
	int zero_count = 0;  // 跟踪连续零值的数量
	int iteration_count = 0;
	
	while (ros::ok())							// publish positionesian coordinates
	{
		iteration_count++;
		
		// 每50次循环主动请求一次位置
		if (iteration_count % 50 == 0) {
			requestCurrentPosition();
		}
		
		// 每150次循环请求一次关节角度并尝试通过正运动学计算位置
		if (iteration_count % 150 == 0) {
			requestJointAngles();
		}
		
		if (_serial.available())
		{
			int bytes_available = _serial.available();
			ROS_INFO("Available bytes: %d", bytes_available);
			
			result.data = _serial.read(bytes_available);
			ROS_INFO_STREAM("Read from serial: [" << result.data << "]");
			
			// 1. 尝试使用新的解析方法
			parsePositionFromRaw(result.data);
			
			// 2. 同时使用原来的解析方法作为备用
			for (int i = 0; i < result.data.length(); i++)
				handlechar(result.data.c_str()[i]);

			swiftpro_state.pump = 0;
			swiftpro_state.gripper = 0;
			swiftpro_state.swiftpro_status = 0;
			swiftpro_state.motor_angle1 = 0.0;
			swiftpro_state.motor_angle2 = 0.0;
			swiftpro_state.motor_angle3 = 0.0;
			swiftpro_state.motor_angle4 = position[3];
			swiftpro_state.x = position[0];
			swiftpro_state.y = position[1];
			swiftpro_state.z = position[2];
			
			// 检查是否所有值都是0
			if (position[0] == 0.0 && position[1] == 0.0 && position[2] == 0.0) {
				zero_count++;
				if (zero_count % 20 == 0) {  // 每20次连续零值打印一次警告
					ROS_WARN("Position values all zero for %d consecutive readings", zero_count);
					
					// 连续多次零值，尝试重新获取位置
					if (zero_count % 100 == 0) {
						ROS_WARN("Too many zero readings, trying to reinitialize position reporting...");
						_serial.write("M2120 V0\r\n"); // 停止自动上报
						ros::Duration(0.2).sleep();
						_serial.write("M2120 V0.05\r\n"); // 重新启动自动上报
						ros::Duration(0.2).sleep();
						requestCurrentPosition(); // 直接请求当前位置
						
						// 尝试通过关节角度和正运动学获取位置
						if (zero_count % 150 == 0) {
							requestJointAngles();
						}
					}
				}
			} else {
				if (zero_count > 0) {
					ROS_INFO("Received non-zero position after %d zero readings", zero_count);
					zero_count = 0;
				}
			}
			
			pub.publish(swiftpro_state);
			ROS_INFO("Publishing position: x=%.2f, y=%.2f, z=%.2f, angle=%.2f", 
				position[0], position[1], position[2], position[3]);
		}
		else
		{
			ROS_DEBUG_THROTTLE(5, "No data available from serial port");
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}


