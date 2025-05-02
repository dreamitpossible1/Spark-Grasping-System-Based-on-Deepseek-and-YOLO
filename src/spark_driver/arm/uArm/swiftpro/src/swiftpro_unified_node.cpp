/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 *         David Long <xiaokun.long@ufactory.cc>
 * Modified by: Team17 <team17@robot.com>
 */
 
#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <swiftpro/SwiftproState.h>
#include <swiftpro/status.h>
#include <swiftpro/position.h>
#include <swiftpro/angle4th.h>
#include <swiftpro/angle3rd.h>
#include <swiftpro/angle2nd.h>
#include <swiftpro/angle1st.h>
#include <swiftpro/buzzer.h>
#include <thread>
#include <mutex>

serial::Serial _serial;                // serial object
swiftpro::SwiftproState current_state; // current robot state
std::mutex serial_mutex;               // mutex for serial port access

float position[4] = {0.0};             // 3 cartesian coordinates: x, y, z(mm) and 1 angle(degree)
char  strdata[2048];                   // buffer for handling string data
bool  is_reporting = true;             // flag for position reporting

// Position string parsing helper functions
void handlestr()
{
    char  *pch = strtok(strdata, " ");
    float value[8];
    int   index = 0;

    while (pch != NULL && index < 5)
    {
        value[index] = atof(pch+1);
        pch = strtok(NULL, " ");
        index++;
    }
    position[0] = value[1];
    position[1] = value[2];
    position[2] = value[3];
    position[3] = value[4];
}

void handlechar(char c)
{
    static int index = 0;

    switch(c)
    {
        case '\r':
            break;    

        case '\n':
            strdata[index] = '\0';
            handlestr();
            index = 0;
            break;

        default:
            strdata[index++] = c;
            break;
    }
}

// Thread function for reading position data continuously
void read_position_thread(ros::Publisher &pub)
{
    std_msgs::String result;
    ros::Rate loop_rate(20); // 20Hz read rate
    
    while (ros::ok())
    {
        if (is_reporting)
        {
            std::lock_guard<std::mutex> lock(serial_mutex);
            
            if (_serial.available())
            {
                result.data = _serial.read(_serial.available());
                for (int i = 0; i < result.data.length(); i++)
                    handlechar(result.data.c_str()[i]);

                current_state.pump = 0;
                current_state.gripper = 0;
                current_state.swiftpro_status = 0;
                current_state.motor_angle1 = 0.0;
                current_state.motor_angle2 = 0.0;
                current_state.motor_angle3 = 0.0;
                current_state.motor_angle4 = position[3];
                current_state.x = position[0];
                current_state.y = position[1];
                current_state.z = position[2];
                pub.publish(current_state);
                // ROS_INFO("Position: %.2f %.2f %.2f %.2f", position[0], position[1], position[2], position[3]);
            }
        }
        
        loop_rate.sleep();
    }
}

/*
 * Description: callback when receive data from position_write_topic
 * Inputs:      msg(float)           3 cartesian coordinates: x, y, z(mm)
 * Outputs:     Gcode                send gcode to control swift pro
 */
void position_write_callback(const swiftpro::position& msg)
{
    ROS_INFO("=== Position Control Callback Called ===");
    ROS_INFO("Received position: x=%.2f, y=%.2f, z=%.2f, speed=%lu", 
             msg.x, msg.y, msg.z, msg.speed);
    
    std::string Gcode;
    std_msgs::String result;
    std::string x = std::to_string(msg.x);
    std::string y = std::to_string(msg.y);
    std::string z = std::to_string(msg.z);
    std::string speed;
    
    if (msg.speed) {
        speed = std::to_string(msg.speed);
    } else {
        speed = "1000";
    }

    Gcode = "G0 X" + x + " Y" + y + " Z" + z + " F" + speed + "\r\n";
    
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        ROS_INFO("Sending Gcode: %s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
        ROS_INFO("Serial response: %s", result.data.c_str());
    }
    
    ROS_INFO("=== Position Control Command Completed ===\n");
}

/*
 * Description: callback when receive data from angle1st_topic
 * Inputs:      msg(float)           angle of 1st motor(degree)
 * Outputs:     Gcode                send gcode to control swift pro
 */
void angle1st_callback(const swiftpro::angle1st& msg)
{
    ROS_INFO("=== Motor 1 Angle Control Callback Called ===");
    ROS_INFO("Setting Motor 1 to angle: %.2f degrees", msg.angle1st);
    
    std::string Gcode = "";
    std_msgs::String result;
    std::string m1 = std::to_string(msg.angle1st);

    Gcode = (std::string)"G2202 N0 V" + m1 + "F100\r\n";
    
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        ROS_INFO("Sending Gcode: %s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
        ROS_INFO("Serial response: %s", result.data.c_str());
    }
    
    ROS_INFO("=== Motor 1 Control Command Completed ===\n");
}

/*
 * Description: callback when receive data from angle2nd_topic
 * Inputs:      msg(float)           angle of 2nd motor(degree)
 * Outputs:     Gcode                send gcode to control swift pro
 */
void angle2nd_callback(const swiftpro::angle2nd& msg)
{
    ROS_INFO("=== Motor 2 Angle Control Callback Called ===");
    ROS_INFO("Setting Motor 2 to angle: %.2f degrees", msg.angle2nd);
    
    std::string Gcode = "";
    std_msgs::String result;
    std::string m2 = std::to_string(msg.angle2nd);

    Gcode = (std::string)"G2202 N1 V" + m2 + "F100\r\n";
    
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        ROS_INFO("Sending Gcode: %s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
        ROS_INFO("Serial response: %s", result.data.c_str());
    }
    
    ROS_INFO("=== Motor 2 Control Command Completed ===\n");
}

/*
 * Description: callback when receive data from angle3rd_topic
 * Inputs:      msg(float)           angle of 3rd motor(degree)
 * Outputs:     Gcode                send gcode to control swift pro
 */
void angle3rd_callback(const swiftpro::angle3rd& msg)
{
    ROS_INFO("=== Motor 3 Angle Control Callback Called ===");
    ROS_INFO("Setting Motor 3 to angle: %.2f degrees", msg.angle3rd);
    
    std::string Gcode = "";
    std_msgs::String result;
    std::string m3 = std::to_string(msg.angle3rd);

    Gcode = (std::string)"G2202 N2 V" + m3 + "F100\r\n";
    
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        ROS_INFO("Sending Gcode: %s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
        ROS_INFO("Serial response: %s", result.data.c_str());
    }
    
    ROS_INFO("=== Motor 3 Control Command Completed ===\n");
}

/*
 * Description: callback when receive data from angle4th_topic
 * Inputs:      msg(float)           angle of 4th motor(degree)
 * Outputs:     Gcode                send gcode to control swift pro
 */
void angle4th_callback(const swiftpro::angle4th& msg)
{
    ROS_INFO("=== Motor 4 Angle Control Callback Called ===");
    ROS_INFO("Setting Motor 4 to angle: %.2f degrees", msg.angle4th);
    
    std::string Gcode = "";
    std_msgs::String result;
    std::string m4 = std::to_string(msg.angle4th);

    Gcode = (std::string)"G2202 N3 V" + m4 + "F500\r\n";
    
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        ROS_INFO("Sending Gcode: %s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
        ROS_INFO("Serial response: %s", result.data.c_str());
    }
    
    ROS_INFO("=== Motor 4 Control Command Completed ===\n");
}

/*
 * Description: callback when receive data from swiftpro_status_topic
 * Inputs:      msg(uint8)           status of gripper: attach if 1; detach if 0
 * Outputs:     Gcode                send gcode to control swift pro
 */
void swiftpro_status_callback(const swiftpro::status& msg)
{
    ROS_INFO("=== SwiftPro Status Callback Called ===");
    ROS_INFO("Setting status to: %d (1=attach, 0=detach)", msg.status);
    
    std::string Gcode = "";
    std_msgs::String result;

    if (msg.status == 1) {
        Gcode = (std::string)"M17\r\n";   // attach
        ROS_INFO("Attaching motors...");
    }
    else if (msg.status == 0) {
        Gcode = (std::string)"M2019\r\n";
        ROS_INFO("Detaching motors...");
    }
    else {
        ROS_ERROR("Error: Wrong swiftpro status input");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        ROS_INFO("Sending Gcode: %s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
        ROS_INFO("Serial response: %s", result.data.c_str());
    }
    
    ROS_INFO("=== SwiftPro Status Command Completed ===\n");
}

/*
 * Description: callback when receive data from gripper_topic
 * Inputs:      msg(uint8)           status of gripper: work if 1; otherwise 0
 * Outputs:     Gcode                send gcode to control swift pro
 */
void gripper_callback(const swiftpro::status& msg)
{
    ROS_INFO("=== Gripper Control Callback Called ===");
    ROS_INFO("Setting gripper to: %d (1=close, 0=open)", msg.status);
    
    std::string Gcode = "";
    std_msgs::String result;

    if (msg.status == 1) {
        Gcode = (std::string)"M2232 V1" + "\r\n";
        ROS_INFO("Closing gripper...");
    }
    else if (msg.status == 0) {
        Gcode = (std::string)"M2232 V0" + "\r\n";
        ROS_INFO("Opening gripper...");
    }
    else {
        ROS_ERROR("Error: Wrong gripper status input");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        ROS_INFO("Sending Gcode: %s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
        ROS_INFO("Serial response: %s", result.data.c_str());
    }
    
    ROS_INFO("=== Gripper Control Command Completed ===\n");
}

/*
 * Description: callback when receive data from pump_topic
 * Inputs:      msg(uint8)           status of pump: work if 1; otherwise 0
 * Outputs:     Gcode                send gcode to control swift pro
 */
void pump_callback(const swiftpro::status& msg)
{
    ROS_INFO("=== Pump Control Callback Called ===");
    ROS_INFO("Setting pump to: %d (1=on, 0=off)", msg.status);
    
    std::string Gcode = "";
    std_msgs::String result;
    std::string status = std::to_string(msg.status);

    Gcode = (std::string)"M2231 V" + status + "\r\n";
    
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        ROS_INFO("Sending Gcode: %s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
        ROS_INFO("Serial response: %s", result.data.c_str());
    }
    
    ROS_INFO("=== Pump Control Command Completed ===\n");
}

/*
 * Description: callback when receive data from buzzer_topic
 * Inputs:      msg(buzzer)          buzzer time and frequency
 * Outputs:     Gcode                send gcode to control swift pro
*/
void buzzer_callback(const swiftpro::buzzer& buzzer)
{
    ROS_INFO("=== Buzzer Control Callback Called ===");
    ROS_INFO("Setting buzzer: frequency=%d Hz, time=%.1f seconds", 
             buzzer.frequent, buzzer.time);
    
    std::string Gcode = "";
    std_msgs::String result;
    std::string time = std::to_string((int(buzzer.time)) * 1000);
    std::string frequent = std::to_string(buzzer.frequent);

    Gcode = (std::string)"M2210 F" + frequent + " T" + time + "\r\n";
    
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        ROS_INFO("Sending Gcode: %s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
        ROS_INFO("Serial response: %s", result.data.c_str());
    }
    
    ROS_INFO("=== Buzzer Control Command Completed ===\n");
}

/*
 * Description: callback to toggle position reporting
 * Inputs:      msg(uint8)           1 to enable reporting, 0 to disable
 * Outputs:     Gcode                send gcode to control swift pro
 */
void report_status_callback(const swiftpro::status& msg)
{
    ROS_INFO("=== Position Reporting Status Callback Called ===");
    std::string Gcode = "";
    std_msgs::String result;

    if (msg.status == 1) {
        Gcode = (std::string)"M2120 V0.05\r\n";  // enable reporting every 0.05s
        ROS_INFO("Enabling position reporting...");
        is_reporting = true;
    }
    else if (msg.status == 0) {
        Gcode = (std::string)"M2120 V0\r\n";     // disable reporting
        ROS_INFO("Disabling position reporting...");
        is_reporting = false;
    }
    else {
        ROS_ERROR("Error: Wrong reporting status input");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        ROS_INFO("Sending Gcode: %s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
        ROS_INFO("Serial response: %s", result.data.c_str());
    }
    
    ROS_INFO("=== Position Reporting Status Command Completed ===\n");
}

/*
 * Node name:
 *   swiftpro_unified_node
 *
 * Topic publish: (rate = 20Hz, queue size = 10)
 *   SwiftproState_topic
 *
 * Topic subscribe: (queue size = 10)
 *   position_write_topic
 *   swiftpro_status_topic
 *   angle4th_topic
 *   angle3rd_topic
 *   angle2nd_topic
 *   angle1st_topic
 *   gripper_topic
 *   pump_topic
 *   buzzer_topic
 *   report_status_topic
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "swiftpro_unified_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_position = nh.subscribe("position_write_topic", 10, position_write_callback);
    ros::Subscriber sub_status = nh.subscribe("swiftpro_status_topic", 10, swiftpro_status_callback);
    ros::Subscriber sub_angle1st = nh.subscribe("angle1st_topic", 10, angle1st_callback);
    ros::Subscriber sub_angle2nd = nh.subscribe("angle2nd_topic", 10, angle2nd_callback);
    ros::Subscriber sub_angle3rd = nh.subscribe("angle3rd_topic", 10, angle3rd_callback);
    ros::Subscriber sub_angle4th = nh.subscribe("angle4th_topic", 10, angle4th_callback);
    ros::Subscriber sub_gripper = nh.subscribe("gripper_topic", 10, gripper_callback);
    ros::Subscriber sub_pump = nh.subscribe("pump_topic", 10, pump_callback);
    ros::Subscriber sub_buzzer = nh.subscribe("buzzer_topic", 10, buzzer_callback);
    ros::Subscriber sub_report = nh.subscribe("report_status_topic", 10, report_status_callback);
    
    ros::Publisher pub = nh.advertise<swiftpro::SwiftproState>("SwiftproState_topic", 10);

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
        // Initialize the robot
        ros::Duration(3.5).sleep();                // wait for the device to be ready
        
        {
            std::lock_guard<std::mutex> lock(serial_mutex);
            _serial.write("M17\r\n");             // attach motors
            ros::Duration(0.1).sleep();
            _serial.write("M2120 V0.05\r\n");     // report position every 0.05s
        }
        
        ROS_INFO_STREAM("SwiftPro initialized and ready for commands");
    }

    // Start position reading thread
    std::thread read_thread(read_position_thread, std::ref(pub));
    read_thread.detach();  // Detach the thread so it runs independently

    // Spin to handle callbacks
    ros::spin();
    
    // Close serial connection and disable reporting when shutting down
    if (_serial.isOpen())
    {
        _serial.write("M2120 V0\r\n");  // stop reporting
        _serial.close();
    }

    return 0;
} 