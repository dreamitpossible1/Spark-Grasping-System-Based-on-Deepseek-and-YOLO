#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
MASTER_IP=master_ubuntu
#SLAVE_IP=spark_robo5
SLAVE_IP=spark_robo4
export PATH
export ROS_MASTER_URI=http://${MASTER_IP}:11311
export ROS_HOSTNAME=${SLAVE_IP}
source ~/spark_cao/devel/setup.bash


#=================================================
#       System Required: Ubuntu 14.04+
#       Description: Install ROS And Spark
#       Version: 1.0.21
#       Author: J.xiao
#       Site: http://www.nxrobo.com/
#       SPARK技术讨论与反馈群：8346256
#=================================================


sh_ver="1.1.0"
filepath=$(cd "$(dirname "$0")"; pwd)
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Warn="${Yellow_font_prefix}[警告]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"
Separator_1="——————————————————————————————"
password="spark"
Version=$(lsb_release -r --short)
Codename=$(lsb_release -c --short)
OSDescription=$(lsb_release -d --short)
OSArch=$(uname -m)
calibra_default="${filepath}/../.ros/camera_info"


#检查系统要求
check_sys(){
        if [[ "${Version}" == "14.04" ]]; then
                ROS_Ver="indigo"
        elif [[ "${Version}" == "16.04" ]]; then
                ROS_Ver="kinetic"
        elif [[ "${Version}" == "18.04" ]]; then
                ROS_Ver="melodic"
        else
                echo -e "${Error} SPARK不支持当前系统 ${OSDescription} !" && exit 1
        fi
}

#检查设备连接
check_dev(){
        #检查底盘
        if [ ! -n "$(lsusb -d 1a86:7523)" ]; then
                echo -e "${Error} 底盘没有正确连接，请确认正确连接！！"
        fi
}

let_robot_go(){
        PROJECTPATH=$(cd `dirname $0`; pwd)
        source ${PROJECTPATH}/devel/setup.bash
        roslaunch spark_teleop teleop_gao.launch
}
check_dev
let_robot_go
