#!/bin/bash
# 自动启动机器人控制服务器的脚本

# 显示帮助信息
function show_help {
  echo "使用方法: $0 [选项]"
  echo "选项:"
  echo "  -h, --help       显示此帮助信息"
  echo "  -i, --host IP    设置服务器监听的IP地址 (默认: localhost)"
  echo "  -p, --port PORT  设置服务器监听的端口 (默认: 9999)"
  echo "  --no-screen      输出日志到文件而不是屏幕"
  echo
  echo "示例:"
  echo "  $0                            # 使用默认设置启动"
  echo "  $0 -i 0.0.0.0 -p 8888        # 监听所有网络接口，端口 8888"
  echo "  $0 --host 192.168.1.100      # 监听特定IP"
}

# 默认参数
HOST="localhost"
PORT="9999"
LOG_OUTPUT="screen"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      show_help
      exit 0
      ;;
    -i|--host)
      HOST="$2"
      shift 2
      ;;
    -p|--port)
      PORT="$2"
      shift 2
      ;;
    --no-screen)
      LOG_OUTPUT="log"
      shift
      ;;
    *)
      echo "未知参数: $1"
      show_help
      exit 1
      ;;
  esac
done

# 输出启动信息
echo "启动机器人控制服务器..."
echo "主机: $HOST"
echo "端口: $PORT"
echo "日志输出: $LOG_OUTPUT"

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 确保ROS环境已经设置
if [ -z "$ROS_DISTRO" ]; then
  echo "警告: ROS环境似乎未设置。尝试加载..."
  if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
  else
    echo "错误: 无法找到ROS设置文件。请确保ROS已安装。"
    exit 1
  fi
fi

# 获取工作空间根目录
WORKSPACE_DIR=$(cd "$SCRIPT_DIR/../../../.." && pwd)
if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
  source "$WORKSPACE_DIR/devel/setup.bash"
  echo "已加载工作空间: $WORKSPACE_DIR"
else
  echo "警告: 无法找到工作空间设置文件 ($WORKSPACE_DIR/devel/setup.bash)"
  echo "可能会出现包查找错误。"
fi

# 启动服务器
roslaunch auto_match robot_control_socket.launch host:="$HOST" port:="$PORT" log_output:="$LOG_OUTPUT"

echo "机器人控制服务器已关闭。" 