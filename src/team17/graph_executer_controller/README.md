# Robot Control Client-Server Architecture

This project implements a client-server architecture for robot control that allows running the LLM (Large Language Model) component in a different Python environment than the ROS-based robot control component.

## Overview

The system is split into two parts:
1. **Robot Control Server** - Runs in Python 3.8 with ROS Noetic
2. **LLM Client Node** - Runs in Python 3.12 (or any compatible environment)

This separation allows you to use newer Python features and packages with the LLM component while maintaining compatibility with ROS Noetic which requires Python 3.8.

## Components

### Robot Control Server
- Located in `robot_control_server.py`
- Manages all ROS-related operations (subscribers, publishers)
- Communicates with hardware through ROS topics
- Exposes a TCP socket interface for clients to connect

### LLM Client Node
- Located in `llm_client_node.py`
- Processes user input with LLM
- Connects to the Robot Control Server via TCP
- Runs inside the NodeGraphQt framework as a node

### Compatibility Layer
- The original `nodes_arm_control.py` now serves as a compatibility layer
- Maintains backward compatibility with existing code
- Re-routes calls to the new architecture

## Setup and Usage

### 1. Start the Robot Control Server

```bash
# In a terminal with ROS Noetic and Python 3.8 environment
cd path/to/robot_manipulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python src/team17/graph_executer_controller/scripts/start_robot_server.py
```

Options:
- `--host`: Host address to bind (default: localhost)
- `--port`: Port to bind (default: 9999)

### 2. Use the LLM Client Node in Your Graph

In your Python 3.12 environment, use the `LLMClientNode` in your node graph. This node connects to the Robot Control Server.

Node configuration:
- Set `server_host` and `server_port` to match your server configuration
- Configure LLM settings (API key, model, etc.)
- Connect inputs/outputs as needed

### 3. For Legacy Code

If your existing code uses `PandaArmControlNode` or `PandaArmDeepSeekControlNode`, they will still work but now communicate with the server over TCP.

## Node Types

### LLMClientNode
- Process natural language commands with LLM
- Communicates with Robot Control Server

### PandaArmControlNode
- Direct control of robot parameters
- Communicates with Robot Control Server

## Troubleshooting

### Connection Issues
- Ensure the server is running before starting clients
- Check that host/port settings match
- Verify network connectivity and firewall settings

### ROS Issues
- Make sure ROS environment is properly sourced
- Check that all required ROS packages are installed
- Verify that robot hardware/simulation is running

## Development

To modify or extend this architecture:

1. Server changes: Update `robot_control_server.py` to add new commands or features
2. Client changes: Update `llm_client_node.py` to add new user-facing functionality
3. Update the compatibility layer in `nodes_arm_control.py` as needed

## License

[Your license information here] 