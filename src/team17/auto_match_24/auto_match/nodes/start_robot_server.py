#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Control Server Launcher
This script is used to launch the robot control server in the ROS Python 3.8 environment.
"""
import os
import sys
import argparse
import rospy

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def main():
    """Main function to parse arguments and start the server"""
    parser = argparse.ArgumentParser(description='Start the Robot Control Server')
    parser.add_argument('--host', type=str, default='localhost',
                        help='Host address to bind the server (default: localhost)')
    parser.add_argument('--port', type=int, default=9999,
                        help='Port to bind the server (default: 9999)')
    args = parser.parse_args()
    
    try:
        # Import the server class
        from team17.auto_match_24.auto_match.nodes.robot_control_server import RobotControlServer
        
        # Create and start the server
        server = RobotControlServer(host=args.host, port=args.port)
        print(f"Starting Robot Control Server on {args.host}:{args.port}")
        server.start_server()
    
    except rospy.ROSInterruptException:
        print("ROSInterruptException received, shutting down...")
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error: {str(e)}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 