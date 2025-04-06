#!/usr/bin/env python

"""
Project     : Scotty
ROS Package : scotty_controller
Script Name : server_launch.py
Author      : Vishnudev Kurumbaparambil
Organization: Hochschule Anhalt
Description : This script handles the launching of the server for hosting the website
Usage       : These scripts are run from the scotty_controller.launch
"""
import rospy
import rosnode
import http.server
import socketserver
import os
import sys

def run_server(directory, port):
    # Change to the specified directory
    try:
        os.chdir(directory)
        rospy.loginfo("Serving directory: ".format(directory))
    except FileNotFoundError:
        rospy.loginfo("Error: Directory not found: ".format(directory))
        sys.exit(1)
    
    # Create handler
    handler = http.server.SimpleHTTPRequestHandler
    
    # Create server
    try:
        socketserver.TCPServer.allow_reuse_address = True
        httpd = socketserver.TCPServer(("", port), handler)
        rospy.loginfo("Server started at http://localhost:".format(port))
        httpd.serve_forever()
    except OSError as e:
        rospy.loginfo("Server error: ".format(e))
        sys.exit(1)
    except KeyboardInterrupt:
        rospy.loginfo("Server stopped by user")

if __name__ == "__main__":

    try:
        rospy.init_node('webserver_node', anonymous=False)

        # Default values
        directory = "/home/roboubu/Documents/VishnuScooty/scotty_ws/src"
        port = 8080

        run_server(directory, port)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    # rospy.init_node('webserver_node', anonymous=False)

    # # Default values
    # directory = "/home/roboubu/Documents/VishnuScooty/scotty_ws/src"
    # port = 8080
    # # default_directory = os.getcwd()
    # # default_port = 8080
    
    # # # Parse command-line arguments
    # # directory = sys.argv[1] if len(sys.argv) > 1 else default_directory
    
    # # if len(sys.argv) > 2:
    # #     try:
    # #         port = int(sys.argv[2])
    # #     except ValueError:
    # #         print(f"Error: Invalid port number: {sys.argv[2]}")
    # #         sys.exit(1)
    # # else:
    # #     port = default_port
        
    # # Run the server
    # run_server(directory, port)

