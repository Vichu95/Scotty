#!/usr/bin/env python

"""
Project     : Scotty
ROS Package : scotty_controller
Script Name : scotty_controller_manager.py
Author      : Vishnudev Kurumbaparambil
Organization: Hochschule Anhalt
Description : The controller manager
Usage       : These scripts are called from scotty_main_controller.py 
"""

import rospy
from controller_manager_msgs.srv import ListControllers, LoadController, SwitchController
 
class ScottyControllerManager:
    def __init__(self):
        # Initialize the services for controller management
        rospy.wait_for_service("/controller_manager/list_controllers", timeout=10)
        rospy.wait_for_service("/controller_manager/load_controller", timeout=10)
        rospy.wait_for_service("/controller_manager/switch_controller", timeout=10)
        
        self.list_controllers_srv = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
        self.load_controller_srv = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
        self.switch_controller_srv = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
 
    def load_and_start_controller(self, controller_name):
        # Check if the controller is already loaded and running
        try:
            rospy.loginfo("ControllerManager : Checking if {} is already loaded...".format(controller_name))
            list_response = self.list_controllers_srv()
            controller_found = False
            controller_running = False
            
            for controller in list_response.controller:
                if controller.name == controller_name:
                    controller_found = True
                    if controller.state == "running":
                        controller_running = True
                    break
            
            if controller_found:
                if controller_running:
                    rospy.loginfo("ControllerManager : {} is already loaded and running.".format(controller_name))
                    return True
                else:
                    rospy.loginfo("ControllerManager : {} is loaded but not running. Starting it now...".format(controller_name))
                    return self.start_controller(controller_name)
            else:
                rospy.loginfo("ControllerManager : {} not loaded. Loading and starting it now...".format(controller_name))
                return self.load_and_start_new_controller(controller_name)
 
        except rospy.ServiceException as e:
            rospy.logerr("ControllerManager : Service call failed: {}".format(e))
            return False
 
    def load_and_start_new_controller(self, controller_name):
        # Load the controller
        try:
            rospy.loginfo("ControllerManager : Loading {}...".format(controller_name))
            load_response = self.load_controller_srv(controller_name)
            if load_response.ok:
                rospy.loginfo("ControllerManager : {} loaded successfully.".format(controller_name))
            else:
                rospy.logerr("ControllerManager : Failed to load {}.".format(controller_name))
                return False
        except rospy.ServiceException as e:
            rospy.logerr("ControllerManager : Service call failed: {}".format(e))
            return False
 
        # Start the controller
        return self.start_controller(controller_name)
 
    def start_controller(self, controller_name):
        try:
            rospy.loginfo("ControllerManager : Starting {}...".format(controller_name))
            switch_response = self.switch_controller_srv(
                start_controllers=[controller_name],
                stop_controllers=[],
                strictness=2
            )
            if switch_response.ok:
                rospy.loginfo("ControllerManager : {} started successfully.".format(controller_name))
                return True
            else:
                rospy.logerr("ControllerManager : Failed to start {}.".format(controller_name))
                return False
        except rospy.ServiceException as e:
            rospy.logerr("ControllerManager : Service call failed: {}".format(e))
            return False

    def stop_controller(self, controller_name):
        try:
            rospy.loginfo("ControllerManager : Stopping {}...".format(controller_name))
            switch_response = self.switch_controller_srv(
                start_controllers=[],
                stop_controllers=[controller_name],
                strictness=2
            )
            if switch_response.ok:
                rospy.loginfo("ControllerManager : {} stopped successfully.".format(controller_name))
                return True
            else:
                rospy.logerr("ControllerManager : Failed to stop {}.".format(controller_name))
                return False
        except rospy.ServiceException as e:
            rospy.logerr("ControllerManager : Service call failed: {}".format(e))
            return False