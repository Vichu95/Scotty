#!/usr/bin/env python

"""
Project     : Scotty
ROS Package : scotty_controller
Script Name : dummy_controller.py
Author      : Vishnudev Kurumbaparambil
Organization: Hochschule Anhalt
Description : A dummy controller stub
Usage       : This is needed to expose the Controller Services, which otherwise is unavailable.
"""
import rospy
from controller_manager_msgs.srv import ListControllers, ListControllersResponse
from controller_manager_msgs.srv import SwitchController, SwitchControllerResponse
from controller_manager_msgs.srv import LoadController, LoadControllerResponse
from controller_manager_msgs.srv import UnloadController, UnloadControllerResponse

def handle_list_controllers(req):
    # Return an empty list of controllers.
    # This dummy service does not report any real controllers.
    response = ListControllersResponse()
    response.controller = []  # No controllers loaded.
    return response

def handle_switch_controller(req):
    # Always return success for switching controllers.
    response = SwitchControllerResponse()
    response.ok = True
    return response

def handle_load_controller(req):
    # Always return success for loading a controller.
    response = LoadControllerResponse()
    response.ok = True
    return response

def handle_unload_controller(req):
    # Always return success for unloading a controller.
    response = UnloadControllerResponse()
    response.ok = True
    return response

if __name__ == '__main__':
    rospy.init_node('dummy_controller_manager_dummy')

    # Advertise the dummy controller manager services.
    rospy.Service('/controller_manager/list_controllers', ListControllers, handle_list_controllers)
    rospy.Service('/controller_manager/switch_controller', SwitchController, handle_switch_controller)
    rospy.Service('/controller_manager/load_controller', LoadController, handle_load_controller)
    rospy.Service('/controller_manager/unload_controller', UnloadController, handle_unload_controller)

    rospy.loginfo("Dummy Controller Manager (dummy) services are now available.")
    rospy.spin()

