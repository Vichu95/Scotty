#!/usr/bin/env python

"""
Project     : Scotty
ROS Package : scotty_controller
Script Name : scotty_state_idle.py
Author      : Vishnudev Kurumbaparambil
Organization: Hochschule Anhalt
Description : Handles the state Idle
Usage       : These scripts are run from the scotty_controller.launch
Logic       : The script starts by deleting the robot model if present. Then it spawns it, 
              with an initial value for pose. The gravity is also enabled after a delay.
"""

import rospy
import time
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelConfiguration, DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from std_msgs.msg import String

## Handles the callback of /gazebo/model_states
model_found = False  # Global variable to track if the model is found
def model_callback(msg, model_name):
    # Callback function to check if the model is in Gazebo.

    global model_found
    if model_name in msg.name:
        model_found = True


class IdleState:
    def __init__(self):
        rospy.init_node('scotty_state_controller_idle')

        # Specify your robot model name
        self.model_name = "scotty_robot"
        # Load the robot description (URDF)
        self.robot_description = rospy.get_param('/robot_description')

        global model_found
        model_found = False  # Reset the global variable
        rospy.Subscriber('/gazebo/model_states', ModelStates, model_callback, self.model_name)

        # Publisher to broadcast logs to the GUI console
        self.console_log_pub = rospy.Publisher("/scotty_controller/console_log", String, queue_size=10)

        rospy.loginfo("IdleController : Initialized.")
    ##############
    #   Delets the model if present, else it fails
    ##############
    def delete_model(self):
        rospy.wait_for_service('/gazebo/delete_model', timeout=10)
        try:
            delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_srv(self.model_name)
            rospy.loginfo("IdleController : Model '{}' deleted successfully.".format(self.model_name))
        except rospy.ServiceException as e:
            rospy.logwarn("IdleController : Failed to delete model '{}': {}".format(self.model_name, e))


    ##############
    #   Spawns the scotty model
    ##############
    def spawn_model(self):
        rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=10)
        try:
            spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

            # Define the initial pose
            initial_pose = Pose()
            initial_pose.position.x = 0.0
            initial_pose.position.y = 0.0
            initial_pose.position.z = 0.5 

            spawn_model_srv(self.model_name, self.robot_description, "/", initial_pose, "world")
            rospy.loginfo("IdleController : Model '{}' spawned successfully.".format(self.model_name))
        except rospy.ServiceException as e:
            rospy.logerr("IdleController : Failed to spawn model '{}': {}".format(self.model_name, e))


    ##############
    #   Setting the initial pose of the model after it is spawned
    ##############
    def wait_for_model(self,timeout=10):
        # Waits until the specified model is available in Gazebo's /gazebo/model_states topic.
        global model_found
        rospy.loginfo("IdleController : Waiting for model '{}' to be spawned in Gazebo...".format(self.model_name))

        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and not model_found:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                raise RuntimeError("IdleController : Timeout: Model '{}' not found in Gazebo within {} seconds.".format(self.model_name, timeout))
            rospy.sleep(0.1)

        rospy.loginfo("IdleController : Model '{}' is ready in Gazebo!".format(self.model_name))
        return True

    def set_joint_positions(self):
        # Sets the initial joint positions for the specified model in Gazebo.

        # List of joints and their corresponding desired positions
        joint_names = [
            "abad_FL_joint", "hip_FL_joint", "knee_FL_joint",
            "abad_FR_joint", "hip_FR_joint", "knee_FR_joint",
            "abad_RL_joint", "hip_RL_joint", "knee_RL_joint",
            "abad_RR_joint", "hip_RR_joint", "knee_RR_joint"
        ]
        
        joint_positions = [
            0.0, 1.6, -2.2,   # Front Left
            0.0, 1.6, -2.2,   # Front Right
            0.0, 1.6, -2.2,   # Rear Left
            0.0, 1.6, -2.2    # Rear Right
        ]

        rospy.wait_for_service('/gazebo/set_model_configuration', timeout=10)
        try:
            set_model_config = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
            rospy.loginfo("IdleController : Setting initial joint positions for model '{}'...".format(self.model_name))
            response = set_model_config(
                model_name=self.model_name,
                urdf_param_name='robot_description',
                joint_names=joint_names,
                joint_positions=joint_positions
            )
            if response.success:
                rospy.loginfo("IdleController : Successfully set joint positions for model '{}'.".format(self.model_name))
            else:
                rospy.logerr("IdleController : Failed to set joint positions: {}".format(response.status_message))
        except rospy.ServiceException as e:
            rospy.logerr("IdleController : Service call failed: {}".format(e))

    ##############
    #   Reseting the simulation
    ##############
    def reset_simulation(self):

        print("\n")
        rospy.loginfo("IdleController : Resetting simulation...")
        self.console_log_pub.publish("INFO    : Deleting and spawning the model")
        self.delete_model()
        self.spawn_model()

        print("\n")
        ## Step 1: Wait for the model to be ready
        self.wait_for_model()

        # Step 2: Set initial joint positions
        rospy.loginfo("IdleController : Setting up the initial pose")
        self.console_log_pub.publish("INFO    : Setting up the initial pose")
        self.set_joint_positions()

        print("\n")
        # Optional: Pause physics for debugging
        rospy.loginfo("IdleController : Gravity is disabled")
        rospy.wait_for_service('/gazebo/pause_physics')
        pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause_physics()

        # 3. Unpause physics
        time.sleep(2)  # Optional delay for stability
        rospy.loginfo("IdleController : Gravity is enabled")
        self.console_log_pub.publish("INFO    : Gravity enabled")
        rospy.wait_for_service('/gazebo/unpause_physics')
        pause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        pause_physics()

if __name__ == '__main__':
    
    try:
        idle_state = IdleState()
        idle_state.reset_simulation()

    except rospy.ROSInterruptException:
        pass

