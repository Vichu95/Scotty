#!/usr/bin/env python

import rospy
import time
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelConfiguration
from std_srvs.srv import Empty

model_found = False  # Global variable to track if the model is found


def model_callback(msg, model_name):
    """
    Callback function to check if the model is in Gazebo.
    """
    global model_found
    if model_name in msg.name:
        model_found = True


def wait_for_model(model_name, timeout=10):
    """
    Waits until the specified model is available in Gazebo's /gazebo/model_states topic.
    """
    global model_found
    rospy.loginfo("Waiting for model '{}' to be spawned in Gazebo...".format(model_name))
    model_found = False  # Reset the global variable
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_callback, model_name)

    start_time = rospy.Time.now()
    while not rospy.is_shutdown() and not model_found:
        if (rospy.Time.now() - start_time).to_sec() > timeout:
            raise RuntimeError("Timeout: Model '{}' not found in Gazebo within {} seconds.".format(model_name, timeout))
        rospy.sleep(0.1)

    rospy.loginfo("Model '{}' is ready in Gazebo!".format(model_name))
    return True


def set_joint_positions(model_name, joint_names, joint_positions):
    """
    Sets the initial joint positions for the specified model in Gazebo.
    """
    rospy.wait_for_service('/gazebo/set_model_configuration', timeout=10)
    try:
        set_model_config = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        rospy.loginfo("Setting initial joint positions for model '{}'...".format(model_name))
        response = set_model_config(
            model_name=model_name,
            urdf_param_name='robot_description',
            joint_names=joint_names,
            joint_positions=joint_positions
        )
        if response.success:
            rospy.loginfo("Successfully set joint positions for model '{}'.".format(model_name))
        else:
            rospy.logerr("Failed to set joint positions: {}".format(response.status_message))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))



if __name__ == '__main__':
    rospy.init_node('set_initial_conditions')

    # Specify your robot model name
    model_name = "scotty_robot"

    # List of joints and their corresponding desired positions
    joint_names = [
        "abad_FL_joint", "hip_FL_joint", "knee_FL_joint",
        "abad_FR_joint", "hip_FR_joint", "knee_FR_joint",
        "abad_RL_joint", "hip_RL_joint", "knee_RL_joint",
        "abad_RR_joint", "hip_RR_joint", "knee_RR_joint"
    ]
    # joint_positions = [
    #     0.0, 0.6, -1.2,   # Front Left
    #     0.0, 0.6, -1.2,   # Front Right
    #     0.0, 0.6, -1.2,   # Rear Left
    #     0.0, 0.6, -1.2    # Rear Right
    # ]
    joint_positions = [
        0.0, 1.6, -2.2,   # Front Left
        0.0, 1.6, -2.2,   # Front Right
        0.0, 1.6, -2.2,   # Rear Left
        0.0, 1.6, -2.2    # Rear Right
    ]

    try:
        # Step 1: Wait for the model to be ready
        rospy.loginfo("Waiting for the model")
        wait_for_model(model_name)
        rospy.loginfo("Model is loaded in the startup")

        # Step 2: Set initial joint positions
        rospy.loginfo("Setting init joints")
        set_joint_positions(model_name, joint_names, joint_positions)

        # Optional: Pause physics for debugging
        rospy.loginfo("Gravity is disabled")
        rospy.wait_for_service('/gazebo/pause_physics')
        pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause_physics()


        # 3. Unpause physics
        time.sleep(3)  # Optional delay for stability
        rospy.loginfo("Gravity is enabled")
        rospy.wait_for_service('/gazebo/unpause_physics')
        pause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        pause_physics()

    except RuntimeError as e:
        rospy.logerr(str(e))
    except rospy.ROSInterruptException:
        pass

