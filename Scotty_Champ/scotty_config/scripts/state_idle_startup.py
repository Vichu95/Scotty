#!/usr/bin/env python

import rospy
import time
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelConfiguration, DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty

model_found = False  # Global variable to track if the model is found


def model_callback(msg, model_name):
    """
    Callback function to check if the model is in Gazebo.
    """
    global model_found
    if model_name in msg.name:
        model_found = True





class IdleState:
    def __init__(self):
        rospy.init_node('set_initial_conditions')

        # Specify your robot model name
        self.model_name = "scotty_robot"
        # Load the robot description (URDF)
        self.robot_description = rospy.get_param('/robot_description')

        global model_found
        model_found = False  # Reset the global variable

        rospy.Subscriber('/gazebo/model_states', ModelStates, model_callback, self.model_name)

    def delete_model(self):
        rospy.wait_for_service('/gazebo/delete_model', timeout=10)
        try:
            delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_srv(self.model_name)
            rospy.loginfo("Model '{}' deleted successfully.".format(self.model_name))
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to delete model '{}': {}".format(self.model_name, e))


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
            rospy.loginfo("Model '{}' spawned successfully.".format(self.model_name))
        except rospy.ServiceException as e:
            rospy.logerr("Failed to spawn model '{}': {}".format(self.model_name, e))


    def wait_for_model(self,timeout=10):
        """
        Waits until the specified model is available in Gazebo's /gazebo/model_states topic.
        """
        global model_found
        rospy.loginfo("Waiting for model '{}' to be spawned in Gazebo...".format(self.model_name))

        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and not model_found:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                raise RuntimeError("Timeout: Model '{}' not found in Gazebo within {} seconds.".format(self.model_name, timeout))
            rospy.sleep(0.1)

        rospy.loginfo("Model '{}' is ready in Gazebo!".format(self.model_name))
        return True

    def set_joint_positions(self):
        """
        Sets the initial joint positions for the specified model in Gazebo.
        """


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
            rospy.loginfo("Setting initial joint positions for model '{}'...".format(self.model_name))
            response = set_model_config(
                model_name=self.model_name,
                urdf_param_name='robot_description',
                joint_names=joint_names,
                joint_positions=joint_positions
            )
            if response.success:
                rospy.loginfo("Successfully set joint positions for model '{}'.".format(self.model_name))
            else:
                rospy.logerr("Failed to set joint positions: {}".format(response.status_message))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def reset_simulation(self):

        rospy.loginfo("Resetting simulation...")
        self.delete_model()
        self.spawn_model()

        ## Step 1: Wait for the model to be ready
        rospy.loginfo("Waiting for the model")
        self.wait_for_model()
        rospy.loginfo("Model is loaded in the startup")

        # Step 2: Set initial joint positions
        rospy.loginfo("Setting init joints")
        self.set_joint_positions()

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

if __name__ == '__main__':
    
    try:
        idle_state = IdleState()
        idle_state.reset_simulation()

    # except RuntimeError as e:
    #     rospy.logerr(str(e))
    # except rospy.ROSInterruptException:
    #     pass


    except rospy.ROSInterruptException:
        pass

