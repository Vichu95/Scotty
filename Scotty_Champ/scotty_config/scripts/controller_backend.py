#!/usr/bin/env python
 
import rospy
from std_msgs.msg import String
 
class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
 
        self.state = "Idle"
        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        self.command_sub = rospy.Subscriber('/controller_command', String, self.handle_command)
 
        rospy.loginfo("RobotController node initialized.")
        self.publish_state()
 
    def publish_state(self):
        """Publish the current state."""
        self.state_pub.publish(self.state)
        rospy.loginfo("Current State: {}".format(self.state))
 
    def handle_command(self, msg):
        """Handle incoming state transition commands."""
        rospy.loginfo("Received command to switch to state: {}".format(msg.data))
        if self.is_valid_transition(msg.data):
            self.state = msg.data
            self.publish_state()
        else:
            rospy.logwarn("Invalid state transition: {} {}".format(self.state,msg.data ))
 
    def is_valid_transition(self, new_state):
        """Validate state transitions."""
        valid_transitions = {
            "Idle": ["Stand"],
            "Stand": ["Idle", "Walk"],
            "Walk": ["Stand"]
        }
        return new_state in valid_transitions.get(self.state, [])
 
if __name__ == '__main__':
    try:
        RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass