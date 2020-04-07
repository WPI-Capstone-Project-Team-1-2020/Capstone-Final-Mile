#!/usr/bin/env python
import rospy
import time
import numpy as np
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Landing_Node:
    # Call Back Functions
    def callbackCmd(self, msg):
        self.linear_velocity = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.angular_velocity = [msg.angular.x, msg.angular.y, msg.angular.z]
        
    # Utility Functions

    # Main Function
    def __init__(self):
        print("Setting up Landing Node")

        # Variable Initialization
        print("Landing Node: Initializing Variables")
        self.linear_velocity = np.zeros(3)  # Linear Velocity from the command
        self.angular_velocity = np.zeros(3) # Angular Velocity from the command
        # Configuration Parameters
        self.Hertz = 20  # frequency of while loop
        
        # Subscribers
        print("Landing Node: Defining Subscribers")
        rospy.Subscriber("/cmd_vel", Twist, self.callbackCmd, queue_size=1)          # Velocity Command Subscriber

        # Publishers
        print("Landing Node: Defining Publishers")

        # Messages
        print("Landing Node: Defining Messages")

        self.diag_msg = DiagnosticArray()
        self.diag_status = DiagnosticStatus(name = 'Landing Node:', level = 0, message = 'OK')  # Default Status

        # Loop Timing
        rate = rospy.Rate(self.Hertz)
        time.sleep(2)  # allows the callback functions to populate variables

        print("Commencing Landing Node Execution")
        while not rospy.is_shutdown():
            
            # Publish Diagnostic Info

            rate.sleep()


if __name__ == '__main__':
    
    rospy.init_node('landing')
    try:
        ln = Landing_Node()
    except rospy.ROSInterruptException: pass
