#!/usr/bin/env python
import rospy
import time
import os.path
import numpy as np
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Command_Log_Node:
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
        self.time = 0                       # Rospy Time

        # Configuration Parameters
        self.Hertz = 5  # frequency of while loop
        
        # Initialize File
        file_to_open = os.path.join("", "Command_Data.txt")
        self.cmd_data_file = open(file_to_open, 'w')
        self.cmd_data_file.write("cmds = [")
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
        time.sleep(1)  # allows the callback functions to populate variables

        print("Commencing Landing Node Execution")
        while not rospy.is_shutdown():
            self.time = rospy.get_time()
            data_line_time = str(self.time) + ', '
            data_line_linear = str(self.linear_velocity[0]) + ', ' + str(self.linear_velocity[1]) + ', ' + str(self.linear_velocity[2]) + ', '
            data_line_angular = str(self.angular_velocity[0]) + ', ' + str(self.angular_velocity[1]) + ', ' + str(self.angular_velocity[2]) + '; '
            self.cmd_data_file.write(data_line_time + data_line_linear + data_line_angular)
            # Publish Diagnostic Info

            rate.sleep()


if __name__ == '__main__':
    
    rospy.init_node('landing')
    try:
        ln = Command_Log_Node()
    except rospy.ROSInterruptException: pass
