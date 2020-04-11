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
        self.current_command_time = rospy.get_time()

    # Utility Functions

    # Main Function
    def __init__(self):
        print("Setting up Command Logging Node")

        # Variable Initialization
        print("Command Logging Node: Initializing Variables")
        self.linear_velocity = np.zeros(3)  # Linear Velocity from the command
        self.angular_velocity = np.zeros(3) # Angular Velocity from the command
        self.last_command_time = 0.0        # Time of last /cmd_vel message callback
        self.current_command_time = 0.0     # Time of current /cmd_vel message callback
        self.stale_commands = False         # Flag for Status of Receiving Commands
        

        # Configuration Parameters
        self.Hertz = 5.0                                   # frequency of while loop
        self.stale_commands_threshold = 1.1*(1/self.Hertz) # Time in seconds between commands indicating commands are stale

        # Initialize File
        file_to_open = os.path.join("", "Command_Data.txt")
        self.cmd_data_file = open(file_to_open, 'w')
        self.cmd_data_file.write("cmds = [")
        # Subscribers
        print("Command Logging Node: Defining Subscribers")
        rospy.Subscriber("/cmd_vel", Twist, self.callbackCmd, queue_size=1)          # Velocity Command Subscriber

        # Publishers
        print("Command Logging Node: Defining Publishers")
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

        # Messages
        print("Command Logging Node: Defining Messages")

        self.diag_msg = DiagnosticArray()
        self.diag_status = DiagnosticStatus(name = 'Command Logging Node:', level = 0, message = 'OK')  # Default Status

        # Loop Timing
        rate = rospy.Rate(self.Hertz)
        time.sleep(1)  # allows the callback functions to populate variables

        print("Commencing Command Logging Node Execution")
        while not rospy.is_shutdown():
            self.time = rospy.get_time()
            data_line_time = str(self.time) + ', '
            data_line_linear = str(self.linear_velocity[0]) + ', ' + str(self.linear_velocity[1]) + ', ' + str(self.linear_velocity[2]) + ', '
            data_line_angular = str(self.angular_velocity[0]) + ', ' + str(self.angular_velocity[1]) + ', ' + str(self.angular_velocity[2]) + '; '
            self.cmd_data_file.write(data_line_time + data_line_linear + data_line_angular)
            
            # Determine if Commands are stale
            delta_between_commands = self.current_command_time - self.last_command_time
            if delta_between_commands > self.stale_commands_threshold:
                self.stale_commands = True
            else:
                self.stale_commands = False
            self.last_command_time = self.current_command_time

            # Define Diagnostic Info
            if self.stale_commands:
                self.diag_status.values = [ KeyValue(key = 'Node Status', value = 'Running')]
                self.diag_status.level = 1
                self.diag_status.message = 'Not Receiving New Commands'
            else:
                self.diag_status.values = [ KeyValue(key = 'Node Status', value = 'Running')]
                self.diag_status.level = 0
                self.diag_status.message = 'OK'

            # Publish Diagnostic Info
            self.diag_msg.status = [self.diag_status]
            self.diag_pub.publish(self.diag_msg)
            rate.sleep()


if __name__ == '__main__':
    
    rospy.init_node('command_log')
    try:
        ln = Command_Log_Node()
    except rospy.ROSInterruptException: pass
