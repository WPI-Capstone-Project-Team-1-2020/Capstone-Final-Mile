#!/usr/bin/env python
import rospy
import time
import os.path
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from scipy.spatial.transform import Rotation as Rot

class Command_Log_Node:
    # Call Back Functions
    def callbackCmd(self, msg):
        self.linear_velocity = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.angular_velocity = [msg.angular.x, msg.angular.y, msg.angular.z]
        self.current_command_time = rospy.get_time()

    def callbackOdom(self, msg):
        self.gnd_lin_vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]  
        self.gnd_ang_vel = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
        self.gnd_quat    = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.cur_msg_time = rospy.get_rostime()

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

        # Variables for Ground Truth
        self.gnd_lin_vel_past = np.zeros(3) # Linear Velocity (previous message)
        self.gnd_lin_vel = np.zeros(3)      # Linear Velocity
        self.gnd_lin_acc = np.zeros(3)      # Linear Acceleration
        self.gnd_ang_vel = np.zeros(3)      # Angular Velocity
        self.gnd_rpy = np.zeros(3)          # Euler x,y,z
        self.gnd_quat = np.zeros(4)         # Quaternion
        self.past_msg_time = rospy.get_rostime()   # Message Time to calculate acceleration
        self.cur_msg_time = rospy.get_rostime()    # Message Times to calculate acceleration
        self.delta_velocity = np.zeros(3)   # Velocity Difference
        self.delta_time = 0.1               # Time Difference

        # Configuration Parameters
        self.Hertz = 5.0                                   # frequency of while loop
        self.stale_commands_threshold = 1.1*(1/self.Hertz) # Time in seconds between commands indicating commands are stale

        # Initialize Files
        file_to_open = os.path.join("", "Command_Data.txt")
        self.cmd_data_file = open(file_to_open, 'w')
        self.cmd_data_file.write("cmds = [")

        file_for_gnd = os.path.join("", "Ground_Parameters.txt")
        self.gnd_data_file = open(file_for_gnd, 'w')
        self.gnd_data_file.write('drone_parameters = [')

        # Subscribers
        print("Command Logging Node: Defining Subscribers")
        rospy.Subscriber("/cmd_vel", Twist, self.callbackCmd, queue_size=1)                 # Velocity Command Subscriber
        rospy.Subscriber("/ground_truth/state", Odometry, self.callbackOdom, queue_size=1)  # Ground Truth Subscriber

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
            # Data from /cmd_vel
            self.time = rospy.get_time()
            data_line_time = str(self.time) + ', '
            data_line_linear = str(self.linear_velocity[0]) + ', ' + str(self.linear_velocity[1]) + ', ' + str(self.linear_velocity[2]) + ', '
            data_line_angular = str(self.angular_velocity[0]) + ', ' + str(self.angular_velocity[1]) + ', ' + str(self.angular_velocity[2]) + '; '
            self.cmd_data_file.write(data_line_time + data_line_linear + data_line_angular)
            
            # Data From /ground_truth/state
            rotation_object = Rot.from_quat([self.gnd_quat[0], self.gnd_quat[1], self.gnd_quat[2], self.gnd_quat[3]], normalized=True)
            self.gnd_rpy = rotation_object.as_euler('xyz')

            np.subtract(self.gnd_lin_vel, self.gnd_lin_vel_past, self.delta_velocity)
            self.delta_time = self.cur_msg_time.to_sec() - self.past_msg_time.to_sec()
            np.divide(self.delta_velocity, self.delta_time, self.gnd_lin_acc)

            self.gnd_lin_vel_past = self.gnd_lin_vel
            self.past_msg_time = self.cur_msg_time

            gnd_data_line_lin_vel = str(self.gnd_lin_vel[0]) + ', ' + str(self.gnd_lin_vel[1]) + ', ' + str(self.gnd_lin_vel[2]) + ', '
            gnd_data_line_ang_vel = str(self.gnd_ang_vel[0]) + ', ' + str(self.gnd_ang_vel[1]) + ', ' + str(self.gnd_ang_vel[2]) + ', '
            gnd_data_line_lin_acc = str(self.gnd_lin_acc[0]) + ', ' + str(self.gnd_lin_acc[1]) + ', ' + str(self.gnd_lin_acc[2]) + ', '
            gnd_data_line_rpy     = str(self.gnd_rpy[0])     + ', ' + str(self.gnd_rpy[1])     + ', ' + str(self.gnd_rpy[2])     + '; '
            self.gnd_data_file.write(data_line_time + gnd_data_line_lin_vel + gnd_data_line_ang_vel + gnd_data_line_lin_acc + gnd_data_line_rpy)

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
