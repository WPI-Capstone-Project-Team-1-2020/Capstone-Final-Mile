#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
from simple_pid import PID
from scipy.spatial.transform import Rotation as Rot
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from hector_uav_msgs.msg import Altimeter
from geometry_msgs.msg import Twist, Vector3Stamped
from autonomy_msgs.msg import Takeoff, Status
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Take_Off:
    # Call Back Functions
    def callbackOdom(self, msg):
        self.odom_alt = msg.pose.pose.position.z
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        self.odom_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    def callbackTakeoff(self, msg):
        # Save information from the message
        self.goal_reached = msg.goalReached
        self.goal_alt = msg.height
        self.start_time = time.time()
        self.got_new_goal = True

        # Define lateral goal as current local x and y
        self.goal_x = self.odom_x
        self.goal_y = self.odom_y

        # Update the Flag for other nodes indicating the status of the takeoff node
        if not self.goal_reached:
            print("Commence Takeoff")
            self.status_msg.status = self.goal_reached
            self.status_pub.publish(self.status_msg)

    # Utility Functions
    def local_to_vehicle_frame(self, xtruth, ytruth, xgoal, ygoal, heading):
        # Translation from local xyz frame to vehicle frame is xtruth
        # Translation from local xyz frame to vehicle frame is ytruth
        # Ignoring translation in the z, rotation in the x and y based on assumption
        # that the drone remains horizontal during takeoff and the fact that horizontal control
        # is done separately by the barometer.
        offset_angle = -heading*math.pi/180 # Convert to radians for trig functions, - due to rotation matrix from localization giving rotation about the z axis with z+ up
        if offset_angle > math.pi:
            offset_angle =  offset_angle - 2*math.pi
        elif offset_angle < math.pi:
            offset_angle = offset_angle + 2*math.pi
        rot_z = np.array([[math.cos(offset_angle), -math.sin(offset_angle), 0, 0],
                          [math.sin(offset_angle), math.cos(offset_angle), 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        trans_x = np.array([[1, 0, 0, xtruth],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        trans_y = np.array([[1, 0, 0, 0],
                            [0, 1, 0, ytruth],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        hom_trans = trans_x.dot(trans_y)
        hom_trans = hom_trans.dot(rot_z)  # Transforms vehicle to local frame
        hom_trans = np.linalg.inv(hom_trans)  # Transforms local to vehicle frame
        goallocal = np.array([[xgoal], [ygoal], [0], [1]])
        goalveh = hom_trans.dot(goallocal)
        return goalveh

    # Main Function
    def __init__(self):
        print("Setting up Takeoff Node")

        # Variable Initialization
        print("Takeoff Node: Initializing Variables")
        
        # Flags
        self.goal_reached = False       # Assume goal is reached (no action required) until told otherwise.
        self.got_new_goal = False       # Flag for status of getting New Goal
        self.goal_alt = 0               # Initial Goal Altitude
        self.start_time = time.time()   # Initialize Start Time
        self.odom_alt = 13              # Default altitude until first callback    
        self.odom_x = 0                 # Vehicle X position 
        self.odom_y = 0                 # Vehicle Y position
        self.odom_quat = np.zeros(4)    # Quaternion from the Localization Node
        self.goal_x = 0                 # Goal in local frame
        self.goal_y = 0                 # Goal in local frame

        PID_alt = [1, 1, 5]      # PID Controller Tuning Values (altitude) TODO Tune Controller
        PID_x = [0.5, 1, 1]      # PID Controller Tuning Values (latitude) TODO Tune Controller
        PID_y = [0.5, 1, 1]      # PID Controller Tuning Values (longitude) TODO Tune Controller

        # Configuration Parameters
        # goal_x = -219.0
        # goal_y = 1190.0
        # goal_lat = 42.2644910473      # Corresponds to Lat of Takeoff Pad
        # goal_lon = -71.7737136467     # Corresponds to Lon of Takeoff Pad
        alt_threshold = 0.5             # meters
        horizontal_threshold = 2        # meters, summed in x and y axis
        self.Hertz = 20                 # frequency of while loop
        self.should_be_done_time = 120  # Seconds it should take to complete a takeoff
      
        # Subscribers
        print("Takeoff Node: Defining Subscribers")
        rospy.Subscriber("/local_odom", Odometry, self.callbackOdom, queue_size=1)          # Local Odometry Subscriber
        rospy.Subscriber("/takeoff", Takeoff, self.callbackTakeoff, queue_size=10)          # Global Planner Subscriber
        
        # Publishers
        print("Takeoff Node: Defining Publishers")
        vel_pub = rospy.Publisher('/height_controller', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/takeoff_status', Status, queue_size=1)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

        # Messages
        print("Takeoff Node: Defining Messages")
        vel_msg = Twist()
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.diag_msg = DiagnosticArray()
        self.diag_status = DiagnosticStatus(name = 'Takeoff Node:', level = 0, message = 'OK')  # Default Status

        self.status_msg = Status()

        # Loop Timing
        rate = rospy.Rate(self.Hertz)
        time.sleep(2)  # allows the callback functions to populate variables

        print("Commencing Takeoff Node Execution")
        while not rospy.is_shutdown():
            if self.got_new_goal:
                
                # Vertical control based on odometry
                alt_pid = PID(PID_alt[0], PID_alt[1], PID_alt[2], setpoint = self.goal_alt, sample_time= 1/self.Hertz, output_limits = (-10, 10)) # Height PID Controller
                vel_msg.linear.z = alt_pid(self.odom_alt)
                
                # Lateral control based on ground truth
                odom_rotation = Rot.from_quat(self.odom_quat, normalized=True)
                odom_rotation = odom_rotation.as_euler('xyz', degrees=True)
                goal_veh = self.local_to_vehicle_frame(self.odom_x, self.odom_y, self.goal_x, self.goal_y, odom_rotation[2])
                x_pid = PID(PID_x[0], PID_x[1], PID_x[2], setpoint = goal_veh[0], sample_time = 1/self.Hertz, output_limits = (-10, 10))
                y_pid = PID(PID_y[0], PID_y[1], PID_y[2], setpoint = goal_veh[1], sample_time = 1/self.Hertz, output_limits = (-10, 10))
                vel_msg.linear.x = x_pid(0)
                vel_msg.linear.y = y_pid(0)

                vel_pub.publish(vel_msg)

                # Determine when the goal is met and tell the global planner
                delta_alt = abs(self.goal_alt - self.odom_alt)
                horizontal_error = abs(goal_veh[0]) + abs(goal_veh[1])
                if (delta_alt < alt_threshold) and (horizontal_error < horizontal_threshold):
                    print("Takeoff Complete")
                    self.goal_reached = True
                    self.got_new_goal = False
                    self.status_msg.status = self.goal_reached
                    self.status_pub.publish(self.status_msg)

            
            
            # Publish Diagnostic Info
            self.current_time = time.time()
            self.elapsed_time = self.current_time - self.start_time
            
            if not self.got_new_goal:
                self.diag_status.values = [ KeyValue(key = 'Node Status', value = 'Standby'),
                                            KeyValue(key = 'Goal Height', value = 'None')]
                self.diag_status.level = 0
                self.diag_status.message = 'OK'
            elif not self.goal_reached and self.got_new_goal:
                self.diag_status.values = [ KeyValue(key = 'Node Status', value = 'Running'),
                                            KeyValue(key = 'Goal Height', value = '{}'.format(self.goal_alt))]
                if self.elapsed_time > self.should_be_done_time:
                    self.diag_status.level = 1
                    self.diag_status.message = 'Takeoff Taking Longer Than Expected'
                else:
                    self.diag_status.level = 0
                    self.diag_status.message = 'OK'
            else:
                self.diag_status.values = [ KeyValue(key = 'Node Status', value = 'Error'),
                                            KeyValue(key = 'Goal Height', value = 'Error')]
                self.diag_status.level = 2
                self.diag_status.message = 'Invalid Takeoff Node State'    
            self.diag_msg.status = [self.diag_status]
            self.diag_pub.publish(self.diag_msg)
        
            rate.sleep()


if __name__ == '__main__':
    
    rospy.init_node('takeoff')
    try:
        ln = Take_Off()
    except rospy.ROSInterruptException: pass
