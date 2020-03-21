#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
from simple_pid import PID
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from hector_uav_msgs.msg import Altimeter
from geometry_msgs.msg import Twist, Vector3Stamped

class TakeOff:
    # Call Back Functions
    def callbackAltimeter(self, msg):
        global barro_alt
        barro_alt = msg.altitude

    def callbacktruth(self, msg):
        global x_truth, y_truth
        x_truth = msg.pose.pose.position.x
        y_truth = msg.pose.pose.position.y

    def callbackMagnetic(self, msg):
        global cardinal_heading
        x = msg.vector.x
        y = msg.vector.y
        theta = math.atan2(y, x)
        cardinal_heading_temp = theta
        cardinal_heading_temp = cardinal_heading_temp*180/math.pi
        if cardinal_heading_temp < 0:
            cardinal_heading = 360 + cardinal_heading_temp
        elif cardinal_heading_temp > 360:
            cardinal_heading = 360 - cardinal_heading_temp
        else:
            cardinal_heading = cardinal_heading_temp

    # Utility Functions
    def local_to_vehicle_frame(self, xtruth, ytruth, xgoal, ygoal, heading):
        # Translation from local xyz frame to vehicle frame is xtruth
        # Translation from local xyz frame to vehicle frame is ytruth
        # Ignoring translation in the z, rotation in the x and y based on assumption
        # that the drone remains horizontal during takeoff and the fact that horizontal control
        # is done separately by the barometer.
        offset_angle = heading*math.pi/180 # Convert to radians for trig functions
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
        
        # Variable Initialization
        trigger = False
        PID_alt = [1, 1, 5]     # PID Controller Tuning Values (altitude) TODO Tune Controller
        PID_x = [0.5, 1, 1]     # PID Controller Tuning Values (latitude) TODO Tune Controller
        PID_y = [0.5, 1, 1]     # PID Controller Tuning Values (longitude) TODO Tune Controller

        # Configuration Parameters
        goal_x = -1230.999713
        goal_y = -285.600030
        # goal_lat = 42.2644910473    # Corresponds to Lat of Takeoff Pad
        # goal_lon = -71.7737136467   # Corresponds to Lon of Takeoff Pad
        goal_alt = 50           # Desired Alititude in Meters TODO Change from Hardcoded
        trigger = True          # Boolean telling node to activate TODO Change from Hardcoded

        self.Hertz = 20  # frequency of while loop
      
        # Subscribers
        # rospy.Subscriber("/fix", NavSatFix, self.callbackGPS, queue_size=1)                 # GPS Subscriber
        rospy.Subscriber("/altimeter", Altimeter, self.callbackAltimeter, queue_size=1)     # Altimeter Subscriber
        rospy.Subscriber("/magnetic", Vector3Stamped, self.callbackMagnetic, queue_size=1)  # Compass Subscriber
        rospy.Subscriber("/ground_truth/state", Odometry, self.callbacktruth, queue_size=1) # Ground truth Subscriber
        # Publishers
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Messages
        vel_msg = Twist()
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Loop Timing
        rate = rospy.Rate(self.Hertz)
        time.sleep(2)  # allows the callback functions to populate variables

        while not rospy.is_shutdown():
            if trigger:
                # Vertical control based on altimeter (barometer)
                alt_pid = PID(PID_alt[0], PID_alt[1], PID_alt[2], setpoint = goal_alt, sample_time= 1/self.Hertz, output_limits = (-10, 10)) # Height PID Controller
                vel_msg.linear.z = alt_pid(barro_alt)
                
                # Lateral control based on ground truth
                goal_veh = self.local_to_vehicle_frame(x_truth, y_truth, goal_x, goal_y, cardinal_heading)
                x_pid = PID(PID_x[0], PID_x[1], PID_x[2], setpoint = goal_veh[0], sample_time = 1/self.Hertz, output_limits = (-10, 10))
                y_pid = PID(PID_y[0], PID_y[1], PID_y[2], setpoint = goal_veh[1], sample_time = 1/self.Hertz, output_limits = (-10, 10))
                vel_msg.linear.x = x_pid(0)
                vel_msg.linear.y = y_pid(0)

                vel_pub.publish(vel_msg)
                # TODO Add trigger from higher level algorithm to execute or not.
            rate.sleep()

if __name__ == '__main__':
    
    rospy.init_node('takeoff')
    try:
        ln = TakeOff()
    except rospy.ROSInterruptException: pass
