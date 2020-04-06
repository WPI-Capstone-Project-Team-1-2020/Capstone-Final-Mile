#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import NavSatFix, Imu
from hector_uav_msgs.msg import Altimeter
from geometry_msgs.msg import Vector3Stamped, Point, Quaternion, Pose, Twist, Vector3
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Take_Off:
    # Call Back Functions
    def callbackAltimeter(self, msg):
        self.barro_alt = msg.altitude

    def callbackMagnetic(self, msg):
        x = msg.vector.x
        y = msg.vector.y
        theta = math.atan2(y, x)
        cardinal_heading_temp = theta
        cardinal_heading_temp = cardinal_heading_temp*180/math.pi
        cardinal_heading_temp = cardinal_heading_temp + self.declination
        self.true_heading = self.heading_limit(cardinal_heading_temp)

    def callbackGPS(self, msg):
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude
        self.gps_alt = msg.altitude

    def callbackGPSVel(self, msg):
        self.gps_vel[0] = msg.vector.x
        self.gps_vel[1] = msg.vector.y
        self.gps_vel[2] = msg.vector.z

    def callbackIMU(self, msg):
        self.ang_vel[0] = msg.angular_velocity.x
        self.ang_vel[1] = msg.angular_velocity.y
        self.ang_vel[2] = msg.angular_velocity.z

        self.quaternion = msg.orientation

    # Utility Functions
    def heading_limit(self, heading):
        if heading < 0:
            hding_lim = 360 + heading
        elif heading > 360:
            hding_lim = 360 - heading
        else: 
            hding_lim = heading
        return hding_lim

    def global_to_local(self, gpslat, gpslon, reflat, reflon):
        # Uses equations from https://en.wikipedia.org/wiki/Geographic_coordinate_system specifically for WGS84
        # Converts global (GPS) to local (Gazebo x,y)

        lat_avg = ((gpslat + reflat)/2)*(math.pi/180)   # Average of the two latitudes in radians

        # delta in latittude converted to meters IMPORTANT: Only works in this quadrant of the hemisphere based on order of the reference and gps lat/lon.
        xlocal = (gpslat - reflat)*(111132.92 - 559.82*math.cos(2*lat_avg) + 1.175*math.cos(4*lat_avg) - 0.0023*math.cos(6*lat_avg))  
        # delta in longitude converted to meters
        ylocal = (reflon - gpslon)*(111412.84*math.cos(lat_avg) - 93.5*math.cos(3*lat_avg) + 0.118*math.cos(5*lat_avg))

        return xlocal, ylocal

    def average_altitude(self, barroalt, gpsalt):
        alt_avg = (barroalt + gpsalt)/2
        return alt_avg

    # Main Function
    def __init__(self):
        print("Setting up Localization Node")

        # Variable Initialization
        print("Localization Node: Initializing Variables")
        self.barro_alt = 0                # Altitude in meters from the barrometer
        self.gps_alt = 0                  # Altitude in meters from GPS
        self.true_heading = 0             # True heading, degrees
        self.gps_lat = 0                  # Latitude from GPS
        self.gps_lon = 0                  # Longitude from GPS
        self.gps_vel = np.zeros(3)        # GPS Velocity
        self.quaternion = Quaternion()    # Quaternion
        self.ang_vel = np.zeros(3)        # IMU Angular Velocity

        # Configuration Parameters
        self.Hertz = 50  # frequency of while loop
        self.lat_ref = 42.275011    # Latitude at origin of local frame
        self.lon_ref = -71.777747   # Longitude at origin of local frame
        self.declination = -14.10   # Difference between magnetic and true north at origin of local frame (West -> minus)
                                    # https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml based on reference lat/lon and 05APR20
        self.inclination = -0.1076  # Inclination (same source as above, 146 meters above sea level)
      
        # Subscribers
        print("Localization Node: Defining Subscribers")
        rospy.Subscriber("/fix", NavSatFix, self.callbackGPS, queue_size=1)                     # GPS Subscriber
        rospy.Subscriber("/fix_velocity", Vector3Stamped, self.callbackGPSVel, queue_size=1)    # GPS Velocity Subscriber
        rospy.Subscriber("/altimeter", Altimeter, self.callbackAltimeter, queue_size=1)         # Altimeter Subscriber
        rospy.Subscriber("/magnetic", Vector3Stamped, self.callbackMagnetic, queue_size=1)      # Compass Subscriber
        rospy.Subscriber("/raw_imu", Imu, self.callbackIMU, queue_size=1)                       # IMU Subscriber        
        
        # Publishers
        print("Localization Node: Defining Publishers")
        self.odom_pub = rospy.Publisher('/local_odom', Odometry, queue_size=1)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

        # Messages
        print("Localization Node: Defining Messages")
        self.odom_msg = Odometry()
        self.diag_msg = DiagnosticArray()
        self.diag_status = DiagnosticStatus(name = 'Localization Node:', level = 0, message = 'OK')  # Default Status

        # Loop Timing
        rate = rospy.Rate(self.Hertz)
        time.sleep(2)  # allows the callback functions to populate variables

        print("Commencing Localization Node Execution")
        while not rospy.is_shutdown():
            # Odometry Component Calculations
            # Pose Point
            altitude = self.average_altitude(self.barro_alt, self.gps_alt)
            x_local, y_local = self.global_to_local(self.gps_lat, self.gps_lon, self.lat_ref, self.lon_ref)
            # Pose Quaternion
                # Taken straight from /raw_imu
            # Twist Linear
                # Taken striaght from /fix_velocity
            # Twist Angular
            veh_to_local_rotation = Rot.from_quat([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w], normalized=True)
            rotated_ang_vel = veh_to_local_rotation.apply(self.ang_vel, inverse=True)

            # Form Odometry Message (Local (Gazebo) Frame)
            self.odom_msg.pose.pose.position = Point(x_local, y_local, altitude)
            self.odom_msg.pose.pose.orientation = self.quaternion
            self.odom_msg.twist.twist.linear.x = self.gps_vel[0]
            self.odom_msg.twist.twist.linear.y = self.gps_vel[1]
            self.odom_msg.twist.twist.linear.z = self.gps_vel[2]
            self.odom_msg.twist.twist.angular.x = rotated_ang_vel[0]
            self.odom_msg.twist.twist.angular.y = rotated_ang_vel[1]
            self.odom_msg.twist.twist.angular.z = rotated_ang_vel[2]
            self.odom_msg.header.stamp = rospy.Time.now()

            # Publish Odometry Message
            self.odom_pub.publish(self.odom_msg)

            # Publish Diagnostic Info

            rate.sleep()


if __name__ == '__main__':
    
    rospy.init_node('takeoff')
    try:
        ln = Take_Off()
    except rospy.ROSInterruptException: pass
