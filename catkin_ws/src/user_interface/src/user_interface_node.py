#!/usr/bin/env python
from __future__ import print_function
import rospy
import time
import math
import curses
from hector_uav_msgs.msg import Altimeter
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix
from autonomy_msgs.msg import Landing, Takeoff
from std_msgs.msg import Bool

class User_Interface:
    # Call Back Functions
    def callbackAltimeter(self, msg):
        global barro_alt
        barro_alt = round(msg.altitude, 1)

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
        cardinal_heading = round(cardinal_heading)

    def callbackGPS(self, msg):
        global gps_lat, gps_lon
        gps_lat = msg.latitude
        gps_lon = msg.longitude

    def callbackGPSVel(self, msg):
        global gps_x, gps_y, gps_z
        gps_x = msg.vector.x   # Velocity in N/S direction, + = N
        gps_y = msg.vector.y   # Velocity in E/W direction, + = W
        gps_z = msg.vector.z   # Velocity in Vertical Direction

    def callbackTakeoff(self, msg):
        global takeoff_status
        takeoff_status = msg.goalReached

    def callbackLanding(self, msg):
        global landing_status
        landing_status = msg.goalReached

    def callbackEstop(self, msg):
        global estop_status
        estop_status = msg.data

    # Utility Functions
    def nav_status(self, lnd_stat, tkoff_stat):
        if not lnd_stat and not tkoff_stat: #If both are False, that means the global planner is commanding both nodes to issue commands (problem)
            nav_stat_msg = "Error: Conflicting Nav Commands."
        elif not lnd_stat:
            nav_stat_msg = "Landing."
        elif not tkoff_stat:
            nav_stat_msg = "Taking Off."
        else:
            nav_stat_msg = "No Nav Commands in Execution."
        return nav_stat_msg

    # Main Function
    def __init__(self):
        print("Setting up User Interface")

        # Terminal Setup
        stdscr = curses.initscr()   # Creates Screen (Terminal) Object
        curses.noecho()             # Keystrokes do not automatically appear in terminal 
        curses.cbreak()             # Keystrokes take effect without hitting "Enter"
        stdscr.nodelay(True)        # keystroke request doesn't block the while loop

        # Variable Initialization

        # Configuration Parameters

        self.Hertz = 20  # frequency of while loop
      
        # Subscribers
        # Telemetry Data Subscribers
        rospy.Subscriber("/altimeter", Altimeter, self.callbackAltimeter, queue_size=1)     # Altimeter Subscriber 
        rospy.Subscriber("/magnetic", Vector3Stamped, self.callbackMagnetic, queue_size=1)  # Compass Subscriber
        rospy.Subscriber("/fix", NavSatFix, self.callbackGPS, queue_size=1)                 # GPS Subscriber   
        rospy.Subscriber("/fix_velocity", Vector3Stamped, self.callbackGPSVel, queue_size=1)# GPS Velocity Subscriber
        # Navigation Status Subscribers
        rospy.Subscriber("/takeoff", Takeoff, self.callbackTakeoff, queue_size=1)           # Global Planner Subscriber    
        rospy.Subscriber("/landing", Landing, self.callbackLanding, queue_size=1)           # Global Planner Subscriber
        # Estop Subscriber
        rospy.Subscriber("/estop", Bool, self.callbackEstop, queue_size=1)
        # TODO Add global planner to local planner topic subscriber

        # Publishers

        # Messages

        # Loop Timing
        rate = rospy.Rate(self.Hertz)
        time.sleep(2)  # allows the callback functions to populate variables

        try:
            while not rospy.is_shutdown():
                # Nav Status Update
                if 'landing_status' in globals() and 'takeoff_status' in globals():
                    nav_status_msg = self.nav_status(landing_status, takeoff_status)
                else:
                    nav_status_msg = "No Commands Given Since Startup."

                if 'estop_status' not in globals():
                    estop_status_local = False
                else:
                    estop_status_local = estop_status

                # Display Output
                stdscr.addstr(0, 0, "-----Telemetry Data-----")
                stdscr.addstr(1, 0, "Altitude         (m): % 4.1f" %(barro_alt))
                stdscr.addstr(2, 0, "Heading        (deg): % 3d" %(cardinal_heading))
                stdscr.addstr(3, 0, "Latitude     (dd.dd): % 7.5f" %(gps_lat))
                stdscr.addstr(4, 0, "Longitude    (dd.dd): % 7.5f" %(gps_lon))
                stdscr.addstr(5, 0, "N/S Velocity   (m/s): % 3.1f" %(gps_x))
                stdscr.addstr(6, 0, "W/E Velocity   (m/s): % 3.1f" %(gps_y))
                stdscr.addstr(7, 0, "Vert. Velocity (m/s): % 3.1f" %(gps_z))
                stdscr.addstr(8, 0, "-----Navigation Status-----")
                stdscr.addstr(9, 0, nav_status_msg + "                            ")
                stdscr.addstr(10, 0, "Estop Status        : {0}  ".format(estop_status_local))
                stdscr.addstr(11, 0, "Press 'q' to Initiate Emergency Stop")
                stdscr.refresh()

                rate.sleep()
        finally:
            curses.echo()
            curses.nocbreak()
            curses.endwin()

if __name__ == '__main__':
    
    rospy.init_node('user_interface')
    try:
        ln = User_Interface()
    except rospy.ROSInterruptException: pass
