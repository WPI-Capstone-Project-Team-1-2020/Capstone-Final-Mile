#!/usr/bin/env python
import rospy
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Diagnostics_Node:
    # Call Back Functions

    def callbackDiagnostics(self, msg):
        self.received_diag_msg = msg.status[0]
        if self.received_diag_msg.name in self.names:
            self.diag_agg_status[self.names.index(self.received_diag_msg.name)] = self.received_diag_msg
        else:
            self.names.append(self.received_diag_msg.name)
            self.diag_agg_status.append(self.received_diag_msg)

    # Utility Functions

    # Main Function
    def __init__(self):
        print("Setting up Diagnostics Node")

        # Variable Initialization
        print("Diagnostics Node: Initializing Variables")
        self.names = []  # List of different diagnostics message names
        self.received_diag_msg = []

        # Configuration Parameters
        self.Hertz = 100  # frequency of while loop

        # Subscribers
        print("Diagnostics Node: Defining Subscribers")
        rospy.Subscriber("/diagnostics", DiagnosticArray, self.callbackDiagnostics, queue_size=1000) # Diagnostics

        # Publishers
        print("Diagnostics Node: Defining Publishers")
        self.diag_pub = rospy.Publisher('/diagnostics_agg', DiagnosticArray, queue_size=1)

        # Messages
        print("Diagnostics Node: Defining Messages")

        self.diag_agg_msg = DiagnosticArray()
        self.diag_agg_status = []

        # Loop Timing
        rate = rospy.Rate(self.Hertz)
        time.sleep(2)  # allows the callback functions to populate variables

        print("Commencing Diagnostics Node Execution")
        while not rospy.is_shutdown():
            self.diag_agg_msg.status = self.diag_agg_status
            self.diag_pub.publish(self.diag_agg_msg)
            rate.sleep()


if __name__ == '__main__':
    
    rospy.init_node('diagnostics')
    try:
        ln = Diagnostics_Node()
    except rospy.ROSInterruptException: pass
