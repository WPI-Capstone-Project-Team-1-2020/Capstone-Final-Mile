#!/usr/bin/env python
import rospy
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Diagnostics_Node:
    # Call Back Functions

    def callbackDiagnostics(self, msg):
        self.received_diag_msg = msg.status[0]

    # Utility Functions

    # Main Function
    def __init__(self):
        print("Setting up Diagnostics Node")

        # Variable Initialization
        print("Initializing Variables")
        self.names = []  # List of different diagnostics message names

        # Configuration Parameters
        self.Hertz = 20  # frequency of while loop

        # Subscribers
        print("Defining Subscribers")
        rospy.Subscriber("/diagnostics", DiagnosticArray, self.callbackDiagnostics, queue_size=1) # Diagnostics

        # Publishers
        print("Defining Publishers")
        self.diag_pub = rospy.Publisher('/diagnostics_agg', DiagnosticArray, queue_size=1)

        # Messages
        print("Defining Messages")

        self.diag_agg_msg = DiagnosticArray()
        self.diag_agg_status = []

        # Loop Timing
        rate = rospy.Rate(self.Hertz)
        time.sleep(2)  # allows the callback functions to populate variables

        print("Commencing Diagnostics Node Execution")
        while not rospy.is_shutdown():
            if self.received_diag_msg.name in self.names:
                self.diag_agg_status[self.names.index(self.received_diag_msg.name)] = self.received_diag_msg
            else:
                self.names.append(self.received_diag_msg.name)
                self.diag_agg_status.append(self.received_diag_msg)

            self.diag_agg_msg.status = self.diag_agg_status
            self.diag_pub.publish(self.diag_agg_msg)
            rate.sleep()


if __name__ == '__main__':
    
    rospy.init_node('diagnostics')
    try:
        ln = Diagnostics_Node()
    except rospy.ROSInterruptException: pass
