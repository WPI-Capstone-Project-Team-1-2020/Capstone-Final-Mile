#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
import cv2 as cv
from geometry_msgs.msg import Twist
from simple_pid import PID
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Camera_Calibration_Node:
    # Call Back Functions
    def callbackCamera(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    # Utility Functions
    def draw(self, img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
        img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
        img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
        img = cv.circle(img,    tuple(imgpts[3].ravel()), 3, (255,0,0), 5) 
        return img

    # Main Function
    def __init__(self):
        # Variable Initialization
        self.grid_size = 5          # grid_size and grid_center are related
        self.grid_center = 2
        self.bridge = CvBridge()    # ROS/Gazebo to cv2 converter              
        self.object_points_index = np.zeros((self.grid_size*self.grid_size,3), np.float32)              # Form object points
        self.object_points_index[:,:2] = np.mgrid[0:self.grid_size,0:self.grid_size].T.reshape(-1,2)    # FOrm object points    
        self.number_of_cal_images = 0                                        # Track number of calibration images taken
        self.object_points = []                                              # Array for storing object points
        self.image_points = []                                               # Array for storing image points
        self.axis = np.float32([[3,0,0], [0,3,0], [0,0,-3], [self.grid_center,self.grid_center,0]]).reshape(-1,3)   # Axis for determining point location
        self.transform = np.zeros((4,4))
        self.center_from_vehicle = np.zeros((4,1))
        self.center_in_image = np.zeros((4,1))
        self.command_timer_start = rospy.get_time()
        

        # Configuration Parameters
        self.Hertz = 20                                                             # frequency of while loop
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)    # Termination Criteria
        self.cal_images_needed = 10                                                 # Number of Calibration images needed
        self.gain = 1
        PID_x = [0.02, 0.5, 3]      # PID Controller Tuning Values (latitude) TODO Tune Controller
        PID_y = [0.02, 0.5, 3]      # PID Controller Tuning Values (longitude) TODO Tune Controller
        self.decent_rate = -0.4

        # Subscribers
        rospy.Subscriber("/down_cam/camera/image", Image, self.callbackCamera, queue_size=1)          # Downward Facing Camera Subscriber

        # Publishers
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Messages
        vel_msg = Twist()
        vel_msg.linear.z = self.decent_rate
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Loop Timing
        rate = rospy.Rate(self.Hertz)
        time.sleep(2)  # allows the callback functions to populate variables

        print("Commencing Camera Calibration")
        while not rospy.is_shutdown():
            # Process Image
            gray_image = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)

            # Check for corners in the image
            ret, corners = cv.findChessboardCorners(gray_image, (self.grid_size,self.grid_size), None)
            # If the corners are seen the image can be processed
            if ret:
                corners2 = cv.cornerSubPix(gray_image, corners, (11,11), (-1,-1), criteria)
                if self.number_of_cal_images <= self.cal_images_needed:
                    self.object_points.append(self.object_points_index) 
                    self.image_points.append(corners)

                    # Update Calibration Tracker
                    self.number_of_cal_images = self.number_of_cal_images + 1
                    print("Need {} more images to complete calibration.".format(self.cal_images_needed - self.number_of_cal_images))

                    # Draw Calibration Image
                    cv.drawChessboardCorners(self.image, (self.grid_size,self.grid_size), corners2, ret)
                    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(self.object_points, self.image_points, gray_image.shape[::-1], None, None)                   
                
                # If the calibration has been done at least once
                if self.number_of_cal_images > 0:
                    ret, rvecs, tvecs = cv.solvePnP(self.object_points_index, corners2, mtx, dist)
                    # rvecs and tvecs are 3x1 arrays
                    # Determine Image Points
                    image_points, jacobian = cv.projectPoints(self.axis, rvecs, tvecs, mtx, dist)
                    annotated_image = self.draw(self.image, corners2, image_points)
                    # Transform
                    rotation = cv.Rodrigues(rvecs)  # 3x3 rotation matrix with 9x3 jacobian
                    tvecs = np.transpose(tvecs)
                    self.transform[0:3,0:3] = rotation[0]
                    self.transform[0:3,3] = tvecs
                    self.transform = np.linalg.pinv(self.transform)
                    formated_center =  np.transpose(self.axis[3,:])
                    self.center_in_image[0:3,0] = formated_center
                    self.center_in_image[3,0] = 1 
                    self.center_from_vehicle = self.transform.dot(self.center_in_image) 

                if (self.number_of_cal_images > self.cal_images_needed):
                    x_pid = PID(PID_x[0], PID_x[1], PID_x[2], setpoint = -self.center_from_vehicle[0], sample_time = 1/self.Hertz, output_limits = (-2, 2))
                    y_pid = PID(PID_y[0], PID_y[1], PID_y[2], setpoint = self.center_from_vehicle[1], sample_time = 1/self.Hertz, output_limits = (-2, 2))
                    vel_msg.linear.x = x_pid(self.grid_center)
                    vel_msg.linear.y = y_pid(self.grid_center)
                    vel_pub.publish(vel_msg)

            elif self.number_of_cal_images > self.cal_images_needed:
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_pub.publish(vel_msg)

            cv.imshow("Processed Image", self.image)
            cv.waitKey(3)

            rate.sleep()

        cv.destroyAllWindows()


if __name__ == '__main__':
    
    rospy.init_node('camera_calibration')
    try:
        ln = Camera_Calibration_Node()
    except rospy.ROSInterruptException: pass
