#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
import cv2 as cv
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
        return img

    # Main Function
    def __init__(self):
        # Variable Initialization
        self.grid_size = 5
        self.bridge = CvBridge()                                                                        # ROS/Gazebo to cv2 converter              
        self.object_points_index = np.zeros((self.grid_size*self.grid_size,3), np.float32)              # Form object points
        self.object_points_index[:,:2] = np.mgrid[0:self.grid_size,0:self.grid_size].T.reshape(-1,2)    # FOrm object points    
        self.number_of_cal_images = 0                                        # Track number of calibration images taken
        self.object_points = []                                              # Array for storing object points
        self.image_points = []                                               # Array for storing image points
        self.axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)   # Axis for determining point location

        # Configuration Parameters
        self.Hertz = 20                                                             # frequency of while loop
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)    # Termination Criteria
        self.cal_images_needed = 15                                                 # Number of Calibration images needed

        # Subscribers
        rospy.Subscriber("/down_cam/camera/image", Image, self.callbackCamera, queue_size=1)          # Downward Facing Camera Subscriber

        # Publishers

        # Messages

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

                    # Determine Image Points
                    image_points, jacobian = cv.projectPoints(self.axis, rvecs, tvecs, mtx, dist)
                    axis_image = self.draw(self.image, corners2, image_points)
                    print('jacobian', jacobian)
                    print('points', image_points)
            cv.imshow("Processed Image", self.image)
            cv.waitKey(3)

            rate.sleep()

        cv.destroyAllWindows()


if __name__ == '__main__':
    
    rospy.init_node('camera_calibration')
    try:
        ln = Camera_Calibration_Node()
    except rospy.ROSInterruptException: pass
