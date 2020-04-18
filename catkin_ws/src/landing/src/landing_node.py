#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as Rot
from simple_pid import PID
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Range, Image
from hector_uav_msgs.msg import Altimeter
from geometry_msgs.msg import Twist, Vector3Stamped
from autonomy_msgs.msg import Landing, Status, HospitalGoal, Takeoff
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Landing_Node:
    # Call Back Functions

    def callbackSonic(self, msg):
        global sonic_dist, sonic_max
        sonic_dist = msg.range
        sonic_max = msg.max_range
   
    def callbackLanding(self, msg):
        self.goal_reached = msg.goalReached
        self.start_time = time.time()
        self.goal_x = msg.x_local
        self.goal_y = msg.y_local
        self.start_time = rospy.Time.now()  # Time Command sent
        if not self.goal_reached:
            print("Landing Node: Commence Landing")
            print('Landing Node: Stabliizing for {} seconds'.format(self.drone_stable_time.to_sec()))
            # Inform the Global Planner that the Goal is NOT reached
            self.status_msg.status = self.goal_reached
            self.status_pub.publish(self.status_msg)

    def callbackOdom(self, msg):
        self.odom_alt = msg.pose.pose.position.z
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        self.odom_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    def callbackCamera(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print("Landing Node:", e)

    def callbackTakeoff(self, msg):
        self.takeoff_initiated = not msg.goalReached

    # Utility Functions
    def local_to_vehicle_frame(self, xtruth, ytruth, xgoal, ygoal, euler_xyz):
        # Translation from local xyz frame to vehicle frame is xtruth
        # Translation from local xyz frame to vehicle frame is ytruth
        # Ignoring translation in the z, rotation in the x and y based on assumption
        # that the drone remains horizontal during takeoff and the fact that horizontal control
        # is done separately by the barometer.
        rot_x = np.array([[1, 0, 0, 0],
                          [0, math.cos(euler_xyz[0]), -math.sin(euler_xyz[0]), 0],
                          [0, math.sin(euler_xyz[0]),  math.cos(euler_xyz[0]), 0],
                          [0, 0, 0, 1]])
        rot_y = np.array([[math.cos(euler_xyz[1]), 0, math.sin(euler_xyz[1]), 0],
                          [0, 1, 0, 0],
                          [-math.sin(euler_xyz[1]), 0, math.cos(euler_xyz[1]), 0],
                          [0, 0, 0, 1]])
        rot_z = np.array([[math.cos(euler_xyz[2]), -math.sin(euler_xyz[2]), 0, 0],
                          [math.sin(euler_xyz[2]), math.cos(euler_xyz[2]), 0, 0],
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
        hom_trans = hom_trans.dot(rot_x)
        hom_trans = hom_trans.dot(rot_y)
        hom_trans = hom_trans.dot(rot_z)  # Transforms vehicle to local frame
        hom_trans = np.linalg.inv(hom_trans)  # Transforms local to vehicle frame
        goallocal = np.array([[xgoal], [ygoal], [0], [1]])
        goalveh = hom_trans.dot(goallocal)
        return goalveh

    def camera_to_vehicle_frame(self):
        # Translation from the camera frame to the vehicle frame
        # Does not change with time
        # Angles derived from reversing the rpy values between the optical link -> camera link -> base link
        x_ang = math.pi/2
        y_ang = -math.pi/2
        z_ang = math.pi/2
        rot_x = np.array([[1, 0, 0, 0],
                          [0, math.cos(x_ang), -math.sin(x_ang), 0],
                          [0, math.sin(x_ang),  math.cos(x_ang), 0],
                          [0, 0, 0, 1]])
        rot_y = np.array([[math.cos(y_ang), 0, math.sin(y_ang), 0],
                          [0, 1, 0, 0],
                          [-math.sin(y_ang), 0, math.cos(y_ang), 0],
                          [0, 0, 0, 1]])
        rot_z = np.array([[math.cos(z_ang), -math.sin(z_ang), 0, 0],
                          [math.sin(z_ang), math.cos(z_ang), 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        cam_to_veh = rot_x.dot(rot_z)
        cam_to_veh = cam_to_veh.dot(rot_y)
        return cam_to_veh

    def image_to_camera_frame (self, pixel_width, pixel_height, pixel_in_image):
        z_ang = - math.pi / 2
        x_ang = math.pi
        x_trans = pixel_width / 2
        y_trans =  pixel_height / 2
        rot_x = np.array([[1, 0, 0, 0],
                          [0, math.cos(x_ang), -math.sin(x_ang), 0],
                          [0, math.sin(x_ang),  math.cos(x_ang), 0],
                          [0, 0, 0, 1]])
        rot_z = np.array([[math.cos(z_ang), -math.sin(z_ang), 0, 0],
                          [math.sin(z_ang), math.cos(z_ang), 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        trans_x = np.array([[1, 0, 0, x_trans],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        trans_y = np.array([[1, 0, 0, 0],
                            [0, 1, 0, y_trans],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]]) 

        hom_trans = trans_x.dot(trans_y)
        hom_trans = hom_trans.dot(rot_z)
        hom_trans = hom_trans.dot(rot_x)
        pixel_in_image = pixel_in_image.ravel()
        pixel_in_cam_frame = hom_trans.dot(np.transpose(np.array([pixel_in_image[0], pixel_in_image[1], 0, 1])))
        return pixel_in_cam_frame

    def draw(self, img, corners, imgpts):
        img = cv.circle(img,    tuple(imgpts[3].ravel()), 4, (255,0,0), 5) 
        return img

    def ratio_lookup(self, alt, altitudes, ratios):
        for altitude in altitudes:
            if alt < altitude:
                index = altitudes.index(altitude)
                return ratios[index]
            else:
                return ratios[-1]

    # Main Function
    def __init__(self):
        print("Setting up Landing Node")

        # Variable Initialization
        print("Landing Node: Initializing Variables")
        # Landing Variable Initialization
        self.goal_reached = True            # Assume goal reached (no action required) upon startup.
        self.takeoff_initiated = False      # See if Takeoff has been ordered (opportunity for calibration)
        self.landing_check = 0              # Track number of sequential returns less than threshold
        self.start_time = rospy.Time.now()  # Initialize Start Time
        self.goal_x = 287.0                 # Hospital Goal x in local frame (default value)
        self.goal_y = -1356.0               # Hospital Goal y in local fram (default value)
        self.odom_alt = 13                  # Default altitude until first callback    
        self.odom_x = 0                     # Vehicle X position 
        self.odom_y = 0                     # Vehicle Y position
        self.odom_quat = np.zeros(4)        # Quaternion from the Localization Node
        self.goal_veh = [0,0]               # Goal Initialization
        self.horizontal_error = 0.0         # Initial Horizontal Error

        # Camera Variable Initialization
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
        self.camera_calibration_status_options = ['complete', 'partially done', 'not done']
        self.camera_calibration_status = self.camera_calibration_status_options[2]
        self.camera_calibration_started = False
        self.GPS_dist_to_goal = 0.0
        self.drone_stable_time = rospy.Duration(2)  # Send 0 commands this long to stabilize (seconds)
        self.last_cam_command_time = 0
        self.center_of_image = [320, 240]   # Half of the camera resolution, x is width, y is height, measured from the top left of the image
        self.altitudes = (15.9, 16, 16.6, 21.6, 22, 22.5, 22.8, 23.3, 23.6, 24, 24.5, 24.8, 25.1, 26.3, 27.5, 27.9, 28.4, 29.2, 29.6)
        self.ratios = (0.004193548387, 0.004276315789, 0.003095238095, 0.024, 0.02590909091, 0.02768292683, 0.03026666667, 0.03289855072, 0.03546875, 0.03830508475, 0.04185185185, 0.0452, 0.04891304348, 0.06787878788, 0.0892, 0.1013636364, 0.1233333333, 0.17, 0.22)
        self.cam_x_m = 0
        self.cam_y_m = 0
        self.cam_GPS_diff = [0, 0]
        self.error_recalc = 6

        # Landing Configuration Parameters
        PID_alt = [0.4, 0.2, 1.0]          # PID Controller Tuning Values 
        PID_x =   [0.3, 0.3, 3.0]           # PID Controller Tuning Values (odom) 
        PID_y =   [0.3, 0.3, 3.0]           # PID Controller Tuning Values (odom) 
        self.goal_alt = 8                   # Desired Alititude in Meters (staying hardcoded since building height won't change)
        alt_threshold = 0.25                # meters, based on sonar return
        horizontal_threshold = 2            # meters, summed in x and y axis
        self.Hertz = 20                     # frequency of while loop
        self.altitude_ctrl_shift = 25       # meters at which control shifts to ultrasonic sensor
        self.landing_check_threshold = 10   # Number of sequential returns required to call landing complete
        self.should_be_done_time = rospy.Duration(120)      # Seconds it should take to complete a landing
        
        # Camera Configuration Parameters
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)    # Termination Criteria
        self.cal_images_needed = 10                                                 # Number of Calibration images needed
        # PID_x_cam = [0.03, 0.0, 5.0]     # PID Controller Tuning Values (camera) 
        # PID_y_cam = [0.03, 0.0, 5.0]     # PID Controller Tuning Values (camera) 
        self.enable_camera_altitude = 40 # Enable camera processing once below this altitude, based on 20m range with 5m buffer
        self.lost_image_control_decay = 1.03 # Factor to decay commands by until image is regained.
        self.lost_pad_distance = 8       # meter offset from "GPS goal" to switch back to GPS lateral control

        # Subscribers
        print("Landing Node: Defining Subscribers")
        # rospy.Subscriber("/fix", NavSatFix, self.callbackGPS, queue_size=1) # GPS Data
        rospy.Subscriber("/sonar_height", Range, self.callbackSonic, queue_size=1)           # Altimeter Subscriber
        rospy.Subscriber("/local_odom", Odometry, self.callbackOdom, queue_size=1)           # Local Odometry Subscriber
        rospy.Subscriber("/landing", Landing, self.callbackLanding, queue_size=1)            # Global Planner Subscriber
        rospy.Subscriber("/down_cam/camera/image", Image, self.callbackCamera, queue_size=1) # Downward Facing Camera Subscriber
        rospy.Subscriber("/takeoff", Takeoff, self.callbackTakeoff, queue_size=10)           # Global Planner Subscriber
        # rospy.Subscriber("/ground_truth/state", Odometry, self.callbackTruth, queue_size=1)  # Used for Meter to pixel ratio

        # Publishers
        print("Landing Node: Defining Publishers")
        vel_pub = rospy.Publisher('/height_controller', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/landing_status', Status, queue_size=1)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

        # Messages
        print("Landing Node: Defining Messages")
        vel_msg = Twist()
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.diag_msg = DiagnosticArray()
        self.diag_status = DiagnosticStatus(name = 'Landing Node:', level = 0, message = 'OK')  # Default Status
        
        self.status_msg = Status()

        # Loop Timing
        rate = rospy.Rate(self.Hertz)
        time.sleep(2)  # allows the callback functions to populate variables

        print("Commencing Landing Node Execution")
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            # Perform calibration during takeoff
            if self.takeoff_initiated:
                if not self.camera_calibration_started:
                    print('Landing Node: Commencing Camera Calibration')
                    self.camera_calibration_started = True  # Only print the above statement once

                # Only process the image if calibration is still needed
                if self.number_of_cal_images <= self.cal_images_needed: 
                    # Convert Image to Greyscale
                    gray_image = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)

                    # Check for corners in the image
                    ret, corners = cv.findChessboardCorners(gray_image, (self.grid_size,self.grid_size), None)

                    # If the corners are seen the image can be processed
                    if ret:
                        corners2 = cv.cornerSubPix(gray_image, corners, (11,11), (-1,-1), criteria)
                        
                        self.object_points.append(self.object_points_index) 
                        self.image_points.append(corners)

                        # Update Calibration Tracker
                        # print("Landing Node: Need {} more images to complete calibration.".format(self.cal_images_needed - self.number_of_cal_images))
                        self.number_of_cal_images = self.number_of_cal_images + 1
                        
                        # Draw Calibration Image
                        cv.drawChessboardCorners(self.image, (self.grid_size,self.grid_size), corners2, ret)    # Comment out for production
                        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(self.object_points, self.image_points, gray_image.shape[::-1], None, None)                  

            # Determine Calibration Status
            if self.camera_calibration_status != self.camera_calibration_status_options[0]: # If the calibration is not done, update the status
                if self.number_of_cal_images == 0:
                    self.camera_calibration_status = self.camera_calibration_status_options[2]
                elif self.number_of_cal_images < self.cal_images_needed:
                    self.camera_calibration_status = self.camera_calibration_status_options[1]
                elif self.number_of_cal_images >= self.cal_images_needed:
                    self.camera_calibration_status = self.camera_calibration_status_options[0]
                    print('Landing Node: Camera Calibration Complete.')

            # Calculate and issue commands if the node has been called on by the global planner
            if not self.goal_reached:
                if self.current_time < (self.start_time + self.drone_stable_time):
                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                else:
                    # Vertical control based on altimeter (barometer)
                    alt_pid = PID(PID_alt[0], PID_alt[1], PID_alt[2], setpoint = self.goal_alt, sample_time= 1/self.Hertz, output_limits=(-1, 2)) # PID Controller
                    if self.odom_alt > self.altitude_ctrl_shift:  # If outside range of the sonic sensor, use the barometer.
                        vel_msg.linear.z = alt_pid(self.odom_alt) 
                    else:                        # In range of the sonic sensor
                        vel_msg.linear.z = alt_pid(self.goal_alt + sonic_dist)
                    
                    # Determine to use GPS or camera for lateral control
                    self.GPS_dist_to_goal = math.sqrt(math.pow(self.goal_x - self.odom_x, 2) + math.pow(self.goal_y - self.odom_y, 2))
                    odom_rotation = Rot.from_quat(self.odom_quat, normalized=True)
                    odom_rotation = odom_rotation.as_euler('xyz', degrees=False)

                    if (self.odom_alt < self.enable_camera_altitude) and (self.number_of_cal_images > 0) and (rospy.get_time() > (self.last_cam_command_time + self.error_recalc)):
                        # Camera must been near visual range and at a minimum a partial calibration done
                        # Lateral control based on the camera
                        # Process Image
                        gray_image = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)

                        # Check for corners in the image
                        ret, corners = cv.findChessboardCorners(gray_image, (self.grid_size,self.grid_size), None)

                        # If the corners are seen the image can be processed
                        if ret:
                            # Identify the inner corners
                            corners2 = cv.cornerSubPix(gray_image, corners, (11,11), (-1,-1), criteria)
                            # Determine the rotation and translation vectors
                            ret, rvecs, tvecs = cv.solvePnP(self.object_points_index, corners2, mtx, dist)
                            # rvecs and tvecs are 3x1 arrays
                            # Determine Image Points
                            image_points, jacobian = cv.projectPoints(self.axis, rvecs, tvecs, mtx, dist)
                            annotated_image = self.draw(self.image, corners2, image_points) # Comment out for production
                            
                            # Form the 4x4 Transform Matrix
                            # rotation = cv.Rodrigues(rvecs)  # 3x3 rotation matrix with 9x3 jacobian
                            # tvecs = np.transpose(tvecs)
                            self.center_from_vehicle = self.image_to_camera_frame(640, 480, image_points[3]) # not compensated for tilt
                            
                            # Define the Offset from GPS
                            m_pix_ratio = self.ratio_lookup(self.odom_alt, self.altitudes, self.ratios)
                            self.cam_x_m = self.center_from_vehicle[0] * m_pix_ratio * 0.05
                            self.cam_y_m = self.center_from_vehicle[1] * m_pix_ratio * 0.05
                            self.cam_GPS_diff = [self.goal_veh[0] - self.cam_x_m, self.goal_veh[1] - self.cam_y_m]

                            self.last_cam_command_time = rospy.get_time()

                    self.goal_veh = self.local_to_vehicle_frame(self.odom_x, self.odom_y, self.goal_x, self.goal_y, odom_rotation)
                    self.goal_veh[0] = self.goal_veh[0] - self.cam_GPS_diff[0]
                    self.goal_veh[1] = self.goal_veh[1] - self.cam_GPS_diff[1]
                    
                    x_pid = PID(PID_x[0], PID_x[1], PID_x[2], setpoint = self.goal_veh[0], sample_time = 1/self.Hertz, output_limits = (-10, 10))
                    y_pid = PID(PID_y[0], PID_y[1], PID_y[2], setpoint = self.goal_veh[1], sample_time = 1/self.Hertz, output_limits = (-10, 10))
                    vel_msg.linear.x = x_pid(0)
                    vel_msg.linear.y = y_pid(0)

                vel_pub.publish(vel_msg)

                # Display the image for development (comment out when not developing)
                cv.imshow("Processed Image", self.image)
                cv.waitKey(3)
                # Determine when the goal is met and tell the global planner
                self.horizontal_error = np.sqrt(np.power(abs(self.goal_veh[0]),2) + np.power(abs(self.goal_veh[1]),2))
                if sonic_dist < alt_threshold:
                    self.landing_check = self.landing_check + 1
                else:
                    self.landing_check = 0

                if (self.landing_check > self.landing_check_threshold) and (self.horizontal_error < horizontal_threshold):
                    print("Landing Complete")
                    self.goal_reached = True
                    self.status_msg.status = self.goal_reached
                    self.status_pub.publish(self.status_msg)
            
            # Publish Diagnostic Info
            self.elapsed_time = self.current_time - self.start_time
            
            if self.goal_reached:
                self.diag_status.values = [ KeyValue(key = 'Node Status', value = 'Standby'),
                                            KeyValue(key = 'Goal Height', value = 'None'),
                                            KeyValue(key = 'Camera Cal',  value = '{}'.format(self.camera_calibration_status))]           
            else:
                self.diag_status.values = [ KeyValue(key = 'Node Status', value = 'Running'),
                                            KeyValue(key = 'Goal Height', value = '{}'.format(self.goal_alt)),
                                            KeyValue(key = 'Camera Cal',  value = '{}'.format(self.camera_calibration_status))]
                if self.elapsed_time > self.should_be_done_time:
                    self.diag_status.level = 1
                    self.diag_status.message = 'Landing Taking Longer Than Expected'
            self.diag_msg.status = [self.diag_status]
            self.diag_pub.publish(self.diag_msg)

            rate.sleep()

        # Close any open cv windows upon terminating the node
        cv.destroyAllWindows()

if __name__ == '__main__':
    
    rospy.init_node('landing')
    try:
        ln = Landing_Node()
    except rospy.ROSInterruptException: pass
