#!/usr/bin/env python3

""" This is a script that walks through some of the basics of working with
    images with opencv in ROS. """

#ROS imports
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose
#Open CV imports
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
#helper imports
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        
        # the latest image from the camera
        self.cv_image = None                  
        self.scan_topic = "scan"      
        # used to convert ROS messages to OpenCV
        self.bridge = CvBridge()

        # ceate publishers
        rospy.Subscriber(self.scan_topic, LaserScan, self.process_laser_msg)     # create a subscriber to read LIDAR data
        rospy.Subscriber(image_topic, Image, self.process_image_msg)            # create a subscriber to the camera topic
        rospy.Subscriber('/odom', Odometry, self.get_odom)                   # create a subscriber to get odom position
        # create subscribers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)            # create a publisher to drive the robot

        #create an open cv visualization window
        cv2.namedWindow('video_window')
        #create a call back function for when the image in the window is clicked on
        cv2.setMouseCallback('video_window', self.process_mouse_event)
        
        self.goal_in_sight = False
        self.found_object = False
        self.last_ball_direction = 1
        self.laser_scan_data = None
        self.Driving_to_Pos = None
        self.state = 0

#HELPER FUNCTIONS ==============================================================================================================
#===============================================================================================================================
    def cart2pol(self, x, y):
        """helper function for converting cartesian coordinates to polar coordinates"""
        theta = math.atan2(y, x)
        d = np.hypot(x, y)
        return (theta, d)
    
    def pol2cart(self, theta, d):
        """helper function for converting cartesian coordinates to polar coordinates"""
        x = d * math.cos(theta)
        y = d * math.sin(theta)
        return np.array([x, y])
    
    def get_odom(self, odom_data):
        pose = odom_data.pose.pose
        orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        yaw = euler_from_quaternion(orientation_list)[2]
        xy_theta_position = np.array([pose.position.x, pose.position.y, yaw])

        self.robot_position = xy_theta_position #+ xy_theta_adjust

    def neato2map(self, theta, distance):
        theta_robot = self.robot_position[2]
        neato2map_matrix = np.array([[math.cos(theta_robot), -math.sin(theta_robot), self.robot_position[0]],
                                     [math.sin(theta_robot),  math.cos(theta_robot), self.robot_position[1]],
                                     [0,                      0,                     1]])
        point_vector = np.append(self.pol2cart(theta, distance), 1)
        point_map_3D = neato2map_matrix.dot(point_vector)
        point_map = point_map_3D[:-1]

        return point_map

    def map2neato(self, x, y):
        theta_robot = self.robot_position[2]
        neato2map_matrix = np.array([[math.cos(theta_robot), -math.sin(theta_robot), self.robot_position[0]],
                                     [math.sin(theta_robot),  math.cos(theta_robot), self.robot_position[1]],
                                     [0,                      0,                     1]])
        map2neato_matrix = np.linalg.inv(neato2map_matrix)
        point_vector = np.array([x, y, 1])
        point_neato_3D = map2neato_matrix.dot(point_vector)
        theta, distance = self.cart2pol(point_neato_3D[0], point_neato_3D[1])
        point_neato = np.array([theta, distance])

        return point_neato

    def pixel_to_degrees(self, found_object_data):
        """A function to convert an object's location in a pixel image to an angle and distance in respect to the Neato"""
        #widest angle in image frame
        vanish_angle = math.radians(43)
        #variable representing camera focal distance
        f = -300/math.tan(vanish_angle)

        if found_object_data[0] == True:
            theta_rad = math.atan((found_object_data[1]-300)/f)    # theta = atan(x/f)
            theta = int(math.degrees(theta_rad))
            
            if self.laser_scan_data != None:
                #ping the object to find the distance in the laser scan
                distance = self.laser_scan_data[theta]             
            else:
                distance = 100
            obj_pos = (theta, distance)
        else:
            obj_pos = (None,None)

        return (obj_pos[0],obj_pos[1],found_object_data[0])        
#===============================================================================================================================
#===============================================================================================================================

    def process_mouse_event(self, event, x,y,flags,param):
        """ A function that is called when the mouse clicks on the open camera window. Function displays a popup with text describing the color value of the camera pixel you clicked on"""
        #create a notification window
        image_info_window = 255*np.ones((500,500,3))
        #display the color in the new popup window
        cv2.putText(image_info_window,'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),(5,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0))
        #display the text in the window
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def process_laser_msg(self, msg):
        """process laser scann images from ROS and stash them in an atrivute called laser_scan"""
        self.laser_scan_data = msg.ranges

    def process_image_msg(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image. """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def find_object_in_binary_image(self, binary_image):
        moments = cv2.moments(binary_image)

        #process the binary image to get the balls position
        if cv2.countNonZero(binary_image) == 0:
            found = False
        else:
            found = True

        if moments['m00'] != 0:
            #find the center x and y cords
            center_x = moments['m10']/moments['m00']
            center_y = moments['m01']/moments['m00']
    
            return (found,center_x, center_y)
        
        return (found, None ,None)

    def process_image(self):
        """A function which processes self.cv_image to retrieve important information. It filters self.cv image into binary images which can be used for processing in decision and navigational algorithms. And it retrieves image locations of objects in the scene from the images"""
        self.ball_binary_image = cv2.inRange(self.cv_image, (0,0,80), (20,20,255))
        self.blue_goal_binary_image = cv2.inRange(self.cv_image,(95,19,19),(110,27,27))
        self.yellow_goal_binary_image = cv2.inRange(self.cv_image,(40,220,220),(53,240,240))

        #find the position of the ball 
        #find the image coordinates of the ball
        found_ball_data = self.find_object_in_binary_image(self.ball_binary_image)
        #convert the image coordinates into an angle and a distance relative to the bot
        self.ball_pos_data = self.pixel_to_degrees(found_ball_data)        

        #find the position of the blue goal
        found_blue_goal_data = self.find_object_in_binary_image(self.blue_goal_binary_image)
        self.blue_goal_pos_data = self.pixel_to_degrees(found_blue_goal_data)
        
        #find the position of the yellow goal
        found_yellow_goal_data = self.find_object_in_binary_image(self.yellow_goal_binary_image)
        self.yellow_goal_pos_data = self.pixel_to_degrees(found_yellow_goal_data)

    #    print("ball:",self.ball_pos_data)
    #    print("blue_goal:", self.blue_goal_pos_data)
    #    print("yellow_goal:", self.yellow_goal_pos_data)        

        if self.ball_pos_data[2] == True:
            self.last_ball_direction = -(found_ball_data[1]-300)/abs(found_ball_data[1]-300)

    def search_for_ball(self):
        angvel = self.last_ball_direction
        linvel = 0

        msg = Twist(Vector3(linvel,0,0), Vector3(0,0,angvel))
        return msg

    def get_lineup_pos(self):
        # Position of Goal and Ball in map
        goal_map = np.array([8, 0])
        ball_map = self.neato2map(math.radians(self.ball_pos_data[0]), self.ball_pos_data[1]+.5)

        # Find desired_position in map frame
        goal2ball = ball_map - goal_map                                 # Vector from goal to the ball
        theta, d = self.cart2pol(goal2ball[0], goal2ball[1])
        desired_position_from_goal = self.pol2cart(theta, d+2)          # Extending the vector from the goal to the ball
        self.desired_position_map = goal_map + desired_position_from_goal    # defining linup position in terms of the map, not the goal
        print(self.ball_pos_data)

    def position_neato(self):
        """ Find where the neato needs to be to kick the ball into the goal and drive to that position.
            This function requires a (theta, d) vector for the desired goal and for the ball
        """
        # Desired_position in terms of the neato
        desired_position = self.map2neato(self.desired_position_map[0], self.desired_position_map[1])

        #move neato to desired_position
        if desired_position[1] >= .3:
            angvel = math.degrees(desired_position[0])/30


            linvel = 1
            self.Driving_to_Pos = True
        else:
            angvel = 0
            linvel = 0
            self.Driving_to_Pos = False
        
        msg = Twist(Vector3(linvel,0,0), Vector3(0,0,angvel))
        return msg

    def kick_ball(self):
        #find the ball again
        if self.ball_pos_data[2] == False:
            msg = self.search_for_ball()
        else:
            # face the ball
            if abs(math.degrees(self.ball_pos_data[0])) > 1:
                angvel = self.ball_pos_data[0]/30
                linvel = 0
            # kick the ball
            else:
                # move at 3 m/s straight        
                linvel = 3
                angvel = 0
                msg = Twist(Vector3(linvel,0,0), Vector3(0,0,angvel))
                # send the message to the robot`
                self.pub.publish(msg)

                # move forward for 2 seconds
                rospy.sleep(2.0)

                # stop
                linvel = 0
                self.state = 0

            msg = Twist(Vector3(linvel,0,0), Vector3(0,0,angvel))
        return msg
    
    def Arbiter(self):
        """ Controller function for soccer player. Manages the following behaviors:
            if no ball -- search for ball
            if ball -- position behind 
                when going to desire position, dont need to see the ball
            if in position -- kick the ball
        """
        self.msg = Twist(Vector3(0,0,0), Vector3(0,0,0))        
        
        print(self.state)
        if self.state == 0:
            if self.ball_pos_data[2] == False:
                self.msg = self.search_for_ball()
            elif self.ball_pos_data[1] < 10:
                self.get_lineup_pos()
                self.state = 1
            else:
                self.msg = Twist(Vector3(1,0,0), Vector3(0,0,0))
        if self.state == 1:
            self.msg = self.position_neato()
            if self.Driving_to_Pos == False:
                self.state = 2
        if self.state == 2:
            self.msg = self.kick_ball()
        
    def run(self):
        """ The main run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            
            # update the filtered binary images
            self.process_image()

            # Arbiter to controll behaviors
            self.Arbiter()
            
            # display self.cv_image
            if not self.cv_image is None:
                cv2.imshow('video_window', self.cv_image)
                cv2.waitKey(5)

            # display the ball filter image
            if not self.ball_binary_image is None:
                cv2.imshow('ball_filter',self.ball_binary_image)
                cv2.waitKey(5)        
            
            self.pub.publish(self.msg)

            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()
