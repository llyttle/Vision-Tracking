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

class Helper_Functions(object):
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

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        #the latest image from the camera
        self.cv_image = None                  
        self.scan_topic = "scan"      
        #used to convert ROS messages to OpenCV
        self.bridge = CvBridge()

        self.goal_in_sight = False
        
        self.found_object = False

        self.last_ball_direction = 1

        # Initialize helper functions class
        self.Convert = Helper_Functions()
        
        #ceate publishers and subscribers
        rospy.Subscriber(self.scan_topic, LaserScan, self.process_laser_msg)     # create a subscriber to read LIDAR data
        rospy.Subscriber(image_topic, Image, self.process_image_msg)            # create a subscriber to the camera topic
        rospy.Subscriber('/odom', Odometry, self.get_odom)                   # create a subscriber to get odom position

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)            # create a publisher to drive the robot

        #create an open cv visualization window
        cv2.namedWindow('video_window')
        #create a call back function for when the image in the window is clicked on
        cv2.setMouseCallback('video_window', self.process_mouse_event)
        
        self.laser_scan_data = None
    
    #    cv2.setMouseCallback('video_window', self.process_mouse_event)
    def cart2pol(self, x, y):
        """helper function for converting cartesian coordinates to polar coordinates"""
        theta = math.atan2(y, x)
        d = np.hypot(x, y)
        return (theta, d)

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

        print("ball:",self.ball_pos_data)
        print("blue_goal:", self.blue_goal_pos_data)
        print("yellow_goal:", self.yellow_goal_pos_data)        

        if self.ball_pos_data[2] == True:
            self.last_ball_direction = -(found_ball_data[1]-300)/abs(found_ball_data[1]-300)
        
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

        print("-----------------------------------")
        return (obj_pos[0],obj_pos[1],found_object_data[0])        

<<<<<<< HEAD
<<<<<<< HEAD
    def search_for_ball(self):
        angvel = self.last_ball_direction
        linvel = 0

        msg = Twist(Vector3(linvel,0,0), Vector3(0,0,angvel))

        return msg
    
    def get_odom(self, odom_data):
        pose = odom_data.pose.pose
        orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        yaw = euler_from_quaternion(orientation_list)[2]
        xy_theta_position = np.array([pose.position.x, pose.position.y, yaw])

        self.robot_position = xy_theta_position #+ xy_theta_adjust

    def position_neato(self):
        """ Find where the neato needs to be to kick the ball into the goal and drive to that position.
            This function requires a (theta, d) vector for the desired goal and for the ball
        """
        # Position of Goal and Ball in map
        goal_map = np.array([8, 0])
        
        theta_ball = self.robot_position[2]
        neato2map_matrix = np.array([[math.cos(theta_ball), -math.sin(theta_ball), self.robot_position[0]],
                                     [math.sin(theta_ball),  math.cos(theta_ball), self.robot_position[1]],
                                     [0,                0,                         1]])

        ball_matrix = np.append(self.Convert.pol2cart(math.radians(self.ball_pos_data[0]), self.ball_pos_data[1]), 1)
        ball_map_3D = neato2map_matrix.dot(ball_matrix)
        ball_map = ball_map_3D[:-1]
        
        # Vector from goal to the ball
        goal2ball = ball_map - goal_map
        # Extending the vector from the goal to the ball to get lineup position
        theta, d = self.Convert.cart2pol(goal2ball[0], goal2ball[1])

        desired_position_from_goal = self.Convert.pol2cart(theta, d+2)

        # defining linup position in terms of the map, not the goal
        desired_position_map = goal_map + desired_position_from_goal

        print(desired_position_map)

    def kick_ball(self):
        error_margin = 1        # Margin that the robot will consider "close enough" of straight forward

        # if the ball is farther than 2 meters, go towards the ball
        if self.ball_pos_data[1] > 2:
=======
<<<<<<< HEAD
=======
>>>>>>> Find linup position in map frame
    def face_ball(self):
        error_margin = 1        #margin that the robot will consider "close enough" of straight forward
        if self.ball_pos_data[0] != None:
>>>>>>> Combine kick and face_ball functions and organize arbiter
            if self.ball_pos_data[0] < 0-error_margin or self.ball_pos_data[0] > 0+error_margin:
                angvel = self.ball_pos_data[0]/50
            else:
                    angvel = 0
            linvel = 1
        # if the ball is closer than 2 meters, kick the ball
        else:
            # move at 10 m/s straight        
            linvel = 10
            angvel = 0
            msg = Twist(Vector3(linvel,0,0), Vector3(0,0,angvel))
            # send the message to the robot`
            self.pub.publish(msg)

            # move forward for 2 seconds
            rospy.sleep(2.0)

            # stop
            linvel = 0

        msg = Twist(Vector3(linvel,0,0), Vector3(0,0,angvel))

        return msg
    
<<<<<<< HEAD
    def Arbiter(self):
        """ Controller function for soccer player. Manages the following behaviors:
            if no ball -- search for ball
            if ball -- position behind ball
            if in position -- kick the ball
        """
        if self.ball_pos_data[2] == False: #and self.positioning == False:
            self.msg = self.search_for_ball()
        #elif self.ball_pos_data != None: 
            #self.msg = self.position_neato()
        #    pass
        else: #self.ball_pos_data != None and self.in_position == True:       # """in position""":
            self.position_neato()
            self.msg = self.kick_ball()
=======
    def kick(self):
        """ this is a function that tells the neato to kick the ball"""
        #move at 10 m/s straight        
        linvel = Vector3(10,0,0)
        angvel = Vector3(0,0,0)
        msg = Twist(linvel,angvel)
        #send the message to the robot`
        self.pub.publish(msg)

        #move forward for 2 seconds
        rospy.sleep(2.0)

        #stop
        linvel = Vector3(0,0,0)
        msg = Twist(linvel,angvel)
        return msg
>>>>>>> Find linup position in map frame

    def get_Goal(self, odom_data):
        """This function finds the position of the goal
        
        if goal in sight:
            goal_position = TIMS CODE
            adjust robot position for any error
        
        elif goal not in sight:
            goal_position = ROBOT_TRANSFORM(previous_goal_position)
        """
        #positions of each goal in map frame (x, y)
        goal1_pos = (8, 0)
        goal2_pos = (-8, 0)

        #finding the current position of the robot (x, y, theta)
        pose = odom_data.pose.pose
        orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        yaw = euler_from_quaternion(orientation_list)[2]
        xy_theta_position = np.array([pose.position.x, pose.position.y, yaw])

        if self.goal_in_sight == True:
            #TODO: put Tim's code here

            #Re-defining the position of the robot when the real goal is in sight.
            """ if last_goal_position != current_goal_position:
                    current_goal_position_xy_theta = 
                    last_goal_position_xy_theta = 
                    xy_theta_adjust = current_goal_position_xy_theta - last_goal_position_xy_theta
            """
            pass

        elif self.goal_in_sight == False:
            adjusted_position = xy_theta_position #+ xy_theta_adjust

            theta1, d1 = self.cart2pol(goal1_pos[0]-adjusted_position[0], goal1_pos[1]-adjusted_position[1])
            theta2, d2 = self.cart2pol(goal2_pos[0]-adjusted_position[0], goal2_pos[1]-adjusted_position[1])

            goal1_vec = (math.degrees(theta1 + adjusted_position[2]), d1)
            goal2_vec = (math.degrees(theta2 - adjusted_position[2]), d2)

            print(goal1_vec)
            print(goal2_vec)

    def Arbiter(self):
        if self.ball_pos == None or self.ball_pos[1] > 2:
            self.msg = self.face_ball()
        else:
            self.msg = self.kick()


    def get_Goal(self, odom_data):
        """This function finds the position of the goal
        
        if goal in sight:
            goal_position = TIMS CODE
            adjust robot position for any error
        
        elif goal not in sight:
            goal_position = ROBOT_TRANSFORM(previous_goal_position)
        """
        #positions of each goal in map frame (x, y)
        goal1_pos = (8, 0)
        goal2_pos = (-8, 0)

        #finding the current position of the robot (x, y, theta)
        pose = odom_data.pose.pose
        orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        yaw = euler_from_quaternion(orientation_list)[2]
        xy_theta_position = np.array([pose.position.x, pose.position.y, yaw])

        if self.goal_in_sight == True:
            #TODO: put Tim's code here

            #Re-defining the position of the robot when the real goal is in sight.
            """ if last_goal_position != current_goal_position:
                    current_goal_position_xy_theta = 
                    last_goal_position_xy_theta = 
                    xy_theta_adjust = current_goal_position_xy_theta - last_goal_position_xy_theta
            """
            pass

        elif self.goal_in_sight == False:
            adjusted_position = xy_theta_position #+ xy_theta_adjust

            theta1, d1 = self.cart2pol(goal1_pos[0]-adjusted_position[0], goal1_pos[1]-adjusted_position[1])
            theta2, d2 = self.cart2pol(goal2_pos[0]-adjusted_position[0], goal2_pos[1]-adjusted_position[1])

            goal1_vec = (math.degrees(theta1 + adjusted_position[2]), d1)
            goal2_vec = (math.degrees(theta2 - adjusted_position[2]), d2)

            print(goal1_vec)
            print(goal2_vec)

    def Arbiter(self):
        if self.ball_pos == None or self.ball_pos[1] > 2:
            self.msg = self.face_ball()
        else:
            self.msg = self.kick()


    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            # Code to run constantly
            self.process_image()        # update the filtered binary images
=======
<<<<<<< HEAD
            # update the filtered binary images
            self.process_image()
            
<<<<<<< HEAD
            if self.ball_pos_data[1] == None or self.ball_pos_data[0] == None or self.ball_pos_data[1] > 2:
=======
            self.get_Goal
            self.pixel_to_degrees
            
            if self.ball_pos == None or self.ball_pos[1] > 2:
>>>>>>> Track the position of the goal if not in view
                self.msg = self.face_ball()
            else:
                self.msg = self.kick()
>>>>>>> Clean up code
=======
            # Code to run constantly
            self.process_image()        # update the filtered binary images
            self.get_Goal               # get (theta, distance) of the goal
            self.pixel_to_degrees       # if there is a ball find position and update self.ball_pos
>>>>>>> Clean up code

            # Arbiter to controll behaviors
            self.Arbiter()
            
<<<<<<< HEAD
            # if there is a cv.image
            if not self.cv_image is None:
=======
=======
>>>>>>> Find linup position in map frame
            # Code to run constantly
            self.process_image()        # update the filtered binary images

            # Arbiter to controll behaviors
            self.Arbiter()
            
=======
>>>>>>> Clean up code
            # if there is a cv.image
        #    if not self.cv_image is None:
                
                # debug text
            #    print("\n self.cv_image:")
            #    print(self.cv_image.shape)
                
                # display self.cv_image
                cv2.imshow('video_window', self.cv_image)
                cv2.waitKey(5)

            if not self.ball_binary_image is None:
                # debug text
            #    print("\n self.ball_binary_image: ")
            #    print(self.ball_binary_image.shape)
                
                # display the ball filter image
                cv2.imshow('ball_filter',self.ball_binary_image)
                cv2.waitKey(5)        
            

            self.pub.publish(self.msg)

            # start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()
