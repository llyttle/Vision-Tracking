#!/usr/bin/env python3

""" This is a script that walks through some of the basics of working with
    images with opencv in ROS. """

#imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import math
import numpy as np
from geometry_msgs.msg import Twist, Vector3

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
        
        self.found_object = False

        self.last_ball_direction = 1

        #ceate publishers and subscribers
        #create a subscriber to the camera topic
        rospy.Subscriber(image_topic, Image, self.process_image_msg)
        rospy.Subscriber(self.scan_topic, LaserScan, self.process_laser_msg)
        #create a publisher to drive the robot
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lidar_array = []

        #create list to hold ball position (theta, distance)
        #self.ball_pos = []
        self.ball_pos = (0, 0)

        self.velocity = 0
        self.angular = .5
        
        #create an open cv visualization window
        cv2.namedWindow('video_window')
        #create a call back function for when the image in the window is clicked on
        cv2.setMouseCallback('video_window', self.process_mouse_event)
        
        self.laser_scan_data = None
    
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

    def face_ball(self):
        error_margin = 1        #margin that the robot will consider "close enough" of straight forward
        if self.ball_pos_data[0] != None:
            if self.ball_pos_data[0] < 0-error_margin or self.ball_pos_data[0] > 0+error_margin:
                turn = self.ball_pos_data[0]/50
            else:
                turn = 0
            speed = 1
        else:
            turn = self.last_ball_direction
            speed = 0

        msg = Twist(Vector3(speed, 0, 0),Vector3(0, 0, turn))
        return msg
    
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

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            
            # update the filtered binary images
            self.process_image()
            
            if self.ball_pos_data[1] == None or self.ball_pos_data[0] == None or self.ball_pos_data[1] > 2:
                self.msg = self.face_ball()
            else:
                self.msg = self.kick()

            if not self.cv_image is None:
                
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
