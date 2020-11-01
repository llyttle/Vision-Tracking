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

        #ceate publishers and subscribers
        #create a subscriber to the camera topic
        rospy.Subscriber(self.scan_topic, LaserScan, self.pixel_to_degrees)

        rospy.Subscriber(image_topic, Image, self.process_image_msg)
        #create a publisher to drive the robot
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.lidar_array = []

        #create list to hold ball position (theta, distance)
        self.ball_pos = []
        
        #create an open cv visualization window
        cv2.namedWindow('video_window')
        #create a call back function for when the image in the window is clicked on
    #    cv2.setMouseCallback('video_window', self.process_mouse_event)

    def process_mouse_event(self, event, x,y,flags,param):
        """ A function that is called when the mouse clicks on the open camera window. Function displays a popup with text describing the color value of the camera pixel you clicked on"""
        #create a notification window
        image_info_window = 255*np.ones((500,500,3))
        #display the color in the new popup window
        cv2.putText(image_info_window,'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),(5,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0))
        #display the text in the window
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def process_image_msg(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image. """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_image(self):
        """A function which processes self.cv_image to retrieve important information. It filters self.cv image into binary images which can be used for processing in decision and navigational algorithms. And it retrieves image locations of objects in the scene from the images"""
        self.ball_binary_image = cv2.inRange(self.cv_image, (0,0,80), (20,20,255))


        #process the binary image to get the balls position
        moments = cv2.moments(self.ball_binary_image)
        if moments['m00'] != 0:

            #find the center x and y cords
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
            #print(self.center_x, self.center_y)


    def pixel_to_degrees(self, msg):
        """A function to convert an object's location in a pixel image to an angle and distance in respect to the Neato"""
        vanish_angle = math.radians(43)                 # widest angle in image frame
        f = -300/math.tan(vanish_angle)                 # variable representing camera focal distance

        theta_rad = math.atan((self.center_x-300)/f)    # theta = atan(x/f)
        theta = math.degrees(theta_rad)

        distance = msg.ranges[int(theta)]               # ping degrees of center of object to find distance

        if distance < 10:
            self.ball_pos = [theta, distance]
            print("Theta =      ", self.ball_pos[0])
            print("Distance =   ", self.ball_pos[1])
        else:
            self.ball_pos = []
            print("no object in view")
        
        print("-----------------------------------")

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
        self.pub.publish(msg)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            
            # update the filtered binary images
            self.process_image()

            self.pixel_to_degrees

            # if there is a cv.image
        #    if not self.cv_image is None:
                
                # debug text
            #    print("\n self.cv_image:")
            #    print(self.cv_image.shape)
                
                # display self.cv_image
            #    cv2.imshow('video_window', self.cv_image)
            #    cv2.waitKey(5)

            if not self.ball_binary_image is None:
                # debug text
            #    print("\n self.ball_binary_image: ")
            #    print(self.ball_binary_image.shape)
                
                # display the ball filter image
                cv2.imshow('ball filter',self.ball_binary_image)
                cv2.waitKey(5)        

            # start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()
