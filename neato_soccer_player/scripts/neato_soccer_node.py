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
        #the latest image from the camera
        self.cv_image = None                  
        self.scan_topic = "scan"      
        #used to convert ROS messages to OpenCV
        self.bridge = CvBridge()

        self.goal_in_sight = False
        
        self.found_object = False

        self.last_ball_direction = 1

        #ceate publishers and subscribers
        rospy.Subscriber(self.scan_topic, LaserScan, self.pixel_to_degrees)     # create a subscriber to read LIDAR data
        rospy.Subscriber(image_topic, Image, self.process_image_msg)            # create a subscriber to the camera topic
        rospy.Subscriber('/odom', Odometry, self.get_Goal)                   # create a subscriber to get odom position

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)            # create a publisher to drive the robot
        
        self.last_xy_theta = [0, 0, 0]

        self.lidar_array = []

        #create list to hold ball position (theta, distance)
        self.ball_pos = (0, 0)

        self.velocity = 0
        self.angular = .5
        
        #create an open cv visualization window
        cv2.namedWindow('video_window')
        #create a call back function for when the image in the window is clicked on
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
        
        if cv2.countNonZero(self.ball_binary_image) == 0:
            self.found_object = False
        else:
            self.found_object = True

    def pixel_to_degrees(self, msg):
        """A function to convert an object's location in a pixel image to an angle and distance in respect to the Neato"""
        vanish_angle = math.radians(43)                 # widest angle in image frame
        f = -300/math.tan(vanish_angle)                 # variable representing camera focal distance

        if self.found_object == True:
            theta_rad = math.atan((self.center_x-300)/f)    # theta = atan(x/f)
            theta = int(math.degrees(theta_rad))

            distance = msg.ranges[theta]               # ping degrees of center of object to find distance
            
            self.ball_pos = (theta, distance)
            
            print("Theta =      ", self.ball_pos[0])
            print("Distance =   ", self.ball_pos[1])

            self.last_ball_direction = -(self.center_x-300)/abs(self.center_x-300) 
        else:
            self.ball_pos = None
            print(self.ball_pos)

        print("-----------------------------------")

    def face_ball(self):
        error_margin = 1        #margin that the robot will consider "close enough" of straight forward
        if self.ball_pos != None:
            if self.ball_pos[0] < 0-error_margin or self.ball_pos[0] > 0+error_margin:
                turn = self.ball_pos[0]/50
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
            adjusted_position = xy_theta_position + xy_theta_adjust

            theta1, d1 = self.cart2pol(goal1_pos[0]-adjusted_position[0], goal1_pos[1]-adjusted_position[1])
            theta2, d2 = self.cart2pol(goal2_pos[0]-adjusted_position[0], goal2_pos[1]-adjusted_position[1])

            goal1_vec = (math.degrees(theta1 + adjusted_position[2]), d1)
            goal2_vec = (math.degrees(theta2 - adjusted_position[2]), d2)

            print(goal1_vec)
            print(goal2_vec)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            
            # update the filtered binary images
            self.process_image()
            
            self.get_Goal
            self.pixel_to_degrees
            
            if self.ball_pos == None or self.ball_pos[1] > 2:
                self.msg = self.face_ball()
            else:
                self.msg = self.kick()

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


            self.pub.publish(self.msg)

            # start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()
