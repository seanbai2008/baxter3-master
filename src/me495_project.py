#!/usr/bin/env python

import rospy
import roslib

import cv
import cv2
import cv_bridge
import copy

import numpy
import math
import os
import sys
import string
import time
import random
import tf
import argparse
from sensor_msgs.msg import Image
import baxter_interface
from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
#from PIL import Image
from PIL import Image as PIL_Image
from sensor_msgs.msg import JointState

# load the package manifest
#reads the package manifest and sets up the python library path based on the package dependencies.
roslib.load_manifest("baxter3")

# initialise ros node
rospy.init_node("pick_and_place", anonymous = True)

# directory used to save analysis images
image_directory = os.getenv("HOME") + "/Project/"

# locate class
class locate():
    #initialization function: limb used, gripper, tolerences, workspace, vision parameters, start position
    def __init__(self, arm, distance):
        global image_directory

        #Interface class for a limb on the Baxter robot.
        # arm ("left" or "right")
        self.limb           = arm
        self.limb_interface = baxter_interface.Limb(self.limb)

        if arm == "left":
            self.other_limb = "right"
        else:
            self.other_limb = "left"

        self.other_limb_interface = baxter_interface.Limb(self.other_limb)

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(arm)

        # image directory
        self.image_dir = image_directory

        # flag to control saving of analysis images
        self.save_images = True

        # required position accuracy in metres
        self.ball_tolerance = 0.005
        #self.cube_tolerance = 0.005
        self.tray_tolerance = 0.05


        # adding movement for if ball not found. 
        self.no_circles = "left"

        # start positions
        self.ball_tray_x = 0.25                        # x     = front back
        self.ball_tray_y = 0.60                        # y     = left right
        self.ball_tray_z = 0.15                        # z     = up down
        self.green_tray_x = 0.50                        # x     = front back
        self.green_tray_y = 0.60                        # y     = left right
        self.green_tray_z = 0.15                        # z     = up down
        self.objects_x = 0.50                        # x     = front back
        self.objects_y = 0.30                        # y     = left right
        self.objects_z = 0.15                        # z     = up down
        self.roll        = -1.0 * math.pi              # roll  = horizontal
        self.pitch       = 0.0 * math.pi               # pitch = vertical
        self.yaw         = 0.0 * math.pi               # yaw   = rotation

        self.ymin=-0.7
        self.xmax= 0.4

        self.pose = [self.objects_x, self.objects_y, self.objects_z,     \
                     self.roll, self.pitch, self.yaw]

        # camera parameters (NB. other parameters in open_camera)
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0#0.025                      # camera gripper offset
        self.cam_y_offset = -0.025
        self.width        = 960                        # Camera resolution
        self.height       = 600

        # Hough circle accumulator threshold and minimum radius.
        self.hough_accumulator = 20#35
        self.hough_min_radius  = 30 # need to define limits appropriate for our objects
        self.hough_max_radius  = 60 # need to define limits appropriate for our objects

        # canny image
        self.canny = cv.CreateImage((self.width, self.height), 8, 1)

        # Canny transform parameters
        self.canny_low  = 110
        self.canny_high = 150

        self.robotposition=JointState()
        
        # minimum ball tray area
        #use to identify droping area
        self.min_area = 20000

        # callback image
        self.cv_image = cv.CreateImage((self.width, self.height), 8, 3)

        # colours
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)

        # initiallize ball tray corners array --> i dont think this is required, but if we find we need the find_corners function add back in
        #self.ball_tray_corner = [(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0)]


        # Enable the actuators
        baxter_interface.RobotEnable().enable()

        # set speed as a ratio of maximum speed
        self.limb_interface.set_joint_position_speed(0.5)
        self.other_limb_interface.set_joint_position_speed(0.5)

        # create image publisher to head monitor
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True,queue_size = 10)

        # calibrate the gripper
        self.gripper.calibrate()

        # display the start splash screen
        self.splash_screen("ME 495", "Pick and Place")

        # reset cameras
        self.reset_cameras()  #call function below

        # close all cameras
        #self.close_camera("left")
        #        self.close_camera("right") #call function below
        #        self.close_camera("head") #call function below

        # open required camera
        self.open_camera(self.limb, self.width, self.height) # open camera on the limb in motion only via function below

        # subscribe to required camera
        self.subscribe_to_camera(self.limb)

        # distance of arm to table and ball tray
        self.distance      = distance #distance found by running the setup file
        self.tray_distance = distance - 0.075

        # move other arm out of harms way
        if arm == "left":
            self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, 0.0))
        else:
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))

    # display message on head display
    def splash_screen(self, s1, s2):
        splash_array = numpy.zeros((self.height, self.width, 3), numpy.uint8)
        font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 3.0, 3.0, 9)

        ((text_x, text_y), baseline) = cv.GetTextSize(s1, font)
        org = ((self.width - text_x) / 2, (self.height / 3) + (text_y / 2))
        cv2.putText(splash_array, s1, org, cv.CV_FONT_HERSHEY_SIMPLEX, 3.0,          \
                    self.white, thickness = 7)

        ((text_x, text_y), baseline) = cv.GetTextSize(s2, font)
        org = ((self.width - text_x) / 2, ((2 * self.height) / 3) + (text_y / 2))
        cv2.putText(splash_array, s2, org, cv.CV_FONT_HERSHEY_SIMPLEX, 3.0,          \
                    self.white, thickness = 7)

        splash_image = cv.fromarray(splash_array)
        splash_image = numpy.asarray(splash_image)
        # 3ms wait
        cv2.waitKey(3)

        msg = cv_bridge.CvBridge().cv2_to_imgmsg(splash_image, encoding="bgr8")
        self.pub.publish(msg)

    # reset all cameras (incase cameras fail to be recognised on boot)
    def reset_cameras(self):
        reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
        rospy.wait_for_service('cameras/reset', timeout=10)
        reset_srv()

    # open a camera and set camera parameters
    def open_camera(self, camera, x_res, y_res):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - open_camera - Invalid camera")

        # close camera
        #        cam.close()

        # set camera parameters
        print (int(x_res), int(y_res))
        cam.resolution          = (int(x_res), int(y_res))
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # open camera
        cam.open()

    # close a camera
    def close_camera(self, camera):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - close_camera - Invalid camera")

        # set camera parameters to automatic
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # close camera
        cam.close()

    # camera call back function
    def camera_callback(self, data, camera_name):
        # Convert image from a ROS image message to a CV image
        try:
            self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.CvBridgeError, e:
            print e

        # 3ms wait
        cv.WaitKey(3)

    # left camera call back function
    def left_camera_callback(self, data):
        self.camera_callback(data, "Left Hand Camera")

    # right camera call back function
    def right_camera_callback(self, data):
        self.camera_callback(data, "Right Hand Camera")

    # head camera call back function
    def head_camera_callback(self, data):
        self.camera_callback(data, "Head Camera")

    # create subscriber to the required camera
    def subscribe_to_camera(self, camera):
        if camera == "left":
            callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        elif camera == "head":
            callback = self.head_camera_callback
            camera_str = "/cameras/head_camera/i3mage"
        else:
            sys.exit("ERROR - subscribe_to_camera - Invalid camera")

        camera_sub = rospy.Subscriber(camera_str, Image, callback)

    # inverse kinematic function used to move baxter --> replace with Move It! function
    def baxter_ik_move(self, limb, rpy_pose):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        iterate_IK = True 
        n_iterations = 0
        ik_request.pose_stamp.append(quaternion_pose)
        while iterate_IK:
            try:
                rospy.wait_for_service(node, 5.0)
                ik_response = ik_service(ik_request)
            except (rospy.ServiceException, rospy.ROSException), error_message:
                rospy.logerr("Service request failed: %r" % (error_message,))
                sys.exit("ERROR - baxter_ik_move - Failed to append pose")

            if ik_response.isValid[0]:
                print("PASS: Valid joint configuration found")
                # convert response to joint position control dictionary
                limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
                # move limb
                if self.limb == limb:
                    self.limb_interface.move_to_joint_positions(limb_joints)
                else:
                    self.other_limb_interface.move_to_joint_positions(limb_joints)
                iterate_IK = False
            else:
                # display invalid move message on head display
                self.splash_screen("Iterating", "IK")
                # little point in continuing so exit with error message
                #print "requested move =", rpy_pose

                #sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")          
            
                #print locator.limb_interface.joint_names()
                #print locator.limb_interface.joint_angles()

                # Changing initial seed to try and solve for IK solution
                currentJointState = rospy.wait_for_message("/robot/joint_states",JointState)
                newJointState=copy.copy(currentJointState)

                seed_angle_val=JointState()
                seed_angle_val.header.stamp = rospy.Time.now()
                current_angle = []
                name_val = []
                for i in range (2,9):
                    temp_name = copy.copy(currentJointState.name[i])
                    temp_angle = copy.copy(currentJointState.position[i])
                    temp_angle = temp_angle+(random.random()-0.5)/10
                    name_val.append(temp_name)
                    current_angle.append(temp_angle)
                
                #print name_val
                #print current_angle

                seed_angle_val.name=name_val
                seed_angle_val.position=current_angle
                #print seed_angle_val
                seed_angle_list=[seed_angle_val]
                #print seed_angle_list 
                ik_request.seed_angles = seed_angle_list
                n_iterations=n_iterations+1
                print n_iterations

            
            if n_iterations > 50:
                # display invalid move message on head display
                self.splash_screen("Invalid IK", "Solution")
                #sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")
                print 'IK did not converge after %d iterations' %n_iterations
                return 

        if self.limb == limb:               # if working arm
            quaternion_pose = self.limb_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            self.pose = [position[0], position[1],                                \
                         self.pose[2], self.pose[3], self.pose[4], self.pose[5]]

    # Not a tree walk due to python recursion limit
    def tree_walk(self, image, x_in, y_in):
        almost_black = (1, 1, 1)

        pixel_list = [(x_in, y_in)]                   # first pixel is black save position
        cv.Set2D(image, y_in, x_in, almost_black)     # set pixel to almost black
        to_do = [(x_in, y_in - 1)]                    # add neighbours to to do list
        to_do.append([x_in, y_in + 1])
        to_do.append([x_in - 1, y_in])
        to_do.append([x_in + 1, y_in])

        while len(to_do) > 0:
            x, y = to_do.pop()                             # get next pixel to test
            if cv.Get2D(image, y, x)[0] == self.black[0]:  # if black pixel found
                pixel_list.append([x, y])                  # save pixel position
                cv.Set2D(image, y, x, almost_black)        # set pixel to almost black
                to_do.append([x, y - 1])                   # add neighbours to to do list
                to_do.append([x, y + 1])
                to_do.append([x - 1, y])
                to_do.append([x + 1, y])

        return pixel_list

    # update pose in x and y direction
    def update_pose(self, dx, dy):
        x = self.pose[0] + dx
        y = self.pose[1] + dy
        pose = [x, y, self.pose[2], self.roll, self.pitch, self.yaw]
        self.baxter_ik_move(self.limb, pose)
        
    # print all 6 arm coordinates (only required for programme development)
    def print_arm_pose(self):
        return
        pi = math.pi

        quaternion_pose = self.limb_interface.endpoint_pose()
        position        = quaternion_pose['position']
        quaternion      = quaternion_pose['orientation']
        euler           = tf.transformations.euler_from_quaternion(quaternion)

        print
        print "             %s" % self.limb
        print 'front back = %5.4f ' % position[0]
        print 'left right = %5.4f ' % position[1]
        print 'up down    = %5.4f ' % position[2]
        print 'roll       = %5.4f radians %5.4f degrees' %euler[0], 180.0 * euler[0] / pi
        print 'pitch      = %5.4f radians %5.4f degrees' %euler[1], 180.0 * euler[1] / pi
        print 'yaw        = %5.4f radians %5.4f degrees' %euler[2], 180.0 * euler[2] / pi

    # randomly adjust a pose to dither arm position
    # used to prevent stalemate when looking for ball tray
    def dither(self):
        x = self.ball_tray_x
        y = self.ball_tray_y + (random.random() / 10.0)
        pose = (x, y, self.ball_tray_z, self.roll, self.pitch, self.yaw)

        return pose

    # find next object of interest
    #send Hough circle data to determine which ball to select first
    def find_next_object(self, ball_data, iteration):
        # if only one object then object found
        if len(ball_data) == 1:
            return ball_data[0]

        # sort objects right to left
        od = []
        for i in range(len(ball_data)):
            od.append(ball_data[i])

        od.sort()
        return od[0]
        # #we want to select the right most object --> apply to both halves of the workspace with the drop off in the center so both arms can sort at once without collision 
        # # if one ball is significantly to the right of the others
        # if od[1][0] - od[0][0] > 30:       # if ball significantly to right of the others
        #     return od[0]                   # return right most ball
        # elif od[1][1] < od[0][1]:          # if right most ball below second ball
        #     return od[0]                   # return lower ball
        # else:                              # if second ball below right most ball
        #     return od[1]                   # return lower ball

    # find gripper angle to avoid nearest neighbour
    def find_gripper_angle(self, next_ball, ball_data):
        # if only one ball any angle will do
        if len(ball_data) == 1:
            return self.yaw

        # find nearest neighbour
        neighbour = (0, 0)
        min_d2    = float(self.width * self.width + self.height * self.height)

        for i in range(len(ball_data)):
            if ball_data[i][0] != next_ball[0] or ball_data[i][1] != next_ball[1]:
                dx = float(ball_data[i][0]) - float(next_ball[0])   # NB x and y are ushort
                dy = float(ball_data[i][1]) - float(next_ball[1])   # float avoids error
                d2 = (dx * dx) + (dy * dy)
                if d2 < min_d2:
                    neighbour = ball_data[i]
                    min_d2    = d2

        # find best angle to avoid hitting neighbour
        dx = float(next_ball[0]) - float(neighbour[0])
        dy = float(next_ball[1]) - float(neighbour[1])
        if abs(dx) < 1.0:
            angle = - (math.pi / 2.0)             # avoid divide by zero
        else:
            angle = math.atan(dy / dx)            # angle in radians between -pi and pi
        angle = angle + (math.pi / 2.0)           # rotate pi / 2 radians
        if angle > math.pi / 2.0:                 # ensure angle between -pi and pi
            angle = angle - math.pi

        return - angle                            # return best angle to grip golf ball

    # if ball near the ball drop location
    def is_near_ball_tray(self, ball):
     #if d2 < max distance within the tray to its center -->need box dimensions
        #d2 = math.fabs(math.sqrt((self.ball_tray_centre[0] - ball[0])**2 + (self.ball_tray_centre[1] - ball[1])**2))
        d2=ball[1]
        #if d2 < 0.17: #change to max distance of box edge to center
        if d2 > self.ball_tray_centre[1]-0.20: #change to max distance of box edge to center
               return True

        return False
    # if ball off the table
    def is_off_table(self, ball):
     #if d2 < max distance within the tray to its center -->need box dimensions
        d2 = math.fabs(math.sqrt((self.ball_tray_centre[0] - ball[0])**2 + (self.ball_tray_centre[1] - ball[1])**2))
        d2=ball[1]
        d1=ball[0]
        if d2 < self.ball_tray_centre[1] + self.ymin: #change to max distance of box edge to center
            return True
        elif d1 > self.ball_tray_centre[0]+self.xmax:
            return True

        return False

    # Convert cv image to a numpy array
    def cv2array(self, im):
        depth2dtype = {cv.IPL_DEPTH_8U: 'uint8',
                       cv.IPL_DEPTH_8S: 'int8',
                       cv.IPL_DEPTH_16U: 'uint16',
                       cv.IPL_DEPTH_16S: 'int16',
                       cv.IPL_DEPTH_32S: 'int32',
                       cv.IPL_DEPTH_32F: 'float32',
                       cv.IPL_DEPTH_64F: 'float64'}
  
        arrdtype=im.depth
        a = numpy.fromstring(im.tostring(),
                             dtype = depth2dtype[im.depth],
                             count = im.width * im.height * im.nChannels)
        a.shape = (im.height, im.width, im.nChannels)

        return a

    def pixel_to_baxter(self, px, dist):
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist)                \
          + self.pose[0] + self.cam_x_offset
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist)                 \
          + self.pose[1] + self.cam_y_offset

        return (x, y)
    # Use Hough circles to find ball centres (Only works with round objects)
    def hough_it(self, n_ball, iteration):
        repeat=True
        no_valid_balls = True
        while no_valid_balls:
            while repeat == True:
                # move down to pick up ball
                # move down to pick up ballt == True:
                # create gray scale image of objects
                gray_image = cv.CreateImage((self.width, self.height), 8, 1)
                cv.CvtColor(cv.fromarray(self.cv_image), gray_image, cv.CV_BGR2GRAY)

                # create gray scale array of objects
                gray_array = self.cv2array(gray_image)

                # find Hough circles  --> set the max and min radius to only find spheres, set a different circle size to find blocks
                circles = cv2.HoughCircles(gray_array, cv.CV_HOUGH_GRADIENT, 1, 40, param1=50,  \
                          param2=self.hough_accumulator, minRadius=self.hough_min_radius,       \
                          maxRadius=self.hough_max_radius)

                # Check for at least one ball found
                if circles is None:
                    # display no balls found message on head display
                    self.splash_screen("searching", "for balls")

                    if self.no_circles == "left":
                        pose = (self.pose[0],         # rotate gripper to avoid hitting neighbour
                                    self.pose[1]-0.3,
                                    self.pose[2],
                                    self.pose[3],
                                    self.pose[4],
                                    self.pose[5])
                        self.baxter_ik_move(self.limb, pose)
                        self.no_circles = "frontleft"
                    elif self.no_circles == "frontleft":
                        pose = (self.pose[0]+0.15,         # rotate gripper to avoid hitting neighbour
                                self.pose[1],
                                self.pose[2],
                                self.pose[3],
                                self.pose[4],
                                self.pose[5])
                        self.baxter_ik_move(self.limb, pose)
                        self.no_circles = "frontright"
                    elif self.no_circles == "frontright":
                        pose = (self.pose[0],         # rotate gripper to avoid hitting neighbour
                                self.pose[1]+0.3,
                                self.pose[2],
                                self.pose[3],
                                self.pose[4],
                                self.pose[5])
                        self.baxter_ik_move(self.limb, pose)
                        self.no_circles = "back"
                    elif self.no_circles == "back":
                        pose = (self.pose[0]-0.15,         # rotate gripper to avoid hitting neighbour
                                self.pose[1],
                                self.pose[2],
                                self.pose[3],
                                self.pose[4],
                                self.pose[5])
                        self.baxter_ik_move(self.limb, pose)
                        self.no_circles = "left"
                else:
                    repeat = False
                    # no point in continuing so exit with error message
                    #sys.exit("ERROR - hough_it - No golf balls found")
            # Reset ball search variables
            repeat=True
            circles = numpy.uint16(numpy.around(circles))

            # initialuze ball_data array and number of balls
            ball_data = {}
            n_balls   = 0

            circle_array = numpy.asarray(cv.fromarray(self.cv_image))

            # check if ball is in ball tray --> if so, don't take it!
            for i in circles[0,:]:
                # convert to baxter coordinates
                ball = self.pixel_to_baxter((i[0], i[1]), self.tray_distance)

                if self.is_near_ball_tray(ball) or self.is_off_table(ball):
                    # draw the outer circle in red
                    cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 0, 255), 2)
                    # draw the center of the circle in red
                    cv2.circle(circle_array, (i[0], i[1]), 2, (0, 0, 255), 3)
                elif i[1] > 800:
                    # draw the outer circle in red
                    cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 0, 255), 2)
                    # draw the center of the circle in red
                    cv2.circle(circle_array, (i[0], i[1]), 2, (0, 0, 255), 3)
                else:
                    # draw the outer circle in green
                    cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # draw the center of the circle in green
                    cv2.circle(circle_array, (i[0], i[1]), 2, (0, 255, 0), 3)

                    ball_data[n_balls]  = (i[0], i[1], i[2])
                    n_balls            += 1

            circle_image = cv.fromarray(circle_array)

            cv.ShowImage("Hough Circle", circle_image)

            # 3ms wait
            cv.WaitKey(3)

            # display image on head monitor
            font     = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 1)
            position = (30, 60)
            s = "Searching for objects"
            cv.PutText(circle_image, s, position, font, self.white)
            circle_image = numpy.asarray(circle_image)    
            msg = cv_bridge.CvBridge().cv2_to_imgmsg(circle_image, encoding="bgr8")
            self.pub.publish(msg)

            if self.save_images:
                # save image of Hough circles on raw image
                file_name = self.image_dir                                                 \
                          + "hough_circle_" + str(n_ball) + "_" + str(iteration) + ".jpg"
                cv.SaveImage(file_name, cv.fromarray(circle_image))

            # Check for at least one ball found
            if n_balls == 0:                    # no balls found
                # display no balls found message on head display
                self.splash_screen("continuing", "ball search")
                # less than 12 balls found, no point in continuing, exit with error message
                #sys.exit("ERROR - hough_it - No balls found")
            else:
                no_valid_balls = False

        # resetting ball search 
        self.no_circles = "left"



        # select next ball and find it's position
        next_ball = self.find_next_object(ball_data, iteration)

        # find best gripper angle to avoid touching neighbouring ball --> may not be necessary test this code without it 
        angle = self.find_gripper_angle(next_ball, ball_data)

        # return next golf ball position and pickup angle
        return next_ball, angle
        
    # used to place camera over objects
    def object_iterate(self, n_ball, iteration, ball_data):
        # print iteration number
        print "OBJECT", n_ball, "ITERATION ", iteration

        # find displacement of ball from centre of image
        pixel_dx    = (self.width / 2) - ball_data[0]
        pixel_dy    = (self.height / 2) - ball_data[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.tray_distance)

        x_offset = - pixel_dy * self.cam_calib * self.tray_distance
        y_offset = - pixel_dx * self.cam_calib * self.tray_distance

        # update pose and find new ball data
        self.update_pose(x_offset, y_offset)
        ball_data, angle = self.hough_it(n_ball, iteration)

        # find displacement of ball from centre of image
        pixel_dx    = (self.width / 2) - ball_data[0]
        pixel_dy    = (self.height / 2) - ball_data[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.tray_distance)

        return ball_data, angle, error
    def get_ball_color(self):
        file_name = self.image_dir+ "ball.jpg"
        cv.SaveImage(file_name, cv.fromarray(self.cv_image))
        im=PIL_Image.open(file_name)
        crop_var=10
        ball_color='unknown'
        iterations=0
        while (ball_color!='blue') and (ball_color!='red') and iterations < 2:
            half_the_width = im.size[0] / 2
            half_the_height = im.size[1] / 2
            crop_var=5*crop_var
            im = im.crop(
                (
                    half_the_width - crop_var,
                    half_the_height - crop_var,
                    half_the_width + crop_var,
                    half_the_height + crop_var
                )
            )
            im.save(self.image_dir+"edited.jpg")
            colors=im.getcolors(30000)
            max_occurence, most_present = 0, 0
            try:
                for c in colors:
                        if c[0] > max_occurence:
                            (max_occurence, most_present) = c
            except TypeError:
                print("Too many colors in the image")
            if (most_present[0]>most_present[1]) and (most_present[0]>most_present[2]):
                print "Red"
                ball_color='red'
            elif (most_present[2]>most_present[0]) and (most_present[2]>most_present[1]):
                print "Blue"
                ball_color='blue'
            else:
                print "Ball Not Red or Blue"
                print most_present
                ball_color='unknown'
            crop_var-=1
            iterations +=1
        return ball_color

    # find all the golf balls and place them in the ball tray
    def pick_and_place(self):
        n_ball = 0
        self.ball_tray_centre = (self.ball_tray_x, self. ball_tray_y, self.ball_tray_z)
        self.green_tray_centre = (self.green_tray_x, self.green_tray_y, self.green_tray_z)

        while True and n_ball < 6:  # assume no more than 6 objects
            n_ball          += 1
            iteration        = 0
            angle            = 0.0

            # use Hough circles to find balls and select one ball
            next_ball, angle = self.hough_it(n_ball, iteration)

            error     = 2 * self.ball_tolerance

            print
            print "Ball number ", n_ball
            print "==============="

            # iterate to find next object
            # if hunting to and fro accept error in position
            while error > self.ball_tolerance and iteration < 10:
                iteration               += 1
                next_ball, angle, error  = self.object_iterate(n_ball, iteration, next_ball)

            font     = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 1)
            position = (30, 60)
            s        = "Picking up object"
            cv.PutText(cv.fromarray(self.cv_image), s, position, font, self.white)
            msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
            self.pub.publish(msg)

            #avoid disturbing other objects --> likely not necessary change to just grasp/drop ball in vertical orientation always... 
            if angle != self.yaw:             # if neighbouring ball
                 # rotate gripper to avoid hitting neighbour
                self.move_to_pose_with_offset(0,0,0,0,0,angle-self.pose[5])

                cv.PutText(cv.fromarray(self.cv_image), s, position, font, self.white)
                msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
                self.pub.publish(msg)

            # Move to just above the ball
            self.move_to_pose_with_offset(self.cam_x_offset,self.cam_y_offset,- self.distance +0.1,0,0,angle-self.pose[5])
            
            execute_pickup = False
            while execute_pickup == False:
                gui_offset_x=0
                gui_offset_y=0
                gui_offset_z=0
                gui_offset_roll=0
                gui_offset_pitch=0
                gui_offset_yaw=0
                # Here is Jane's GUI code
                self.splash_screen("Valid pickup", "position?")
                (gui_offset_x,gui_offset_y,gui_offset_z,gui_offset_roll,gui_offset_pitch,gui_offset_yaw,execute_pickup)=adjust(gui_offset_x,gui_offset_y,gui_offset_z,gui_offset_roll,gui_offset_pitch,gui_offset_yaw)
                print 'x offset is now %f, y offset is now %f, z offset is now %f' % (gui_offset_x,gui_offset_y,gui_offset_z)
                print 'roll offset is now %f, pitch offset is now %f, yaw offset is now %f' %(gui_offset_roll,gui_offset_pitch,gui_offset_yaw)

                self.move_to_pose_with_offset(gui_offset_x,gui_offset_y, -self.distance+0.1+gui_offset_z, gui_offset_roll,gui_offset_pitch,gui_offset_yaw+angle-self.pose[5])

            # Checking the color of the ball that is about to be picked up
            self.ball_colour = self.get_ball_color()
            color_correct = raw_input("If color is correct press 'enter'. otherwise enter 'n': ")
            if color_correct != "":
                self.ball_colour = "unknown"

            while self.ball_colour != "blue" and self.ball_colour !="red":
                print self.ball_colour
                self.ball_colour = raw_input("Ball color inconclusive. Enter ball color ('red'/'blue'): ")

            self.splash_screen("Ball color is", self.ball_colour)

            # slow down to reduce scattering of neighbouring golf balls --> likely not necessary
            self.limb_interface.set_joint_position_speed(0.1)

            # move down to pick up ball
            self.move_to_pose_with_offset(gui_offset_x,gui_offset_y, -self.distance+gui_offset_z, gui_offset_roll,gui_offset_pitch,gui_offset_yaw+angle-self.pose[5])
            self.print_arm_pose()

            # close the gripper
            self.gripper.close()

            self.splash_screen("Grasp", "successful?")
            successful_pickup = raw_input("Ball successfully picked up press enter. Else enter 'n': ")
            if successful_pickup == "":
                s = "Moving to ball tray"
                cv.PutText(cv.fromarray(self.cv_image), s, position, font, self.white)
                msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
                self.pub.publish(msg)

                # move straight up to avoid disturbing other objects
                self.move_to_pose_with_offset(gui_offset_x,gui_offset_y, -self.distance+0.15+gui_offset_z, gui_offset_roll,gui_offset_pitch,gui_offset_yaw+angle-self.pose[5])

                #speed up again 
                self.limb_interface.set_joint_position_speed(0.5)

                # display current image on head display
                cv.PutText(cv.fromarray(self.cv_image), s, position, font, self.white)
                msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
                self.pub.publish(msg)

                # move to the tray based on colour
                if self.ball_colour == "red":
                    pose = (self.ball_tray_centre[0]+self.cam_x_offset, 
                            self.ball_tray_centre[1]+self.cam_y_offset, 
                            self.pose[2] - 0.07, 
                            self.pose[3],
                            self.pose[4],
                            self.pose[5])
                elif self.ball_colour == "blue":
                    pose = (self.green_tray_centre[0]+self.cam_x_offset, 
                            self.green_tray_centre[1]+self.cam_y_offset, 
                            self.pose[2] - 0.07, 
                            self.pose[3],
                            self.pose[4],
                            self.pose[5])

                self.baxter_ik_move(self.limb, pose)

                # display currentfrom numpy import * image on head display
                s = "Placing ball in ball tray"
                cv.PutText(cv.fromarray(self.cv_image), s, position, font, self.white)
                msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
                self.pub.publish(msg)

                # open the gripper to drop ball
                self.gripper.open()
            else: 
                # open the gripper to drop ball
                self.gripper.open()
                n_ball -= 1

            # prepare to look for next ball
            #move to general ball location to find next right-most ball
            pose = (self.objects_x ,
                    self.objects_y ,
                    self.objects_z ,
                    -1.0 * math.pi,
                    0.0 * math.pi,
                    0.0 * math.pi)
            self.baxter_ik_move(self.limb, pose)

        # display all balls found on head display
        self.splash_screen("all balls", "found")

        print "All balls found"

    def move_to_pose_with_offset(self,dx,dy,dz,droll,dpitch,dyaw):
        pose = (self.pose[0]+dx,
                self.pose[1]+dy,
                self.pose[2]+dz,
                self.pose[3]+droll,
                self.pose[4]+dpitch,
                self.pose[5]+dyaw) 
        self.baxter_ik_move(self.limb, pose)

# read the setup parameters from setup.dat
def get_setup():
    global image_directory
    file_name = image_directory + "setup.dat"

    try:
        f = open(file_name, "r")
    except ValueError:
        sys.exit("ERROR: setup.py must be run before me495_project.py")

    # find limb
    s = string.split(f.readline())
    if len(s) >= 3:
        if s[2] == "left" or s[2] == "right":
            limb = s[2]
        else:
            sys.exit("ERROR: invalid limb in %s" % file_name)
    else:
        sys.exit("ERROR: missing limb in %s" % file_name)

    # find distance to table
    s = string.split(f.readline())
    if len(s) >= 3:
        try:
            distance = float(s[2])
        except ValueError:
            sys.exit("ERROR: invalid distance in %s" % file_name)
    else:
        sys.exit("ERROR: missing distance in %s" % file_name)

    return limb, distance

# Code for the GUI to adjust the gripper position before a pickup
def adjust(x,y,z,roll,pitch,yaw):
    state = False
    def set_vars(input_var,x,y,z,roll,pitch,yaw):
        if input_var[0]=='x':
            x+=float(raw_input("Enter amount x should be adjusted by: "))
            print 'x adjusted to be %f' % x
        elif input_var[0]=='y':
            y+=float(raw_input("Enter amount y should be adjusted by: "))
            print 'y adjusted to be %f' % y
        elif input_var[0]=='z':
            z+=float(raw_input("Enter amount z should be adjusted by: "))
            print 'z adjusted to be %f' % z
        elif input_var[0]=='r':
            roll+=float(raw_input("Enter amount roll should be adjusted by: "))
            print 'roll adjusted to be %f' % roll
        elif input_var[0]=='p':
            pitch+=float(raw_input("Enter amount pitch should be adjusted by: "))
            print 'pitch adjusted to be %f' % pitch
        elif input_var[0]=='a':
            yaw+=float(raw_input("Enter amount yaw should be adjusted by: "))
            print 'yaw adjusted to be %f' % yaw
        else:
            print 'Something is wrong'
        return (x,y,z,roll,pitch,yaw,False)

    def complete(x,y,z,roll,pitch,yaw):
        state=True
        #return (x,y,z,roll,pitch,yaw,False)
    adjust_yn = raw_input("Is this position ok press enter? Else press 'n': ")
    # display the start splash screen
    #self.splash_screen("Is this position ok?", "(y/n)")
    if adjust_yn=="":
        # display the start splash screen
        #self.splash_screen("Continue Automated", "Control")
        state = True
        return (x,y,z,roll,pitch,yaw,True)
    else:
        print 'Invalid input'
    while(state == False):
        # display the start splash screen
        #splashself.splash_screen("Manually Controlling", "Robot")
        bindings = {
            #key: (function, args, description)
            'x': (set_vars, ['x'], "Adjust x Position"),
            'y': (set_vars, ['y'], "Adjust y Position"),
            'z': (set_vars,['z'],"Adjust z position"),
            'R': (set_vars,['r'],"Adjust Roll"),
            'P': (set_vars,['p'],"Adjust Pitch"),
            'Y': (set_vars,['a'],"Adjust Yaw"),
            'q': (complete,[], "Quit"),
        }  
        print("key bindings: ")
        for key, val in sorted(bindings.items(),
            key=lambda x: x[1][2]):
            print("  %s: %s" % (key, val[2]))

        command_choice=raw_input("Which variable would you like to adjust? : ")
        if command_choice in bindings:
            if command_choice=='q':
                #complete(x,y,z,roll,pitch,yaw)
                return (x,y,z,roll,pitch,yaw,False)
                state =True
            else:
                (x,y,z,roll,pitch,yaw,temp)=bindings[command_choice][0](bindings[command_choice][1],x,y,z,roll,pitch,yaw)
        else:
            print("key bindings: ")
            for key, val in sorted(bindings.items(),
                key=lambda x: x[1][2]):
                print("  %s: %s" % (key, val[2]))   

def main():
    # get setup parameters
    #set up parameters set by running golf_setup.py first
    limb, distance = get_setup()
    print "limb     = ", limb
    print "distance = ", distance
    distance +=(.03)
    # create locate class instance
    locator = locate(limb, distance)

    raw_input("Press Enter to start: ")

    # open the gripper
    locator.gripper.open()

    locator.pose = (locator.objects_x,
                    locator.objects_y,
                    locator.objects_z,
                    locator.roll,
                    locator.pitch,
                    locator.yaw)
    #move close to the general object location
    locator.baxter_ik_move(locator.limb, locator.pose)
    # find all objects and place them in the ball tray
    locator.pick_and_place()

if __name__ == "__main__":
    main()



