#!/usr/bin/env python

import argparse
import os
import rospy
import baxter_interface
import baxter_external_devices
from std_msgs.msg import String
from baxter_interface import CHECK_VERSION

def msg_publisher(msg): #publishes messages so that position controller knows what to do
    if not rospy.is_shutdown(): 
       pub = rospy.Publisher("position_correction", String, queue_size=10)
       rate = rospy.Rate(10) # 10hz
       msg_str = msg 
       rospy.loginfo(msg_str)
       pub.publish(msg_str)
       rate.sleep()

def adjust_yn():
    def yes():
	done = True
        rospy.signal_shutdown("Shutting Down")
	
    def no():
	main_menu()

    bindings = {
    #   key: (function, args, description)
        'y': (yes,[], "Let Robot Continue"),
        'n': (no, [], "Enter Control Mode"),
     }
    done = False
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Shutting Down.")
            elif c in bindings:
                cmd = bindings[c]
                cmd[0](*cmd[1])
            else:
                print("key bindings: ")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

def main_menu():
    
    def complete():
	done = True
        rospy.signal_shutdown("Shutting Down")
    def rotate():
        rotation_keyboard()
    def xpos():
	xpos = raw_input("Enter x adjustment (positive or negative): ")
        xpos ='x: '+xpos
        msg_publisher(xpos)
    def ypos():
	ypos = raw_input("Enter y adjustment (positive or negative): ")
        ypos ='y: '+ypos
        msg_publisher(ypos)
    def zpos():
	zpos = raw_input("Enter z adjustment (positive or negative): ")
        zpos ='z: '+zpos
        msg_publisher(zpos)

    bindings = {
    #   key: (function, args, description)
        'd': (complete,[], "Quit"),
        'r': (rotate,[], "Adjust Rotational Orientation"),
        'x': (xpos,[],"Input X Position Adjustment"),
        'y': (ypos,[],"Input Y Position Adjustment"),
        'z': (zpos,[],"Input Z Position Adjustment"),
     }

    print("Main Menu for Control Mode")
    print("key bindings: ")
    for key, val in sorted(bindings.items(),
        key=lambda x: x[1][2]):
        print("  %s: %s" % (key, val[2]))
    print("  ?: Help")
    done = False
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Shutting Down.")
            elif c in bindings:
                cmd = bindings[c]
                cmd[0](*cmd[1])
            else:
                print("key bindings: ")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

def rotation_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    def complete():
	done = True
        rospy.signal_shutdown("Shutting Down")

    bindings = {
    #   key: (function, args, description)
        'k': (set_j, [left, lj[6], 0.1], "left wrist increase"),
        'l': (set_j, [left, lj[6], -0.1], "left wrist decrease"),
        
        'a': (set_j, [right, rj[6], 0.1], "right wrist increase"),
        's': (set_j, [right, rj[6], -0.1], "right wrist decrease"),

	'd': (complete,[], "Quit"),
        'm': (main_menu,[], "Return To Main Menu"),
     }

    done = False
    print("Controlling Rotational Orientation")
    print("key bindings: ")
    print("  ?: Help")
    for key, val in sorted(bindings.items(),
        key=lambda x: x[1][2]):
        print("  %s: %s" % (key, val[2]))
    #os.system('rosrun baxter_examples xdisplay_image.py --file ~/Pictures/key_bindings.jpg')
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Program finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "set_j(right, 's0', 0.1)"
                cmd[0](*cmd[1])
                print("command: %s" % (
[2],))
            else:
                print("key bindings: ")
                print("  d: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

def main():
    

    rospy.init_node("rotational_correction")
    #print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()
    #os.system('rosrun baxter_examples xdisplay_image.py --file ~/Pictures/orientation_ok.jpg')
    print("Is this location ok? Enter y to let me continue, n to enter control mode.")
    def clean_shutdown():
        print("\nExiting program...")
        #if not init_state:
        #    print("Disabling robot...")
        #    rs.disable()
    rospy.on_shutdown(clean_shutdown)
    adjust_yn()
    print("Done.")


if __name__ == '__main__':
    main()
