#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Zack Woodruff

# ==========================================================
## Import Packages
# ==========================================================
import argparse
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

from copy import deepcopy

from std_msgs.msg import (
	Empty,
	Bool,
	String
)

from baxter_core_msgs.msg import (
	CollisionAvoidanceState,
)

import topic_tools

# Moveit Packages
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import ik_solver
from baxter_core_msgs.srv import (
  SolvePositionIK,
  SolvePositionIKRequest,
)

from geometry_msgs.msg import (    
    PoseStamped,
    Point,
    Quaternion,
)



# ==========================================================
## Initalize variables
# ==========================================================
# Set table height/position
# Set bin height/positions

# ==========================================================
## Go to neutral position
# ==========================================================

# ==========================================================
## Find objects
# ==========================================================
# output is # of objects, COM locations, object type. 

# ==========================================================
## Move objects (Can add dual arm if necessary) 
# ==========================================================
# While nobjects > 0
# Move above first object
# Ask if orientation is correct
# Lower to table
# Close Gripper
# Raise from table above current position
# Move above box
# Open Gripper
# Repeat until all objects are gone. 

class moveRobot(object):
	def move_basic(self,joint_position,arm,time=5):
		baxter_interface.Limb(arm).move_to_joint_positions(dict(zip(baxter_interface.Limb(arm).joint_names(), joint_position)),time)
	def gripper(self,gripperstate):
		# gripper state
		g=10
	def initialize_moveit(self):
		print "============ Initializing Moveit"
		moveit_commander.roscpp_initialize(sys.argv)
		#rospy.init_node('baxter3_moveit',anonymous=True)

		## Instantiate a RobotCommander object. This object is an interface to
		## the robot as a whole.
		self.robot = moveit_commander.RobotCommander()

		## Instantiate a PlanningSceneInterface object. This object is an interface
		## to the world surrounding the robot.
		self.scene = moveit_commander.PlanningSceneInterface()

		print "============ Waiting for RVIZ..."
		rospy.sleep(3)
		print "============ Starting tutorial "

		print "============ Adding collision objects..."
		# Add in objects
		# Table
		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = 0.8
		p.pose.position.y = 0.025
		p.pose.position.z = -0.6
		p.pose.orientation.x = 1
		self.scene.add_box("table", p, (0.762, 1.25, 1))

		# Box 1
		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = 0.6
		p.pose.position.y = 0.025
		p.pose.position.z = -0.22
		p.pose.orientation.x = 1
		self.scene.add_box("box1", p, (0.25, 0.2, 0.08))

		# Box 2
		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = 1
		p.pose.position.y = 0.025
		p.pose.position.z = -0.22
		p.pose.orientation.x = 1
		self.scene.add_box("box2", p, (0.25, 0.2, 0.08))
		print "============"

		self.group_left = moveit_commander.MoveGroupCommander("left_arm")
		self.group_left.set_goal_position_tolerance(0.01)
		self.group_left.set_goal_orientation_tolerance(0.01)
		self.group_right = moveit_commander.MoveGroupCommander("right_arm")
		self.group_right.set_goal_position_tolerance(0.01)
		self.group_right.set_goal_orientation_tolerance(0.01)

		## We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=10)
	def goto_joint_angle(self,limb_joints,limb):
		#print "============ IK Solution"
		#limb_joints = ik_solver.ik_solve('left', pose_target2.pose.position, pose_target2.pose.orientation)
		#print "============"
		## Then, we will get the current set of joint values for the group
		if limb =="left":
			#group_left_variable_values = group_left.get_current_joint_values()
			self.group_left.set_joint_value_target(limb_joints)
			plan = self.group_left.plan()
			print "============ Waiting while RVIZ displays plan2..."
			rospy.sleep(5)
			self.group_left.go(wait=True)
		elif limb == "right":
			#group_right_variable_values = group_right.get_current_joint_values()
			self.group_right.set_joint_value_target(limb_joints)
			plan = self.group_right.plan()
			print "============ Waiting while RVIZ displays plan2..."
			rospy.sleep(5)
			self.group_right.go(wait=True)
	def ik_solution(self, limb,limb_pose):
		limb_joints = ik_solver.ik_solve(limb, limb_pose.position, limb_pose.orientation)
		return limb_joints


def main():
	print("Starting Pick/Place Program")
	rospy.loginfo("Starting Pick/Place Program")
	rospy.init_node("main_controller")
	moveBaxter = moveRobot()
	_joint_moves = {
			'home': {
					'left':  [-0.08, -1.0,  -1.19, 1.94,  0.67, 1.03, -0.50],
					'right':  [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]
					},
			'block1': {
					'left':  [-0.336464083021216, -0.17044986297854514, -0.2998723260845084, 1.3693390268787056, 0.7280307034547473, 0.4279198306172507, -0.24374133515873098],
					'right':  [-0.20956444408910535, -0.5667841267281482, 0.9732504646529234, 1.0150376059755404, -0.7736364244393252, 1.3983346251413824, -0.4840842699792196]
					},
			'block2': {
					'left':  [-0.4739501413588769, 0.22003615319960712, -0.5161764408950891, 0.34893298110920057, 0.5866901919158787, 1.0007251416279432, 0.14320832411513873],
					'right':  [-0.20956444408910535, -0.5667841267281482, 0.9732504646529234, 1.0150376059755404, -0.7736364244393252, 1.3983346251413824, -0.4840842699792196]
					},
			'block3': {
					'left':  [0.004228942363553756, -0.07812510266315353, -0.443038371168873, 1.1816080252766161, 0.8048252751368974, 0.5853858748058691, 0.08385952812848974],
					'right':  [-0.20956444408910535, -0.5667841267281482, 0.9732504646529234, 1.0150376059755404, -0.7736364244393252, 1.3983346251413824, -0.4840842699792196]
					},
			'blockdrop': {
					'left':  [-0.9591075808891762, -0.38421351866534703, -0.33450112215263594, 1.5457699990714389, 0.7456969046121538, 0.4966963066825958, -0.9424691319362308],
					'right':  [-0.20956444408910535, -0.5667841267281482, 0.9732504646529234, 1.0150376059755404, -0.7736364244393252, 1.3983346251413824, -0.4840842699792196]
					}			
				}
	leftdrop = PoseStamped()
	leftdrop.pose.position.x=0.536883540379
	leftdrop.pose.position.y=0.52362167282
	leftdrop.pose.position.z=0.060232400958
	leftdrop.pose.orientation.x=0.142611342175
	leftdrop.pose.orientation.y=-0.989446580916
	leftdrop.pose.orientation.z=0.00637607389793
	leftdrop.pose.orientation.w=0.0248357459738

	leftpickup = PoseStamped()
	leftpickup.pose.position.x=0.586414828291
	leftpickup.pose.position.y=0.239222280497
	leftpickup.pose.position.z=-0.0740549093732
	leftpickup.pose.orientation.x=0.066366712664
	leftpickup.pose.orientation.y=-0.99773946
	leftpickup.pose.orientation.z=0.00116046849984
	leftpickup.pose.orientation.w=0.0104920313785

	_rs = baxter_interface.RobotEnable(CHECK_VERSION)
	_rs.enable()

	#controller = "simple"
	controller = "moveit"
	if controller == "simple":
		# Go to Home
		moveBaxter.move_basic(_joint_moves['home']['left'],'left',5)
		moveBaxter.move_basic(_joint_moves['home']['right'],'right',5)  

		# Go to first block position
		moveBaxter.move_basic(_joint_moves['block1']['left'],'left',5)
		moveBaxter.move_basic(_joint_moves['block1']['right'],'right',5)

		# Check position with GUI
		# Lower gripper
		# Close Gripper
		# Check if grip successful with GUI

		# Go to block drop
		moveBaxter.move_basic(_joint_moves['blockdrop']['left'],'left',5)
		moveBaxter.move_basic(_joint_moves['blockdrop']['right'],'right',5)

		# Open Gripper

		# Go to second block position
		moveBaxter.move_basic(_joint_moves['block2']['left'],'left',5)
		moveBaxter.move_basic(_joint_moves['block2']['right'],'right',5)

		# Check position with GUI
		# Lower gripper
		# Close Gripper
		# Check if grip successful with GUI

		# Go to block drop
		moveBaxter.move_basic(_joint_moves['blockdrop']['left'],'left',5)
		moveBaxter.move_basic(_joint_moves['blockdrop']['right'],'right',5)

		# Open Gripper

		# Go to third block position
		moveBaxter.move_basic(_joint_moves['block3']['left'],'left',5)
		moveBaxter.move_basic(_joint_moves['block3']['right'],'right',5)

		# Check position with GUI
		# Lower gripper
		# Close Gripper
		# Check if grip successful with GUI

		# Go to block drop
		moveBaxter.move_basic(_joint_moves['blockdrop']['left'],'left',5)
		moveBaxter.move_basic(_joint_moves['blockdrop']['right'],'right',5)

		# Open Gripper

		# Return to Home
		moveBaxter.move_basic(_joint_moves['home']['left'],'left',5)
		moveBaxter.move_basic(_joint_moves['home']['right'],'right',5)  

	elif controller == "moveit":

		moveBaxter.initialize_moveit()

		# Go to Home
		moveBaxter.goto_joint_angle(_joint_moves['home']['left'],'left')
		moveBaxter.goto_joint_angle(_joint_moves['home']['right'],'right')

		# Go to first block position
		#moveBaxter.goto_joint_angle(_joint_moves['block1']['left'],'left')
		#moveBaxter.goto_joint_angle(_joint_moves['block1']['right'],'right')

		# Check position with GUI
		# Lower gripper
		# Close Gripper
		# Check if grip successful with GUI

		iksol=moveBaxter.ik_solution('left',leftpickup.pose)
		moveBaxter.goto_joint_angle(iksol,'left')

		# Go to block drop
		iksol=moveBaxter.ik_solution('left',leftdrop.pose)
		moveBaxter.goto_joint_angle(iksol,'left')
		#moveBaxter.goto_joint_angle(_joint_moves['blockdrop']['left'],'left')
		moveBaxter.goto_joint_angle(_joint_moves['blockdrop']['right'],'right')
		#moveBaxter.goto_joint_angle(_joint_moves['block2']['right'],'right')
		#moveBaxter.goto_joint_angle(_joint_moves['block2']['right'],'right')

		# Open Gripper

		# Go to second block position
		#moveBaxter.goto_joint_angle(_joint_moves['block2']['left'],'left')
		#moveBaxter.goto_joint_angle(_joint_moves['block2']['right'],'right')

		# Check position with GUI
		# Lower gripper
		# Close Gripper
		# Check if grip successful with GUI

		# Go to block drop
		#moveBaxter.goto_joint_angle(_joint_moves['blockdrop']['left'],'left')
		#moveBaxter.goto_joint_angle(_joint_moves['blockdrop']['right'],'right')

		# Open Gripper

		# Go to third block position
		#moveBaxter.goto_joint_angle(_joint_moves['block3']['left'],'left')
		#moveBaxter.goto_joint_angle(_joint_moves['block3']['right'],'right')

		# Check position with GUI
		# Lower gripper
		# Close Gripper
		# Check if grip successful with GUI

		# Go to block drop
		#moveBaxter.goto_joint_angle(_joint_moves['blockdrop']['left'],'left')
		#moveBaxter.goto_joint_angle(_joint_moves['blockdrop']['right'],'right')

		# Open Gripper

		# Return to Home
		moveBaxter.goto_joint_angle(_joint_moves['home']['left'],'left')
		moveBaxter.goto_joint_angle(_joint_moves['home']['right'],'right')  

if __name__ == "__main__":
	main()