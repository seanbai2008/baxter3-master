#!/usr/bin/env python

"""
Testing robot movements/code with for the simulator.  
"""
import argparse

from copy import deepcopy

import rospy

from std_msgs.msg import (
	Empty,
	Bool,
)

import baxter_interface

from baxter_core_msgs.msg import (
	CollisionAvoidanceState,
)
from baxter_interface import CHECK_VERSION


def main():
	print("Hello Baxter World")
	rospy.loginfo("Hello Baxter World")
	rospy.init_node("zack_testing")

	_joint_moves = {
    		'tuck': {
					'left':  [-1.0, -2.07,  3.0, 2.55,  0.0, 425,  0.01,  0.0],
					'right':  [1.0, -2.07, -3.0, 2.55, -0.0, 0.01,  0.0]
					},
			'untuck': {
					'left':  [-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50],
					'right':  [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]
					}
				}
	#print(_joint_moves)
	#print(_joint_moves['untuck']['right']) 
	#print(dict(zip(baxter_interface.Limb('right').joint_names(), _joint_moves['untuck']['right'])))

	_rs = baxter_interface.RobotEnable(CHECK_VERSION)
	_rs.enable()       
	baxter_interface.Limb('right').move_to_joint_positions(dict(zip(baxter_interface.Limb('right').joint_names(), _joint_moves['untuck']['right'])))
	baxter_interface.Limb('left').move_to_joint_positions(dict(zip(baxter_interface.Limb('left').joint_names(), _joint_moves['untuck']['left'])))
if __name__ == "__main__":
	main()