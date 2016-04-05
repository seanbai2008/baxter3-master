#!/usr/bin/env python
# This is code to enable the robot and sent it to its home position


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
)

from baxter_core_msgs.msg import (
	CollisionAvoidanceState,
)

import topic_tools

def main():
	print("Going to Home Position")
	rospy.loginfo("Going to Home Position")
	rospy.init_node("going_home")

	_joint_moves = {
			'home': {
					'left':  [-0.08, -1.0,  -1.19, 1.94,  0.67, 1.03, -0.50],
					'right':  [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]
					},
				}

	_rs = baxter_interface.RobotEnable(CHECK_VERSION)
	_rs.enable()     
	baxter_interface.Limb('left').move_to_joint_positions(dict(zip(baxter_interface.Limb('left').joint_names(), _joint_moves['home']['left'])),5)  
	baxter_interface.Limb('right').move_to_joint_positions(dict(zip(baxter_interface.Limb('right').joint_names(), _joint_moves['home']['right'])),5)
	#baxter_interface.Limb('left').move_to_joint_positions(dict(zip(baxter_interface.Limb('left').joint_names(), _joint_moves['block1']['left'])))  

if __name__ == "__main__":
	main()