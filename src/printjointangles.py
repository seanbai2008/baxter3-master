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
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
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

from std_msgs.msg import String

## END_SUB_TUTORIAL

def ik_test(limb):
  rospy.init_node("rsdk_ik_service_client")
  ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
  iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
  ikreq = SolvePositionIKRequest()
  hdr = Header(stamp=rospy.Time.now(), frame_id='base')


def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('baxter3_moveit',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  print "============ Waiting for RVIZ..."
  rospy.sleep(3)
  print "============ Starting tutorial "

  print "============ Adding collision objects..."
  # Add in objects

  # Table
  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = 0.8
  p.pose.position.y = 0.025
  p.pose.position.z = -0.6
  p.pose.orientation.x = 1
  scene.add_box("table", p, (0.75, 1.25, 0.68))

  # Box 1
  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = 0.6
  p.pose.position.y = 0.025
  p.pose.position.z = -0.22
  p.pose.orientation.x = 1
  scene.add_box("box1", p, (0.25, 0.2, 0.08))

  # Box 2
  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = 1
  p.pose.position.y = 0.025
  p.pose.position.z = -0.22
  p.pose.orientation.x = 1
  scene.add_box("box2", p, (0.25, 0.2, 0.08))
  print "============"



  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("left_arm")
  #group = moveit_commander.MoveGroupCommander("left_arm")
  group.set_goal_position_tolerance(0.01)
  group.set_goal_orientation_tolerance(0.01)
  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.


  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  #print "============ Printing variable values"
  #print robot.get_current_variable_values()
  #print "============"
  
  #print "============ Current Pose"
  #pose_target2 = geometry_msgs.msg.Pose()
  print group.get_current_pose(group.get_end_effector_link())
  #pose_target2 =group.get_current_pose(group.get_end_effector_link())
  #print pose_target2
  #pose_target2.position.x=1
  
  print "============"

  #set_position = rospy.ServiceProxy("/ExternalTools/left/PositionKinematicsNode/IKService", pose_target2)
  #print set_position

  #print pose_target2.pose.position
  #print pose_target2.pose.orientation

  #pose_target2.pose.position.z=pose_target2.pose.position.z
  #print "============ IK Solution"
  #limb_joints = ik_solver.ik_solve('left', pose_target2.pose.position, pose_target2.pose.orientation)
  #print "============"

  #group.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values

 
 
  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## First, we will define the collision object message
  collision_object = moveit_msgs.msg.CollisionObject()



  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

