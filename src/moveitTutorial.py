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
 



  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("left_arm")
  group.set_goal_position_tolerance(0.01)
  group.set_goal_orientation_tolerance(0.01)
  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=10)

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
  
  
  print "============ Current Pose"
  pose_target2 = geometry_msgs.msg.Pose()
  print group.get_current_pose(group.get_end_effector_link())
  pose_target2 =group.get_current_pose(group.get_end_effector_link())
  #print pose_target2
  #pose_target2.position.x=1
  
  print "============"

  #set_position = rospy.ServiceProxy("/ExternalTools/left/PositionKinematicsNode/IKService", pose_target2)
  #print set_position

  #print pose_target2.pose.position
  #print pose_target2.pose.orientation

  pose_target2.pose.position.z=pose_target2.pose.position.z+0.1
  print "============ IK Solution"
  limb_joints = ik_solver.ik_solve('left', pose_target2.pose.position, pose_target2.pose.orientation)
  print "============"
  """
  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x = 0.69283
  pose_target.orientation.y = 0.1977
  pose_target.orientation.z = -0.16912
  pose_target.orientation.w = 0.67253
  pose_target.position.x = 0.81576
  pose_target.position.y = 0.093893
  pose_target.position.z = 0.2496
  print pose_target
  group.set_pose_target(pose_target2)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan1 = group.plan()

  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(5)

 
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
  ## group.plan() method does this automatically so this is not that useful
  ## here (it just displays the same trajectory again).
  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);

  print "============ Waiting while plan1 is visualized (again)..."
  rospy.sleep(5)
  
  """
  
  ## Moving to a pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Moving to a pose goal is similar to the step above
  ## except we now use the go() function. Note that
  ## the pose goal we had set earlier is still active 
  ## and so the robot will try to move to that goal. We will
  ## not use that function in this tutorial since it is 
  ## a blocking function and requires a controller to be active
  ## and report success on execution of a trajectory.

  # Uncomment below line when working with a real robot
  # group.go(wait=True)

  ## Planning to a joint-space goal 
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Let's set a joint space goal and move towards it. 
  ## First, we will clear the pose target we had just set.

  group.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values

  ## Now, let's modify one of the joints, plan to the new joint
  ## space goal and visualize the plan
  #group_variable_values[0] = 1


  #group.set_joint_value_target(group_variable_values)
  group.set_joint_value_target(limb_joints)
  #plan2 = group.plan()

  print "============ Waiting while RVIZ displays plan2..."
  rospy.sleep(5)
  #group.go(wait=True)

  """
  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints 
  ## for the end-effector to go through.
  waypoints = []

  # start with the current pose
  waypoints.append(group.get_current_pose().pose)

  # first orient gripper and move forward (+x)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.w = 1.0
  wpose.position.x = waypoints[0].position.x + 0.1
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))

  # second move down
  wpose.position.z -= 0.10
  waypoints.append(copy.deepcopy(wpose))

  # third move to the side
  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))

  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
                               
  print "============ Waiting while RVIZ displays plan3..."
  rospy.sleep(5)
  """
 
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
