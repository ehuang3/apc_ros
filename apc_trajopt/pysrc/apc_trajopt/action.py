#!/usr/bin/env python
#
# Copyright (c) 2015, Georgia Tech Research Corporation
# All rights reserved.
#
# Author(s): Eric Huang <ehuang@gatech.edu>
# Georgia Tech Humanoid Robotics Lab
# Under Direction of Prof. Andrea Thomaz <athomaz@cc.gatech.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#     * Redistributions of source code must retain the above
#       copyright notice, this list of conditions and the following
#       disclaimer.
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials
#       provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
import rospy
import numpy
import pdb
from apc_msgs.msg import PrimitiveAction
import trajectory_msgs
import trajoptpy.math_utils as mu
from .apc_assert import ApcError, apc_assert
from IPython.core.debugger import Tracer


def __action_type__(action):
    # Is the robot moving?
    robot_moving = not (action.joint_trajectory.points[0].positions == action.joint_trajectory.points[-1].positions)
    # Is the robot in a grasp?
    grasping = action.grasp
    # Is the object moving? This is all relative to the starting
    # object frame.
    object_moving = False
    if action.object_id:
        object_pose_exists = len(action.object_trajectory.poses) > 0
        object_pose_moving = object_pose_exists and action.object_trajectory.poses[0] != action.object_trajectory.poses[0]
        object_moving = object_pose_moving or (robot_moving and object_pose_exists)
        apc_assert(action.object_id and action.object_key,
                   ("Missing object key to match object id %s") % (action.object_id))
    # Compute the action type.
    transit    = bool(not action.object_id)
    pregrasp   = bool(robot_moving and not grasping and not object_moving and action.object_id)
    grasp      = bool(not robot_moving and grasping and not object_moving and action.object_id)
    postgrasp  = bool(robot_moving and grasping and action.object_id)
    nonprehensile = bool(robot_moving and not grasping and object_moving and action.object_id)
    apc_assert(transit ^ pregrasp ^ grasp ^ postgrasp ^ nonprehensile,
               ("Failed to get a single action type:\n"
                "   transit: %d\n"
                "  pregrasp: %d\n"
                "     grasp: %d\n"
                " postgrasp: %d\n"
                "prehensile: %d\n"
                "    action: %s\n") % (transit, pregrasp, grasp, postgrasp, nonprehensile, action))
    if transit:
        return 'transit'
    if pregrasp:
        return 'pregrasp'
    if grasp:
        return 'grasp'
    if postgrasp:
        return 'postgrasp'
    if nonprehensile:
        return 'nonprehensile'

def is_action_transit(action):
    return __action_type__(action) == 'transit'

def is_action_pregrasp(action):
    return __action_type__(action) == 'pregrasp'

def is_action_grasp(action):
    return __action_type__(action) == 'grasp'

def is_action_postgrasp(action):
    return __action_type__(action) == 'postgrasp'

def is_action_nonprehensile(action):
    return __action_type__(action) == 'nonprehensile'

def is_action_grasping(action):
    return __action_type__(action) == 'grasp' or __action_type__(action) == 'postgrasp'

def is_action_stationary(action):
    start = action.joint_trajectory.points[0].positions
    end = action.joint_trajectory.points[-1].positions
    return numpy.allclose(start, end)

def is_action_linear(action):
    return (action.interpolate_cartesian or action.group_id == 'crichton_left_hand' or
            action.group_id == 'crichton_right_hand')

def print_action_summary(action):
    # action = PrimitiveAction()
    print "action name   :", action.action_name
    print "action group  :", action.group_id
    print "action frame  :", action.frame_id
    print "action object :", action.object_id
    print "action objkey :", action.object_key
    print "action type   :", __action_type__(action)

def compute_disabled_dof_indexes(action, env):
    robot = env.GetRobot('crichton')
    disabled = []
    for joint in robot.GetJoints():
        if joint.GetName() in action.joint_trajectory.joint_names:
            continue
        else:
            disabled.append(joint.GetDOFIndex())
    return disabled

def check_for_nonactive_joint_motion(action, problem, result, env):
    # start = action.joint_trajectory.points[0].positions
    # end = action.joint_trajectory.points[-1].positions
    # return numpy.allclose(start, end)
    return False

def fill_response_action(action, problem, trajectory, env):
    robot = env.GetRobot('crichton')
    disabled = compute_disabled_dof_indexes(action, env)
    M = [0 for name in action.joint_trajectory.joint_names]
    for joint in robot.GetJoints():
        if joint.GetDOFIndex() in disabled:
            continue
        M[action.joint_trajectory.joint_names.index(joint.GetName())] = joint.GetDOFIndex()
    action.joint_trajectory.points = []
    for p in trajectory:
        q = trajectory_msgs.msg.JointTrajectoryPoint()
        for i in range(len(M)):
            q.positions.append(p[M[i]])
        action.joint_trajectory.points.append(q)

def compute_linear_trajectory(action, problem, env):
    robot = env.GetRobot('crichton')
    joint_names = action.joint_trajectory.joint_names
    p_start = action.joint_trajectory.points[0].positions
    p_end = action.joint_trajectory.points[-1].positions
    T = numpy.zeros((2,29))
    T[0,:] = robot.GetDOFValues()
    T[1,:] = robot.GetDOFValues()
    for joint in robot.GetJoints():
        for i in range(len(joint_names)):
            if joint.GetName() == joint_names[i]:
                T[1,joint.GetDOFIndex()] = p_end[i]
    T = mu.interp2d(numpy.linspace(0,1,20), numpy.linspace(0,1,len(T)), T)
    return T

if __name__=='__main__':
    action = apc_msgs.msg.PrimitiveAction()
    action.joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    action.joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    # Test transit
    action.joint_trajectory.points[ 0].positions.append(0)
    action.joint_trajectory.points[-1].positions.append(0)
    apc_assert(is_action_transit(action),
               "Failed action is transit")
    # Test pregrasp
    action.object_id = "test"
    action.grasp = False
    action.joint_trajectory.points[ 0].positions[ 0] = 1
    action.joint_trajectory.points[-1].positions[-1] = 2
    # action.object_trajectory.poses.append(geometry_msgs.msg.Pose())
    # action.object_trajectory.poses.append(geometry_msgs.msg.Pose())
    apc_assert(is_action_pregrasp(action),
               "Failed action is pregrasp")
    # Test grasp
    action.object_id = "test"
    action.grasp = True
    action.joint_trajectory.points[ 0].positions[ 0] = 1
    action.joint_trajectory.points[-1].positions[-1] = 1
    action.object_trajectory.poses.append(geometry_msgs.msg.Pose())
    action.object_trajectory.poses.append(geometry_msgs.msg.Pose())
    apc_assert(is_action_grasp(action),
               "Failed action is grasp")
    # Test postgrasp
    action.object_id = "test"
    action.grasp = True
    action.joint_trajectory.points[ 0].positions[ 0] = 1
    action.joint_trajectory.points[-1].positions[-1] = 2
    apc_assert(is_action_postgrasp(action),
               "Failed action is postgrasp")
    # Test prehensile
    action.object_id = "test"
    action.grasp = False
    action.joint_trajectory.points[ 0].positions[ 0] = 1
    action.joint_trajectory.points[-1].positions[-1] = 2
    action.object_trajectory.poses[ 0].position.x = 1
    action.object_trajectory.poses[-1].position.x = 2
    apc_assert(is_action_nonprehensile(action),
               "Failed action is nonprehensile")

