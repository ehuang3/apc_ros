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

import numpy as np
import rospy
import openravepy
import trajoptpy
import pdb
from IPython.core.debugger import Tracer
from .action import *


def init_trajopt(interactive):
    """Initialize trajectory optimization"""
    # Pause every iteration, until you press 'p'. Press escape to disable further plotting.
    trajoptpy.SetInteractive(interactive)


def build_trajopt_request(srv_request, env):
    # Get the robot.
    robot = env.GetRobot('crichton')
    # Construct goal state.
    joint_target = np.zeros(robot.GetDOF())
    with env:
        robot = env.GetRobot('crichton')
        # Get goal joint names and joint angles.
        group_joint_names = srv_request.action.joint_trajectory.joint_names
        goal_joint_angles = srv_request.action.joint_trajectory.points[-1].positions
        if robot == None:
            print 'No robot found for "crichton"'
        else:
            # For each joint..
            for joint in robot.GetJoints():
                # Set the joint to the starting value.
                joint_target[joint.GetDOFIndex()] = joint.GetValues()[0]
                # If the joint is in the goal set, set it to the goal
                # value instead.
                for i in range(len(group_joint_names)):
                    if joint.GetName() == group_joint_names[i]:
                        # if debug:
                        #     print "Setting joint target", joint.GetName(), "to", goal.name[i], ":", goal.position[i]
                        joint_target[joint.GetDOFIndex()] = goal_joint_angles[i]
    joint_target = joint_target.tolist()

    # Compute the required distance penalty. For pre-grasp trajectories, the
    # distance penalty should be small and positive, to encourage the fingers to
    # avoid the object until the grasp.
    distance_penalty = 0.0
    if is_action_transit(srv_request.action):
        distance_penalty = 0.05 # 1 cm
    if is_action_pregrasp(srv_request.action):
        distance_penalty = 0.02 # 1 cm

    disabled_dofs = compute_disabled_dof_indexes(srv_request.action, env)

    # Fill out trajopt request.
    trajopt_request = {
        "basic_info" : {
            "n_steps" : 20,
            "manip" : "active", # see below for valid values
            "start_fixed" : True, # i.e., DOF values at first timestep are fixed based on current robot state
            "dofs_fixed" : disabled_dofs
        },
        "costs" : [
            {
                "type" : "joint_vel", # joint-space velocity cost
                "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
                # also valid: [1.9, 2, 3, 4, 5, 5, 4, 3, 2, 1]
            },
            {
                "type" : "collision",
                "params" : {
                    "continuous" : True,
                    # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
                    "coeffs" : [1],
                    # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
                    "dist_pen" : [distance_penalty]
                },
            },
            {
                "name": "disc_coll",
                "type" : "collision",
                "params" :
                {
                    "continuous":False,
                    "coeffs" : [1],
                    "dist_pen" : [distance_penalty]
                }
            }
        ],
        "constraints" : [
            {
                "type" : "joint", # joint-space target
                "params" : {"vals" : joint_target } # length of vals = # dofs of manip
            }
        ],
        "init_info" : {
            "type" : "straight_line", # straight line in joint space.
            "endpoint" : joint_target
        }
    }
    return trajopt_request
