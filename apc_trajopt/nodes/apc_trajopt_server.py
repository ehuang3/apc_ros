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

import os
import json
import time
import cProfile
import roslib
import rospy
import rospkg
import numpy as np
import openravepy
import trajoptpy
roslib.load_manifest('apc_trajopt')
import apc_msgs.msg
import apc_msgs.srv
import trajectory_msgs
import tf.transformations
from trajoptpy.check_traj import traj_is_safe
import trajoptpy.math_utils as mu
from IPython.core.debugger import Tracer
import argparse

from apc_trajopt import *


env = None
debug = False
no_optimization = False

def motion_planning_service(request):
    """
    Perform motion planning using trajectory optimization.

    """
    # Use the global copy of openrave environment.
    global env

    global debug
    global no_optimization

    print "--------------------             START             --------------------"

    # If the grasp pose is in collision with the shelf, 
    if request.check_for_impossible_grasp:
        response = apc_msgs.srv.ComputeDenseMotionResponse()
        response.valid = not check_for_impossible_grasp(request.action, env)
        print "impos. grasp :", response.valid
        response.action = request.action
        print "--------------------              END              --------------------"
        return response

    if debug:
        print_action_summary(request.action)

    # Load and set objects to correct location.
    load_and_set_items_to_openrave(request, env)

    # If the last action state is in collision, do not even bother
    # planning.
    # if check_for_end_collision(request.action, request.bin_states, env):
    #     response = apc_msgs.srv.ComputeDenseMotionResponse()
    #     response.valid = False
    #     response.action = request.action
    #     print "--------------------              END              --------------------"
    #     return response

    # Set robot to correct joint angles and positions.
    set_robot_state_to_openrave(request, env)

    # Create trajopt json problem request.
    trajopt_request = build_trajopt_request(request, env)

    # Convert dictionary into json-formatted string.
    trajopt_string = json.dumps(trajopt_request)

    # Create object that stores optimization problem.
    trajopt_problem = trajoptpy.ConstructProblem(trajopt_string, env)

    # Reload objects and positions in case an object is "ungrabbed".
    load_and_set_items_to_openrave(request, env)

    # Do optimization.
    trajopt_result = None
    trajectory = None
    do_trajopt = (not no_optimization and not is_action_stationary(request.action)
                  and not is_action_linear(request.action))

    # Set collision matrix information and grabbed bodies.
    set_target_item_collision_properties(request.action, trajopt_problem, env)

    # Print collision pairs if debugging.
    if debug and not is_action_transit(request.action):
        print_item_collision_pairs(env.GetKinBody(request.action.object_key), trajopt_problem, env)

    # Start a timer.
    t_start = time.time()

    if do_trajopt:
        trajopt_result = trajoptpy.OptimizeProblem(trajopt_problem)
        trajectory = trajopt_result.GetTraj()
    else:
        trajectory = compute_linear_trajectory(request.action, trajopt_problem, env)

    # Compute elapsed time.
    t_elapsed = time.time() - t_start
    print "optimization took %.3f seconds"%t_elapsed

    # Get the robot.
    robot = env.GetRobot('crichton')

    # Set robot DOFs to DOFs in optimization problem.
    trajopt_problem.SetRobotActiveDOFs()

    # Create motion plan response.
    response = apc_msgs.srv.ComputeDenseMotionResponse()

    # Reset collision properties.
    reset_target_item_collision_properties(request.action, trajopt_problem, env)

    # Exit early if we did not run the optimization.
    if no_optimization:
        response.action = request.action
        response.valid = True
        response.collision_free = True
    else:
        # Is the trajectory collision free.
        response.collision_free = check_for_collisions_interp(request.action, trajopt_problem, trajectory, env)

        # Does the trajectory only move the joint names requested.
        response.valid = not check_for_nonactive_joint_motion(request.action, trajopt_problem, trajectory, env)

        # Fill response.
        action = request.action
        fill_response_action(action, trajopt_problem, trajectory, env)
        response.action = action

        print "collsion:", not response.collision_free

    print "--------------------              END              --------------------"
    return response


def main():
    # Initialize ROS.
    rospy.init_node('apc_trajopt_server')

    roslaunch = rospy.get_param('~roslaunch', False)

    if not roslaunch:
        parser = argparse.ArgumentParser(description='apc_trajopt_server')
        parser.add_argument('-i', '--interactive', action='store_true', help='interactive mode')
        parser.add_argument('-d', '--debug', action='store_true', help='print debug')
        parser.add_argument('-n', '--no-optimization', action='store_true', help='no optimization')
        args = parser.parse_args()
    else:
        class Args:
            def __init__(self):
                self.debug = rospy.get_param('~debug', True)
                self.interactive = rospy.get_param('~interactive', False)
                self.no_optimization = rospy.get_param('~no_optimization', False)
        args = Args()

    global debug
    global no_optimization
    debug = args.debug
    no_optimization = args.no_optimization

    # Initialize OpenRAVE.
    print "Starting up OpenRAVE..."
    global env
    env = init_openrave()

    # Initialize TRAJOPT.
    print "Starting up TRAJOPT..."
    init_trajopt(args.interactive)

    # Create service and loop.
    print "Starting up ROS..."
    service_topic = rospy.get_param('~topic', 'compute_dense_motion_service_0')
    service = rospy.Service(service_topic, apc_msgs.srv.ComputeDenseMotion, motion_planning_service)
    rospy.spin()


if __name__ == "__main__":
    main()
