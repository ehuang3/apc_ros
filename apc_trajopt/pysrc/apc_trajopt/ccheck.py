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

import time
import numpy as np
import openravepy
import trajoptpy
import trajoptpy.math_utils as mu
from IPython.core.debugger import Tracer
from .action import *
from .openrave import *
from .collisions import *
from .apc_assert import apc_colors
from copy import deepcopy

import apc_msgs.srv
import re


class Collision(object):


    def __init__(self):
        self.env = None


    def set_action_to_robot_state(self, action, robot, i):
        joint_names = action.joint_trajectory.joint_names
        p_end = action.joint_trajectory.points[i].positions
        T = numpy.zeros((29))
        T[:] = robot.GetDOFValues()
        for joint in robot.GetJoints():
            for i in range(len(joint_names)):
                if joint.GetName() == joint_names[i]:
                    T[joint.GetDOFIndex()] = p_end[i]
        robot.SetActiveDOFValues(T)


    def set_collision_properties(self, action, bins, env, reset = False):
        if is_action_grasp(action):
            print "got grasp", action.action_name

        # Attach item to hand.
        enable = is_action_grasping(action) and not reset
        if enable:
            robot = env.GetRobot('crichton')
            item = env.GetKinBody(action.object_key)
            link  = robot.GetLink(action.attached_link_id)
            robot.Grab(item, link)
        else:
            robot = env.GetRobot('crichton')
            robot.ReleaseAllGrabbed()

        # Set default collision properties.
        set_openrave_collision_properties(action, bins, env, reset)

        # Enable collisions with the bin.
        enable = True and not reset
        env.GetKinBody("kiva_pod").Enable(enable)

        # Disable collisions with the target item.
        if action.object_key:
            enable = False or reset
            env.GetKinBody(action.object_key).Enable(enable)



    def check_collisions(self, action, bins, env):
        """
        Assumes all collision properties are set correctly.

        """
        robot = env.GetRobot('crichton')
        self.set_collision_properties(action, bins, env, False)

        # Set robot to start and end joint angles and check for collisions.
        self.set_action_to_robot_state(action, robot, 0)
        report = openravepy.CollisionReport()
        collision = env.CheckCollision(robot, report)
        if collision:
            print report
            return None
        self.set_action_to_robot_state(action, robot, -1)
        report = openravepy.CollisionReport()
        collision = env.CheckCollision(robot, report)
        if collision:
            print report
            return None

        self.set_collision_properties(action, bins, env, True)

        checked_action = deepcopy(action)
        checked_action.frame_id = ""
        checked_action.frame_key = ""

        return checked_action


    def check_collisions_service(self, request):
        print "--------------------             START             --------------------"

        # Load and set objects to correct location.
        load_and_set_items_to_openrave(request, self.env)

        # Set robot to correct joint angles and positions.
        set_robot_state_to_openrave(request, self.env)

        # Compute a pregrasp for each input grasp.
        collision_free = []
        for plan in request.plans:
            cf = True
            for action in plan.actions:
                # set_robot_state_to_action(grasp, self.env)
                checked = self.check_collisions(action, request.bin_states, self.env)
                if not checked:
                    cf = False
            if cf:
                collision_free.append(plan)

        response = apc_msgs.srv.CheckCollisionsResponse()
        response.plans = collision_free

        print "--------------------              END              --------------------"

        return response


    def init_ros(self):
        # Initialize ROS.
        rospy.init_node('apc_collision')
        # Initialize OpenRAVE.
        print "Starting up OpenRAVE..."
        self.env = init_openrave()

        print "Starting up ROS..."
        service_topic = rospy.get_param('~topic', 'check_collisions')
        service = rospy.Service(service_topic, apc_msgs.srv.CheckCollisions, self.check_collisions_service)


