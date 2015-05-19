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
import openravepy
import trajoptpy
import trajoptpy.math_utils as mu
from IPython.core.debugger import Tracer
from .action import *
from .openrave import *
from .apc_assert import apc_colors

import apc_msgs.srv 


class PreGrasp(object):

    def __init__(self):
        self.env = None

    def set_preshaping_collision_properties(self, action, bins, env, reset = False):
        # Set the robot to be in collision with the grasped item.
        enable = True and not reset
        item_key = action.object_key

        Tracer()()

        env.GetKinBody(item_key).Enable(enable)

        # Find out which bin has the grasped item.
        bin_id = ""
        for bin in bins:
            for object in bin.object_list:
                if object.object_key == item_key:
                    apc_assert(len(bin_id) == 0, "Duplicate object key!")
                    bin_id = bin.bin_name

        # Disable collisions with other objects in the bin.
        enable = False or reset
        for bin in bins:
            if bin.bin_name == bin_id:
                for object in bin.object_list:
                    if object.object_key == item_key:
                        continue
                        env.GetKinBody(object.object_key).Enable(enable)

        # Disable collisions with the bin.
        enable = False or reset
        env.GetKinBody("kiva_pod").Enable(enable)


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


    def compute_pregrasp_finger_shapes(self, action, bins, env):
        """
        Assumes? that the collision properties are set correctly.

        """
        apc_assert(is_action_grasp(action),
                   "Input action to pregrasp finger shaping is invalid")

        # Get the robot.
        robot = env.GetRobot('crichton')

        # Set the robot to be in collision with the grasped item, but not
        # in collision with the other items or the shelf.
        self.set_preshaping_collision_properties(action, bins, env)

        # Set robot to grasp joint angles.
        self.set_action_to_robot_state(action, robot, -1)

        # Open the fingers until they are no longer in collision with the
        # object. Only open the fingers that are in collision with the
        # object.
        report = openravepy.CollisionReport()
        

        Tracer()()

        # Reset the environment collision properties.
        self.set_preshaping_collision_properties(action, bins, env, True)


    def init_ros(self):
        # Initialize ROS.
        rospy.init_node('apc_grasping')

        # parser = argparse.ArgumentParser(description='apc_grasping')
        # parser.add_argument('-i', '--interactive', action='store_true', help='interactive mode')
        # parser.add_argument('-d', '--debug', action='store_true', help='print debug')
        # parser.add_argument('-n', '--no-optimization', action='store_true', help='no optimization')
        # args = parser.parse_args()

        # Initialize OpenRAVE.
        print "Starting up OpenRAVE..."
        self.env = init_openrave()

        service_topic = rospy.get_param('~topic', 'compute_pregrasps')
        service = rospy.Service(service_topic, apc_msgs.srv.ComputePreGrasps, self.compute_pregrasps_service)

        


    def compute_pregrasps_service(self, request):
        print "--------------------             START             --------------------"

        # Load and set objects to correct location.
        load_and_set_items_to_openrave(request, self.env)

        # Set robot to correct joint angles and positions.
        set_robot_state_to_openrave(request, self.env)

        # Compute a pregrasp for each input grasp.
        pregrasps = []
        for grasp in request.grasps:
            # set_robot_state_to_action(grasp, self.env)
            pregrasp = self.compute_pregrasp_finger_shapes(grasp, request.bin_states, self.env)
            pregrasps.append(pregrasp)

        response = apc_msgs.srv.ComputePreGraspsResponse()
        response.pregrasps = pregrasps

        print "--------------------              END              --------------------"

        return response
