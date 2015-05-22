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
import trajoptpy
import trajoptpy.math_utils as mu
from IPython.core.debugger import Tracer
from .action import *
from .openrave import *
from .apc_assert import apc_colors
from copy import deepcopy
from openravepy import *

import apc_msgs.srv
import re

class Ik(object):

    def __init__(self):
        self.env = None

    def set_robot_state_from_ik(self, action, robot, env):
        pass

    def compute_ik_service(self, request):
        print "hello"

        robot = self.env.GetRobot('crichton')
        print robot.GetManipulators()
        # manip = robot.GetManipulators()[0]
        Tracer()()
        # robot.SetActiveManipulator('leftarm_torso')
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        print "hello"

    def init_ros(self):
        # Initialize ROS.
        rospy.init_node('apc_ik')

        # parser = argparse.ArgumentParser(description='apc_grasping')
        # parser.add_argument('-i', '--interactive', action='store_true', help='interactive mode')
        # parser.add_argument('-d', '--debug', action='store_true', help='print debug')
        # parser.add_argument('-n', '--no-optimization', action='store_true', help='no optimization')
        # args = parser.parse_args()

        # Initialize OpenRAVE.
        print "Starting up OpenRAVE..."
        self.env = init_openrave()

        print "Starting up ROS..."
        service_topic = rospy.get_param('~topic', 'compute_ik')
        service = rospy.Service(service_topic, apc_msgs.srv.ComputeIk, self.compute_ik_service)
