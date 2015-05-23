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
import numpy as np
import rospy
import rospkg
import tf.transformations
import openravepy
import pdb
from IPython.core.debugger import Tracer

def init_openrave():
    """Initialize OpenRAVE and load models"""
    # Create an openrave environment
    env = openravepy.Environment()
    # Stop simulation.
    env.StopSimulation()
    # Get path to models
    rospack = rospkg.RosPack()
    path = rospack.get_path('apc_description')
    # Load robot.
    env.Load(path + "/collada/crichton/crichton.dae")
    # Return env.
    return env


def load_item_to_openrave(object_id, object_key, env):
    """Load 'i'-th instance of 'object_name' into openrave environment."""
    # Get path to models
    rospack = rospkg.RosPack()
    path = rospack.get_path('apc_description')
    # Compute the object key "index". HACK
    if object_id == object_key:
        i = '0'
    else:
        i = object_key.split('_')[-1]
    # Join together the object path to load.
    obj_path = os.path.join(path, 'objects')
    obj_path = os.path.join(obj_path, object_id)
    obj_path = os.path.join(obj_path, 'reduced_kinbody')
    obj_path = os.path.join(obj_path, 'kinbody.' + i + '.xml')
    # Load object.
    env.Load(obj_path)


def load_and_set_items_to_openrave(request, env):
    """ Load new objects and set them to the correct transform."""
    # Build a map of loaded objects.
    dirty = {}
    for body in env.GetBodies():
        # As long as it's not a robot, mark the object as dirty.
        if not env.GetRobot(body.GetName()):
            dirty[body.GetName()] = 1

    # For each object in the world state...
    for obj in request.world_state.objects:
        # Get the ID and key.
        obj_id = obj.object_id
        obj_key = obj.object_key
        # If the collision object has not been loaded yet, load it.
        if obj_key not in dirty.keys():
            load_item_to_openrave(obj_id, obj_key, env)
        # Construct transform from scene world data.
        pose = obj.object_pose
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        T = tf.transformations.quaternion_matrix(q)
        v = np.array([pose.position.x, pose.position.y, pose.position.z])
        T[:3, 3] = v

        # Stupid hack to rotate the convex decomp mesh because osg
        # can't load the collada file correctly.
        if obj_key == "kiva_pod" or obj_key == "order_bin":
            T_hack = np.array( [ [ 1, 0, 0, 0 ],
                                 [ 0, 0, -1, 0 ],
                                 [ 0, 1, 0, 0 ],
                                 [ 0, 0, 0, 1 ] ] , dtype=np.float64)
            T = T.dot(T_hack)

        # rospy.loginfo("Setting %s to\n%s", obj_key, T)
        # Copy pose into openrave KinBody.
        env.GetKinBody(obj_key).SetTransform(T)
        # Mark the openrave object in the dirty map as 'clean'.
        dirty[obj_key] = 0

    # For each object that is still dirty...
    if dirty:
        for obj_key in dirty.keys():
            # Remove from the openrave environment.
            if dirty[obj_key] == 1 and obj_key:
                env.RemoveKinBody(obj_key)


def set_robot_state_to_openrave(request, env):
    """Set robot 'crichton' to correct joint angles"""
    # Set robot to correct joint angles and positions.
    joint_state = request.robot_state.joint_state
    with env:
        robot = env.GetRobot('crichton')
        dofs = np.zeros(robot.GetDOF())
        if robot == None:
            print 'No robot found for "crichton"'
        else:
            for joint in robot.GetJoints():
                for i in range(len(joint_state.name)):
                    if joint.GetName() == joint_state.name[i]:
                        rospy.logdebug("Setting joint %s to %s:%f",
                                       joint.GetName(), joint_state.name[i], joint_state.position[i])
                        dofs[joint.GetDOFIndex()] = joint_state.position[i]
            robot.SetDOFValues(dofs)
    # Debug output joint angles.
    rospy.logdebug("Robot starting DOF values\n%s", str(robot.GetDOFValues()))
    # print "grasp", request.action.grasp
