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
roslib.load_manifest('apc_trajopt')
import apc_msgs.msg
import apc_msgs.srv
import trajectory_msgs
import rospy
import rospkg
import tf.transformations
import numpy as np
import openravepy
import trajoptpy
from trajoptpy.check_traj import traj_is_safe

from IPython.core.debugger import Tracer

import argparse

apc_objects = ['champion_copper_plus_spark_plug', 'mark_twain_huckleberry_finn', 'cheezit_big_original']
env = None
debug = False
interactive = False

def init_openrave():
    """Initialize OpenRAVE and load models"""

    # Create an openrave environment and store into the global variable.
    global env
    env = openravepy.Environment()

    # Stop simulation.
    env.StopSimulation()

    # Get path to models
    rospack = rospkg.RosPack()
    path = rospack.get_path('apc_description')

    # Load robot.
    env.Load(path + "/collada/crichton/crichton.dae")


def init_trajopt():
    """Initialize trajectory optimization"""
    # Pause every iteration, until you press 'p'. Press escape to disable further plotting.
    trajoptpy.SetInteractive(interactive)


def load_object(object_name, i):
    """Load 'i'-th instance of 'object_name' into openrave environment."""
    # Get the global openrave environment.
    global env
    # Get path to models
    rospack = rospkg.RosPack()
    path = rospack.get_path('apc_description')
    # Join together the object path to load.
    obj_path = os.path.join(path, 'objects')
    obj_path = os.path.join(obj_path, object_name)
    obj_path = os.path.join(obj_path, 'reduced_kinbody')
    obj_path = os.path.join(obj_path, 'kinbody.' + str(i) + '.xml')
    # Load object.
    env.Load(obj_path)

def load_and_set_objects(request):
    """ Load new objects and set them to the correct transform."""
    # Get the global openrave environment.
    global env

    # Build a map of loaded objects.
    dirty = {}
    for body in env.GetBodies():
        # As long as it's not a robot, mark the object as dirty.
        if not env.GetRobot(body.GetName()):
            dirty[body.GetName()] = 1

    # For each object in the scene world...
    for col_obj in request.scene.world.collision_objects:
        # Check that the file ending is 'stl'.
        if not col_obj.id.split('.')[-1] == 'stl':
            rospy.logwarn('Recieved non-stl object %s', col_obj.id)
        # Object base name, i.e. 'cheezit_big_original'.
        col_obj_base = col_obj.id.partition('.')[0]
        # Convert collision object name to xml name.
        i = 0
        col_obj_name = col_obj_base + '_' + str(i)
        # Look for the next unloaded or dirty object.
        while col_obj_name in dirty.keys() and dirty[col_obj_name] == 0:
            i = i + 1
            col_obj_name = col_obj_base + '_' + str(i)
        # If the collision object has not been loaded yet, load it.
        if col_obj_name not in dirty.keys():
            load_object(col_obj_base, i)
        # Construct transform from scene world data.
        pose = col_obj.mesh_poses[0]
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        T = tf.transformations.quaternion_matrix(q)
        v = np.array([pose.position.x, pose.position.y, pose.position.z])
        T[:3, 3] = v
        # Copy pose into openrave KinBody.
        env.GetKinBody(col_obj_name).SetTransform(T)
        # Mark the openrave object in the dirty map as 'clean'.
        dirty[col_obj_name] = 0

    # For each object that is still dirty...
    if dirty:
        for col_obj_name in dirty.keys():
            # Remove from the openrave environment.
            if dirty[col_obj_name] == 1:
                env.RemoveBody(col_obj_name)


def set_robot(request):
    """Set robot 'crichton' to correct joint angles"""
    # Use the global copy of openrave environment.
    global env

    # Set robot to correct joint angles and positions.
    joint_state = request.start_state.joint_state
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


def motion_planning_service(request):
    """Perform motion planning using trajectory optimization"""
    # Use the global copy of openrave environment.
    global env

    # Load and set objects to correct location.
    load_and_set_objects(request)

    # Get the robot.
    with env:
        robot = env.GetRobot('crichton')

    # Set robot to correct joint angles and positions.
    set_robot(request)

    # Construct goal state.
    joint_target = np.zeros(robot.GetDOF())
    with env:
        robot = env.GetRobot('crichton')
        goal = request.goal_state.joint_state
        if robot == None:
            print 'No robot found for "crichton"'
        else:
            for joint in robot.GetJoints():
                for i in range(len(goal.name)):
                    if joint.GetName() == goal.name[i]:
                        if debug:
                            print "Setting joint target", joint.GetName(), "to", goal.name[i], ":", goal.position[i]
                        joint_target[joint.GetDOFIndex()] = goal.position[i]
    joint_target = joint_target.tolist()
    if debug:
        print "Robot goal DOF values\n", joint_target

    # Tracer()()

    # Create json problem definition.
    request = {
        "basic_info" : {
            "n_steps" : 20,
            "manip" : "active", # see below for valid values
            "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
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
                    "dist_pen" : [0.025]
                },
            },
            # {
            #     "name" : "cont_coll",
            #     "type" : "collision",
            #     "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen], "continuous":True}
            # },
            {
                "name": "disc_coll",
                "type" : "collision",
                "params" : {"coeffs" : [1],"dist_pen" : [0.025], "continuous":False}
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
    s = json.dumps(request) # convert dictionary into json-formatted string
    prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
    t_start = time.time()

    result = trajoptpy.OptimizeProblem(prob) # do optimization
    t_elapsed = time.time() - t_start
    print result
    print "optimization took %.3f seconds"%t_elapsed

    robot = env.GetRobot('crichton')
    prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
    if debug:
        print result.GetTraj()
    # Do not assert collision free trajectories for now...
    # assert traj_is_safe(result.GetTraj(), robot)

    # Create motion plan response.
    response = apc_msgs.srv.GetMotionPlanResponse()

    # Is the trajectory collision free.
    response.valid = traj_is_safe(result.GetTraj(), robot)

    # Fill response.
    trajectory = result.GetTraj()
    action = apc_msgs.msg.PrimitiveAction()
    for i in range(len(trajectory)):
        action.joint_trajectory.joint_names = [joint.GetName() for joint in robot.GetJoints()]
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = trajectory[i]
        action.joint_trajectory.points.append(point)
    response.plan.actions.append(action)

    return response


def main():
    # Initialize ROS.
    rospy.init_node('apc_trajopt_bridge')

    # Initialize OpenRAVE.
    print "Starting up OpenRAVE..."
    init_openrave()

    # Initialize TRAJOPT.
    print "Starting up TRAJOPT..."
    init_trajopt()

    # Create service and loop.
    print "Starting up ROS..."
    service_topic = rospy.get_param('~topic', 'motion_planning_service')
    service = rospy.Service(service_topic, apc_msgs.srv.GetMotionPlan, motion_planning_service)
    rospy.spin()


if __name__ == "__main__":
    main()
