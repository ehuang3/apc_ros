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
import trajoptpy.math_utils as mu

from IPython.core.debugger import Tracer

import argparse

env = None
debug = False
interactive = True
# interactive = False

def initOpenRAVE():
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


def initTRAJOPT():
    """Initialize trajectory optimization"""
    # Pause every iteration, until you press 'p'. Press escape to disable further plotting.
    trajoptpy.SetInteractive(interactive)


def loadItemToOpenRAVE(object_id, object_key):
    """Load 'i'-th instance of 'object_name' into openrave environment."""
    # Get the global openrave environment.
    global env
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


def loadAndSetItemsToOpenRAVE(request):
    """ Load new objects and set them to the correct transform."""
    # Get the global openrave environment.
    global env

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
            loadItemToOpenRAVE(obj_id, obj_key)
        # Construct transform from scene world data.
        pose = obj.object_pose
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        T = tf.transformations.quaternion_matrix(q)
        v = np.array([pose.position.x, pose.position.y, pose.position.z])
        T[:3, 3] = v

        # Stupid hack to rotate the convex decomp mesh because osg
        # can't load the collada file correctly.
        if obj_key == "kiva_pod":
            T_hack = np.array( [ [ 1, 0, 0, 0 ],
                                 [ 0, 0, -1, 0 ],
                                 [ 0, 1, 0, 0 ],
                                 [ 0, 0, 0, 1 ] ] , dtype=np.float64)
            T = T.dot(T_hack)

        rospy.loginfo("Setting %s to\n%s", obj_key, T)
        # Copy pose into openrave KinBody.
        env.GetKinBody(obj_key).SetTransform(T)
        # Mark the openrave object in the dirty map as 'clean'.
        dirty[obj_key] = 0

    # For each object that is still dirty...
    if dirty:
        for obj_key in dirty.keys():
            # Remove from the openrave environment.
            if dirty[obj_key] == 1:
                env.RemoveBody(obj_key)


def printKinBodiesInCollision():
    """
    Print out all colliding kinbodies in the environment.

    """
    global env
    for object in env.GetBodies():
        if env.CheckCollision(object):
            print object.GetName(), "in collision"


def setRobotState(request):
    """Set robot 'crichton' to correct joint angles"""
    # Use the global copy of openrave environment.
    global env

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
    print "grasp", request.action.grasp

    # setTargetItemCollisionProperties(request.action)


def printAllowableCollisions(cc, kinbody, env):
    """
    Print out the allowed collision pairs between kinbody and the rest
    of the environment for a given collision checker.

    """
    for source_link in kinbody.GetLinks():
        for target_body in env.GetBodies():
            for target_link in target_body.GetLinks():
                if cc.CanCollide(source_link, target_link):
                    print source_link.GetName(), "can collide with", target_link.GetName()


def setTargetItemCollisionProperties(action, problem):

    """
    This function sets collision checking for the target item to
    enabled or disabled depending on whether the action is a pre-
    grasp trajectory, a grasp, or a post-grasp trajectory.

    """
    global env
    # 1. Set the grabbed state and openrave collision properties.
    moving = not (action.joint_trajectory.points[0].positions == action.joint_trajectory.points[-1].positions)
    grasping = action.grasp
    # If the action is moving the joints and not grasping (i.e. a pre-grasp
    # trajectory) or if the action is not moving and grasping (i.e. a grasp),
    # we'll disable collisions. That means if the action is moving and grasping
    # (i.e. a post-grasp trajectory), collisions are enabled for the item.
    disable_collisions = (moving and not grasping) or (not moving and grasping)
    # Of course, we should only disable collisions if the action has an object
    # it is interacting with.
    disable_collisions = action.object_key and disable_collisions

    # FIXME Remove this section
    # # If we need to disable collisions, do that for the item in the openrave
    # # context. We will still need to disable collisions in the bullet collision
    # # checker that trajopt uses later. This is for the collision checking we do
    # # post-optimization.
    # if disable_collisions:
    #     item = env.GetKinBody(action.object_key)
    #     for link in item.GetLinks():
    #         # link.Enable(not disable_collisions)
    #         link.Enable(True)
    # # If we don't need to, make sure to enable all links and bodies in the
    # # environment for collision checking.
    # else:
    #     for kinbody in env.GetBodies():
    #         for link in kinbody.GetLinks():
    #             link.Enable(True)

    # If grasping, grab the item with the robot in openrave. This allows us to
    # plan with the grabbed object.
    if grasping:
        item = env.GetKinBody(action.object_key)
        robot = env.GetRobot('crichton')
        link  = robot.GetLink(action.attached_link_id)
        robot.Grab(item, link)
        for kinbody in robot.GetGrabbed():
            for link in kinbody.GetLinks():
                link.Enable(not disable_collisions)
    # If not grasping, release all previously grabbed objects ;).
    else:
        robot = env.GetRobot('crichton')
        robot.ReleaseAllGrabbed()
    # 2. Set the collision properties for the bullet collision checkers used in
    # trajopt's optimization algorithm. If the action is a pre-grasp trajectory,
    # we do not disable collisions because we want the grasp to remain open
    # until the last second when the fingers close around the object. If the
    # action is a grasp or a post-grasp trajectory, we want to disable collision
    # pairs between the end-effector and the grabbed item. If the action is a
    # non-prehensile movement, TODO.
    disable_collisions = action.object_key and grasping
    # If so, disable collisions between the item links and all links in the
    # robot.
    if disable_collisions:
        robot = env.GetRobot('crichton')
        item = env.GetKinBody(action.object_key)
        collision_checkers = problem.GetCollisionCheckers()
        for cc in collision_checkers:
            for item_link in item.GetLinks():
                print "Disabling collisions between", item_link.GetName(), "and crichton"
                for robot_link in robot.GetLinks():
                    # print "Disabling collisions between ", item_link.GetName(), "and", robot_link.GetName()
                    cc.ExcludeCollisionPair(item_link, robot_link)
                    cc.ExcludeCollisionPair(robot_link, item_link)
                for grabbed in robot.GetGrabbed():
                    for grab_link in grabbed.GetLinks():
                        cc.ExcludeCollisionPair(item_link, grab_link)
                        cc.ExcludeCollisionPair(grab_link, item_link)
                for item_link2 in item.GetLinks():
                    cc.ExcludeCollisionPair(item_link, item_link2)
                    cc.ExcludeCollisionPair(item_link2, item_link)
            # DEBUG
    if action.object_key:
        item = env.GetKinBody(action.object_key)
        collision_checkers = problem.GetCollisionCheckers()
        for cc in collision_checkers:
            printAllowableCollisions(cc, item, env)
    # If we are grabbing something and moving, enable collisions
    # between the item and the rest of the items. May want to change
    # this to shelf.
    enable_item_collisions = moving and grasping
    if enable_item_collisions:
        item = env.GetKinBody(action.object_key)
        collision_checkers = problem.GetCollisionCheckers()
        for cc in collision_checkers:
            kiva_pod = env.GetKinBody('kiva_pod')
            for item_link in item.GetLinks():
                for kiva_link in kiva_pod.GetLinks():
                    cc.IncludeCollisionPair(item_link, kiva_link)
                    cc.IncludeCollisionPair(kiva_link, item_link)
            # other_items = [x for x in env.GetKinBodies()]



    # If we are not disabling collisions, enable collisions between all item
    # links and all links in the robot. FIXME This is not necessary as trajopt
    # collision checkers are recreated for each service call.
    # else:
    #     robot = env.GetRobot('crichton')
    #     item = env.GetKinBody(action.object_key)
    #     collision_checkers = problem.GetCollisionCheckers()
    #     for cc in collision_checkers:
    #         for item in env.GetBodies():
    #             # Skip over kinbodies that are also robots.
    #             if env.GetRobot(item.GetName()):
    #                 continue
    #             for item_link in item.GetLinks():
    #                 for robot_link in robot.GetLinks():
    #                     cc.IncludeCollisionPair(item_link, robot_link)


def buildTrajoptRequest(srv_request):
    global env
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
    if debug:
        print "Robot goal DOF values\n", joint_target

    # Compute the required distance penalty. For pre-grasp trajectories, the
    # distance penalty should be small and positive, to encourage the fingers to
    # avoid the object until the grasp.
    distance_penalty = 0.0
    moving = not (srv_request.action.joint_trajectory.points[0].positions ==
                  srv_request.action.joint_trajectory.points[-1].positions)
    grasping = srv_request.action.grasp
    pregrasp = moving and not grasping
    if pregrasp:
        distance_penalty = 0.02 # 1 cm

    # Fill out trajopt request.
    trajopt_request = {
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


def checkInterpCollisionFree(action, problem, result, dn=10):
    """
    Verify that the interpolated trajectory generated by trajopt is collision
    free. Argument 'dn' controls how fine the step sizes between points is.

    """
    global env
    # An action is a pre-grasp trajectory if we are moving towards a grasp.
    moving = not (action.joint_trajectory.points[0].positions == action.joint_trajectory.points[-1].positions)
    grasping = action.grasp
    pregrasp = moving and not grasping and action.object_key
    # If the action is a pre-grasp, then we do not check the last segment for
    # collisions.
    pregrasp_off = 0
    if pregrasp:
        pregrasp_off = 1
    # Check for collisions.
    robot = env.GetRobot("crichton")
    T = result.GetTraj()
    num_pts = T.shape[0]
    num_dof = T.shape[1]
    for i in range(num_pts - 1 - pregrasp_off):
        T_i = T[i:i+1,:]
        T_i = mu.interp2d(np.linspace(0,1,dn), np.linspace(0,1,len(T_i)), T_i)
        for (_,dofs) in enumerate(T_i):
            robot.SetActiveDOFValues(dofs)
            for robot_link in robot.GetLinks():
                collision = env.CheckCollision(robot_link)
                if collision:
                    print robot_link.GetName(), "in collision at", i
                    return False
    return True


def checkBulletCollisionFree(action, problem, result):
    """
    Verify that the trajectory generated by trajopt is collision free.
    FIXME Fails at last timestep on grasps.

    """
    global env
    robot = env.GetRobot("crichton")
    collision_checkers = problem.GetCollisionCheckers()
    for cc in collision_checkers:
        collisions = cc.BodyVsAll(robot)
        if len(collisions) > 0:
            return False
    return True


def printBulletCollisions(problem):
    """
    Print bullet collisions. These are the collisions of trajopt's internal
    optimization machinery.

    """
    collision_checkers = problem.GetCollisionCheckers()
    for cc in collision_checkers:
        collisions = cc.AllVsAll()
        for c in collisions:
            print c.GetCollisionInfo()


def motion_planning_service(request):

    """Perform motion planning using trajectory optimization"""
    # Use the global copy of openrave environment.
    global env

    # Load and set objects to correct location.
    loadAndSetItemsToOpenRAVE(request)

    # Get the robot.
    with env:
        robot = env.GetRobot('crichton')

    # Set robot to correct joint angles and positions.
    setRobotState(request)
    printKinBodiesInCollision()

    # Create json problem definition.
    trajopt_request = buildTrajoptRequest(request)
    s = json.dumps(trajopt_request) # convert dictionary into json-formatted string
    prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
    t_start = time.time()

    # Tracer()()

    # Set collision matrix information and grabbed bodies.
    setTargetItemCollisionProperties(request.action, prob)
    # HACK Reload objects and positions in case an object is "ungrabbed"
    loadAndSetItemsToOpenRAVE(request)

    result = trajoptpy.OptimizeProblem(prob) # do optimization
    t_elapsed = time.time() - t_start
    # print result
    # print "optimization took %.3f seconds"%t_elapsed

    robot = env.GetRobot('crichton')
    prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
    if debug:
        print result.GetTraj()
    # Do not assert collision free trajectories for now...
    # assert traj_is_safe(result.GetTraj(), robot)

    print "=========================OPENRAVE COLLISIONS============================"
    # printKinBodiesInCollision()
    print "=========================BULLET COLLISIONS=============================="
    # printBulletCollisions(prob)
    print "=========================END============================================"

    # Create motion plan response.
    response = apc_msgs.srv.ComputeDenseMotionResponse()

    # Is the trajectory collision free.
    # response.valid = traj_is_safe(result.GetTraj(), robot)
    # response.valid = True
    response.valid = checkInterpCollisionFree(request.action, prob, result)

    # Is the trajectory collision free.
    # response.collision_free = traj_is_safe(result.GetTraj(), robot)
    # response.collision_free = True
    response.collision_free = checkInterpCollisionFree(request.action, prob, result)

    # Fill response.
    trajectory = result.GetTraj()
    action = request.action
    action.joint_trajectory.points = []
    for i in range(len(trajectory)):
        action.joint_trajectory.joint_names = [joint.GetName() for joint in robot.GetJoints()]
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = trajectory[i]
        action.joint_trajectory.points.append(point)
    response.action = action
    # print response

    return response


def main():
    # Initialize ROS.
    rospy.init_node('apc_trajopt_bridge')

    # Initialize OpenRAVE.
    print "Starting up OpenRAVE..."
    initOpenRAVE()

    # Initialize TRAJOPT.
    print "Starting up TRAJOPT..."
    initTRAJOPT()

    # Create service and loop.
    print "Starting up ROS..."
    service_topic = rospy.get_param('~topic', 'motion_planning_service')
    service = rospy.Service(service_topic, apc_msgs.srv.ComputeDenseMotion, motion_planning_service)
    rospy.spin()


if __name__ == "__main__":
    main()
