#pragma once
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <apc_msgs/CheckCollisions.h>

struct CollisionChecker
{
    planning_scene::PlanningScenePtr scene_;
    CollisionChecker();
    CollisionChecker(planning_scene::PlanningScenePtr scene);
    void setPlanningScene(planning_scene::PlanningScenePtr scene);
    bool checkCollisions(robot_state::RobotState& robot_state);
};
