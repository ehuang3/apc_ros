#pragma once
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <apc_msgs/ComputeIk.h>
#include <apc_planning/error.h>
#include <apc_msgs/WorldState.h>

typedef std::map<std::string, Eigen::Affine3d, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d> > >
KeyPoseMap;

struct Ik
{
    robot_state::RobotStatePtr robot_state_;
    // Ik();
    Ik(){}

    void compute_ik_service(apc_msgs::ComputeIkRequest& request,
                            apc_msgs::ComputeIkResponse& response);

    void compute_ik(robot_state::RobotState& robot_state,
                    const std::string& link_id,
                    const std::string& group_id,
                    const std::string& frame_id,
                    const Eigen::Affine3d& T_frame_world,
                    const geometry_msgs::Pose& pose_link_frame,
                    const std::vector<geometry_msgs::Pose>& symmetries);

    void compute_ik(robot_state::RobotState& robot_state,
                    const std::string& link_id,
                    const std::string& group_id,
                    const std::string& frame_id,
                    const Eigen::Affine3d& T_frame_world,
                    const geometry_msgs::Pose& pose_link_frame);
};
