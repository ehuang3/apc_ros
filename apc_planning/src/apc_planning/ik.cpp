#include <apc_planning/ik.h>
#include <apc_planning/error.h>

double elementWiseMatrixNorm(const Eigen::Affine3d& A, const Eigen::Affine3d& B)
{
    Eigen::Matrix4d M;
    for (int i = 0; i < M.rows(); i++)
        for (int j = 0; j < M.cols(); j++)
            M(i,j) = A(i,j) - B(i,j);
    return M.norm() / (double) M.rows() * M.cols();
}

KeyPoseMap convertWorldStateMsgToKeyPoseMap(const apc_msgs::WorldState& world_msg)
{
    KeyPoseMap world_state;
    for (int i = 0; i < world_msg.frames.size(); i++) {
        const apc_msgs::FrameState& F = world_msg.frames[i];
        std::string frame_key = F.frame_key;
        Eigen::Affine3d T_frame_world;
        tf::poseMsgToEigen(F.frame_pose, T_frame_world);
        world_state[frame_key] = T_frame_world;
    }
    return world_state;
}

void setRobotStateToAction(const robot_state::RobotState& robot_state,
                           apc_msgs::PrimitiveAction& action,
                           int index)
{
    if (index == -1)
        index = action.joint_trajectory.points.size() -1;
    APC_ASSERT(action.joint_trajectory.joint_names.size() > 0,
               "Missing joint names in action %s joint trajectory", action.action_name.c_str());
    APC_ASSERT(0 <= index && index < action.joint_trajectory.points.size(),
               "Index out of bounds for action %s joint trajectory", action.action_name.c_str());
    trajectory_msgs::JointTrajectoryPoint point;
    std::vector<std::string> joint_names = action.joint_trajectory.joint_names;
    for (int i = 0; i < joint_names.size(); i++) {
        int index = robot_state.getJointModel(joint_names[i])->getFirstVariableIndex();
        double angle = robot_state.getVariablePosition(index);
        point.positions.push_back(angle);
    }
    action.joint_trajectory.points[index] = point;
}

void Ik::compute_ik_service(apc_msgs::ComputeIkRequest& request,
                            apc_msgs::ComputeIkResponse& response)
{
    // Set robot to start state.
    robot_state::RobotState robot_state = *this->robot_state_;
    robot_state::robotStateMsgToRobotState(request.robot_state, robot_state);

    // GEt work state.
    KeyPoseMap world_state = convertWorldStateMsgToKeyPoseMap(request.world_state);

    std::cout << request << std::endl;

    //
    for (int i = 0; i < request.actions.size(); i++) {
        try {
            const apc_msgs::PrimitiveAction& action = request.actions[i];
            APC_ASSERT(!action.eef_link_id.empty(), "Missing link ID");
            APC_ASSERT(!action.frame_id.empty(), "Missing frame ID");
            APC_ASSERT(!action.group_id.empty(), "Missing group ID");
            APC_ASSERT(!action.frame_key.empty(), "Missing frame KEY");
            APC_ASSERT(action.eef_locked, "EEF not locked");
            APC_ASSERT(action.joint_trajectory.points.size() == 2, "More than 2 points in trajectory");
            APC_ASSERT(action.eef_trajectory.poses.size() == 2, "More than 2 points in trajectory");

            Eigen::Affine3d T_frame_world = world_state[action.frame_key];
            robot_state::RobotState robot_copy = robot_state;

            try {
                compute_ik(robot_copy, action.eef_link_id, action.group_id, action.frame_id,
                           T_frame_world, action.eef_trajectory.poses.front());
            } catch (apc_exception::Exception& error) {
                ROS_INFO("error:%s", error.what());
            }

            apc_msgs::PrimitiveAction ik_action = action;
            ik_action.frame_id = "";
            ik_action.frame_key = "";
            setRobotStateToAction(robot_copy, ik_action, 0);
            setRobotStateToAction(robot_copy, ik_action, -1);

            response.actions.push_back(ik_action);

        } catch (apc_exception::Exception& error) {
            ROS_ERROR("Failed IK\n%s", error.what());
        }
    }

}

void Ik::compute_ik(robot_state::RobotState& robot_state,
                    const std::string& link_id,
                    const std::string& group_id,
                    const std::string& frame_id,
                    const Eigen::Affine3d& T_frame_world,
                    const geometry_msgs::Pose& pose_link_frame)
{
    APC_ASSERT(link_id.size() > 0, "Failed to provide input link");
    APC_ASSERT(group_id.size() > 0, "Failed to provide input group");
    APC_ASSERT(frame_id.size() > 0, "Failed to provide input frame");

    bool use_symmetries = false;

    EigenSTL::vector_Affine3d frame_symmetries;
    frame_symmetries.push_back(Eigen::Affine3d::Identity());

    // If the group does not have an IK solver, return false.
    bool solver = robot_state.getJointModelGroup(group_id)->getSolverInstance();

    std::vector<const moveit::core::JointModelGroup*> subgroups;
    robot_state.getJointModelGroup(group_id)->getSubgroups(subgroups);
    for (int i = 0; i < subgroups.size(); i++)
        if (solver = subgroups[i]->getSolverInstance())
            break;
    APC_ASSERT(solver,
               "Failed to find IK solver for group %s", group_id.c_str());

    Eigen::Affine3d T_symmetry_world_min = Eigen::Affine3d::Identity();
    double min_x = 1e9;
    double max_z = 0;
    for (int i = 0; i < frame_symmetries.size(); i++) {
        // Copy the robot state for IK.
        robot_state::RobotState ik_robot = robot_state;

        // Back out the desired link transform in global coordinates.
        Eigen::Affine3d T_symmetry_frame = frame_symmetries[i];
        // Eigen::Affine3d T_frame_link = T_symmetry_link * T_symmetry_frame.inverse();
        // Eigen::Affine3d T_link_world = T_frame_world * T_frame_link.inverse();
        Eigen::Affine3d T_symmetry_world = T_frame_world * T_symmetry_frame;

        // Back out the desired link transform in global coordinates.
        Eigen::Affine3d T_frame_link;
        tf::poseMsgToEigen(pose_link_frame, T_frame_link);
        Eigen::Affine3d T_link_world = T_symmetry_world * T_frame_link.inverse();

        // Snap to the frame using IK.
        const moveit::core::JointModelGroup* jmg = robot_state.getJointModelGroup(group_id);
        geometry_msgs::Pose pose_link_world;
        tf::poseEigenToMsg(T_link_world, pose_link_world);
        ik_robot.setFromIK(jmg, pose_link_world, link_id);
        // APC_ASSERT(robot.setFromIK(jmg, pose_link_world, link_id),
        //            "Failed to set %s to pose using %s IK", link_id.c_str(), group_id.c_str());
        ik_robot.update();

        Eigen::Affine3d T_ik = ik_robot.getGlobalLinkTransform(link_id);

        if (elementWiseMatrixNorm(T_ik, T_link_world) < 1e-2) {

            double delta_x = std::abs(T_link_world.translation().x()) - min_x;
            double delta_z = std::abs(T_link_world.translation().z()) - max_z;

            if (std::abs(delta_x) < 3e-2) {
                if (delta_z > 0) {
                    min_x = std::abs(T_link_world.translation().x());
                    max_z = std::abs(T_link_world.translation().z());
                    T_symmetry_world_min = T_symmetry_world;
                }
            }
            else if(delta_x < 0) {
                min_x = std::abs(T_link_world.translation().x());
                max_z = std::abs(T_link_world.translation().z());
                T_symmetry_world_min = T_symmetry_world;
            }
        }

        // Manually assert whether the new state places the end-effector at the
        // desired IK position.
        if (!use_symmetries) {
            robot_state = ik_robot;
            APC_ASSERT(elementWiseMatrixNorm(T_ik, T_link_world) < 1e-2,
                       "Failed to IK group %s to pose; error is %.6f", group_id.c_str(),
                       elementWiseMatrixNorm(T_ik, T_link_world));
        }
    }

    return;
}
