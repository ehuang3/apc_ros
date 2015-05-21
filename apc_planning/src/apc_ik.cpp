#include <apc_planning/ik.h>

Ik ik;


bool compute_ik(apc_msgs::ComputeIkRequest& request,
                apc_msgs::ComputeIkResponse& response)
{
    //
    ROS_INFO("Fot asldk");

    ik.compute_ik_service(request, response);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "apc_ik");
    ros::NodeHandle n;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
    ik.robot_state_.reset(new robot_state::RobotState(planning_scene->getCurrentStateNonConst()));

    ros::ServiceServer service = n.advertiseService("compute_ik", compute_ik);

    ros::spin();
}
