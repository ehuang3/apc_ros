 #include <apc_planning/collision.h>

CollisionChecker cc;


bool check_collisions(apc_msgs::CheckCollisionsRequest& request,
                      apc_msgs::CheckCollisionsResponse& response)
{
    // response = collision_checker.check_collisions_service(request);
    request.robot_state;

    robot_state::RobotState robot_state = *cc.scene_->getCurrentStateUpdated(request.robot_state);
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    cc.scene_->checkCollisionUnpadded(collision_request, collision_result, robot_state);

    ROS_INFO_STREAM("Test 1: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " self collision");
    typedef collision_detection::CollisionResult::ContactMap ContactMap;
    for (ContactMap::iterator iter = collision_result.contacts.begin();
         iter != collision_result.contacts.end(); ++iter) {
        ROS_INFO("collision: (%s x %s)", iter->first.first.c_str(), iter->first.second.c_str());
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "apc_collision");
    ros::NodeHandle n;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
    cc.setPlanningScene(planning_scene);

    ros::ServiceServer service = n.advertiseService("check_collisions", check_collisions);

    ros::spin();
}
