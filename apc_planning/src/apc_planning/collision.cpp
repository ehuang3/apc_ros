#include <apc_planning/collision.h>

CollisionChecker::CollisionChecker()
{}

CollisionChecker::CollisionChecker(planning_scene::PlanningScenePtr scene)
    : scene_(scene)
{}

void CollisionChecker::setPlanningScene(planning_scene::PlanningScenePtr scene)
{
    scene_ = scene;
}

// void CollisionChecker::checkSelfCollisions(robot_state::RobotState& robot_state)
// {
// }
