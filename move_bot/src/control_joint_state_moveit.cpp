#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    geometry_msgs::Pose target_pose;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    while(ros::ok()){
        std::cout << "\nEnter the x y z coordinates to achieve: ";
        std::cin >> target_pose.position.x >> target_pose.position.y >> target_pose.position.z;
        move_group_interface.setPoseTarget(target_pose);

        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success){
            move_group_interface.execute(my_plan);
        } else{
            std::cout << "\nError Occured\n Maybe the point does not lie in the workspace.\n";
        }
    }

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
}