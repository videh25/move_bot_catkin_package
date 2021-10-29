#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "move_in_straight_line");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    
    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose target_pose;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::vector<geometry_msgs::Pose> waypoints;

    while(ros::ok()){
        waypoints = {};
        moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
        const Eigen::Isometry3d& current_state = start_state.getGlobalLinkTransform("W3Eff");
        std::cout << "Current Position: \n" << current_state.translation() << "\n";

        std::cout << "\nEnter the start and target coordinates (x1 y1 z1 x2 y2 z2) to achieve: ";
        std::cin >> start_pose.position.x >> start_pose.position.y >> start_pose.position.z >> target_pose.position.x >> target_pose.position.y >> target_pose.position.z;
        waypoints.push_back(start_pose);
        waypoints.push_back(target_pose);

        std::cout << waypoints.size() << "\n";

        moveit_msgs::RobotTrajectory rt;
        double fraction = move_group_interface.computeCartesianPath(waypoints,
                                                        0.01,  // eef_step
                                                        0.0,   // jump_threshold
                                                        rt);

        ROS_INFO("Visualizing line between the given points. (%.2f%% acheived)", fraction * 100.0);
        visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::XLARGE);
        std::cout << visual_tools.trigger() << "\n";
        move_group_interface.execute(rt);
        
    }
}