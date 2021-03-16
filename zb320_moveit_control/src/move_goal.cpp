#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_goal");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // SETUP
    static const std::string PLANNING_GROUP = "zb320_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    // VISUALIZATION
    namespace rvt = rviz_visual_tools;
    
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.3;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // GETTING BASIC INFORMATION
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

    // START DEMO
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // PLANNING
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.0;
    target_pose.position.y = 0.15;
    target_pose.position.z = 0.15;
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // VISUALIZING
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // MOVING TO GOAL
    move_group.move();

    // END
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish the demo");
    ros::shutdown();
    return 0;
}