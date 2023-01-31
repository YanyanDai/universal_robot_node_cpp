#include<moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
 
int main(int argc, char **argv)
{
    //Initialize ROS, create the node handle and an async spinner
    ros::init(argc, argv, "ur_test01");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();
 
    //Get the arm planning group
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    // Create a published for the arm plan visualization
    ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

   //Set the tolerance to consider the goal achieved
    arm.setGoalJointTolerance(0.02);
    //arm.setGoalPositionTolerance(0.01);
    //arm.setGoalOrientationTolerance(0.01);

   //Set max acceleration and velocity
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    ROS_INFO_STREAM("#" << arm.getCurrentPose());
    ROS_INFO_STREAM("#" << arm.getCurrentState());

    // Set a goal message as a pose of the end effector
    geometry_msgs::Pose goal;
    goal.orientation.x = arm.getCurrentPose().pose.orientation.x;
    goal.orientation.y = arm.getCurrentPose().pose.orientation.y;
    goal.orientation.z = arm.getCurrentPose().pose.orientation.z;
    goal.orientation.w = arm.getCurrentPose().pose.orientation.z;
    goal.position.x = arm.getCurrentPose().pose.position.x + 0.05; 
    goal.position.y = arm.getCurrentPose().pose.position.y + 0.1; 
    goal.position.z = arm.getCurrentPose().pose.position.z;

    //Set the target pose, which is the goal we already defined
    arm.setPoseTarget(goal);

    // Perform the planning step, display the current arm trajectory and move the arm
    moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
    moveit_msgs::DisplayTrajectory display_msg;
    display_msg.trajectory_start = goal_plan.start_state_;
    display_msg.trajectory.push_back(goal_plan.trajectory_);
    display_pub.publish(display_msg);
    
    sleep(3);
    
    arm.move();

    ROS_INFO_STREAM("#" << arm.getCurrentPose());

    ros::shutdown();
 
    return 0;
}
