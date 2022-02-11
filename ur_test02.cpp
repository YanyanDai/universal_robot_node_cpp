#include<moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
 
int main(int argc, char **argv)
{
    //initialize，ur_test02 is node name
    ros::init(argc, argv, "ur_test02");
    //multiple threshold
    ros::AsyncSpinner spinner(1);
    //start new threshod
    spinner.start();
 
    //setup move group class 
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    
   //set joint tolerance
    arm.setGoalJointTolerance(0.001);
   //set max acceleration and velocity
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);
 
    // robot move to the home position 
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);
 
    //robot move to the up position
    arm.setNamedTarget("up");
    arm.move();
    sleep(1);
 
    // robot moe to the home position
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);
 
    ros::shutdown();
 
    return 0;
}
