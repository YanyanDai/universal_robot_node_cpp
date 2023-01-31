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

   //Set max acceleration and velocity
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    ROS_INFO_STREAM("#" << arm.getCurrentPose());
    ROS_INFO_STREAM("#" << arm.getCurrentState());
    
    //Obtain the current planning scene and wait until everything is up and running, 
    //otherwise the request won't succeed
    moveit::planning_interface::PlanningSceneInterface current_scene;

    sleep(5);

    //create a box with certain dimensions and orientation, 
    //give it a name which can later be used to remove from the scene
    moveit_msgs::CollisionObject box;

    box.id = "rosbook_box";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.2;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.3;
    pose.position.y = -0.3;
    pose.position.z = 0.2;

    box.primitives.push_back(primitive);
    box.primitive_poses.push_back(pose);
    box.operation = box.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(box);

    // Once all of the objects have been added to the vector, we tell the planning scene to add new box
    current_scene.addCollisionObjects(collision_objects);
    current_scene.applyCollisionObjects(collision_objects);
    // Set a goal message as a pose of the end effector
    geometry_msgs::Pose goal;
    goal.orientation.x = arm.getCurrentPose().pose.orientation.x;
    goal.orientation.y = arm.getCurrentPose().pose.orientation.y;
    goal.orientation.z = arm.getCurrentPose().pose.orientation.z;
    goal.orientation.w = arm.getCurrentPose().pose.orientation.z;
    goal.position.x = arm.getCurrentPose().pose.position.x + 0.05; 
    goal.position.y = arm.getCurrentPose().pose.position.y + 0.1; 
    goal.position.z = arm.getCurrentPose().pose.position.z - 0.2;

    //Set the target pose, which is the goal we already defined
    arm.setPoseTarget(goal);

    //Perform the planning step, display the current arm trajectory and move the arm
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
