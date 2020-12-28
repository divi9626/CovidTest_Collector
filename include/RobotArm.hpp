#pragma once
// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include "fiducial_msgs/FiducialTransformArray.h"
#include <map>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> joint_control_client;
/**
 * Creates the Robot Arm class 
 */
class RobotArm {
    private:
    	ros::NodeHandle nh;

    public:
        RobotArm() : grip_ac("/gripper_controller/follow_joint_trajectory"){
            ROS_INFO("Waiting for action server to start.");
            grip_ac.waitForServer();
            ROS_INFO("Action server started");
            /**
             * Set the safe position of the arm to move the robot in and
             * the position to place the object in the collection area 
             */
            safePose.pose.position.x = 0.363;
            safePose.pose.position.y = -0.197;
            safePose.pose.position.z = 1.0;
            safePose.pose.orientation.x = 0.707;
            safePose.pose.orientation.y = 0;
            safePose.pose.orientation.z = 0;
            safePose.pose.orientation.w = 0.707;
            collectPose.pose.position.x = 0.5;
            collectPose.pose.position.y = -0.197;
            collectPose.pose.position.z = 0.9;
            collectPose.pose.orientation.x = 0.707;
            collectPose.pose.orientation.y = 0;
            collectPose.pose.orientation.z = 0;
            collectPose.pose.orientation.w = 0.707;
        }
        void moveArm(geometry_msgs::PoseStamped pose);
        geometry_msgs::PoseStamped safePose;
        geometry_msgs::PoseStamped collectPose;
        void grip();
        void release();
        joint_control_client grip_ac;
        control_msgs::FollowJointTrajectoryGoal grip_goal;


        
};
