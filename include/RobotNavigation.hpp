#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
/**
 * Creates the Robot Navigation class
 */
class RobotNavigation {
    private:
    	ros::NodeHandle nh;
        ros::Publisher goal_pub;
        move_base_msgs::MoveBaseGoal goal;
        MoveBaseClient ac;

    public:
        RobotNavigation() : ac("move_base", true){
        	ROS_INFO("Waiting for action server to start.");
            ac.waitForServer();
            ROS_INFO("Action server started");
        }
        void sendGoal(double x, double y, double w);
        

};