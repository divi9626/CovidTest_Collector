#include "../include/RobotNavigation.hpp"


/**
 * @brief Move the robot to a designated position and orientation
 * @param x, the x map coordinate to move the robot to 
 * @param y, the x map coordinate to move the robot to 
 * @param w, the w orientation to position the robot  
 * @return none
 */
void RobotNavigation::sendGoal(double x, double y, double w){
    
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    /**
     * Set the header of the move_base action 
     */
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    /**
     * create a quaternion based on a given yaw
     */
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, w);
    /**
     * Set the goal position and orientation 
     */
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.x = myQuaternion[0];
    goal.target_pose.pose.orientation.y = myQuaternion[1];
    goal.target_pose.pose.orientation.z = myQuaternion[2];
    goal.target_pose.pose.orientation.w = myQuaternion[3];
    ROS_INFO_STREAM("Goal pos to go is " << goal.target_pose.pose.position.x << 
    	", " << goal.target_pose.pose.position.y <<
    	"\nGoal orientation to be in is " << goal.target_pose.pose.orientation.w);
    /**
     * Send the goal to the action server 
     */
    ac.sendGoal(goal);
    /**
     * Check to see if the goal was met 
     */
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, goal was met");
    else
      ROS_INFO("The base failed to move to the goal");

}

