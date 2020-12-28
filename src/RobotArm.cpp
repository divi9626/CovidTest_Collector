#include "../include/RobotArm.hpp"

/**
 * @brief Move the robot arm to a designated end effector position and orientation
 * @param goal_pose, the desired end effector pose
 * @return none
 */
void RobotArm::moveArm(geometry_msgs::PoseStamped goal_pose){

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  goal_pose.header.frame_id = "base_footprint";

  std::vector<std::string> torso_arm_joint_names;
  /**
   * choose the planning group arm_torso 
   */
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  /**
   * choose your preferred planner 
   */
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  /**
   * Set the frame to plan in 
   */
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  /**
   * Set the goal pose 
   */
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  /**
   * Set the maximum time to plan 
   */
  group_arm_torso.setPlanningTime(5.0);


  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  ros::Time start = ros::Time::now();
  /**
   * Execute the plan 
   */
  group_arm_torso.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  spinner.stop();
}
/**
 * @brief Uses the trajectory controller to close the gripper
 * @param none
 * @return none
 */
void RobotArm::grip(){
  /**
   * Set the joint names and set the joint states to 0 
   */
  grip_goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
  grip_goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
  grip_goal.trajectory.points.resize(1);
  grip_goal.trajectory.points[0].positions.resize(2);
  grip_goal.trajectory.points[0].positions[0] = 0.0;
  grip_goal.trajectory.points[0].positions[1] = 0.0;
  grip_goal.trajectory.points[0].time_from_start = ros::Duration(4.0);
  /**
   * Execute the grip 
   */
  grip_ac.sendGoal(grip_goal);
  /**
   * Check the result of the action 
   */
  grip_ac.waitForResult();

  if(grip_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, goal was met");
  else
    ROS_INFO("The base failed to move to the goal");

}

void RobotArm::release(){
  /**
   * Set the joint states from the grip method to 0.4 
   */
  grip_goal.trajectory.points[0].positions[0] = 0.4;
  grip_goal.trajectory.points[0].positions[1] = 0.4;
  grip_goal.trajectory.points[0].time_from_start = ros::Duration(4.0);
  /**
   * Execute the release 
   */
  grip_ac.sendGoal(grip_goal);
  grip_ac.waitForResult();
  /**
   * Check the result of the action 
   */
  if(grip_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, goal was met");
  else
    ROS_INFO("The base failed to move to the goal");

}