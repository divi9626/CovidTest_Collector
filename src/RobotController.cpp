#include "../include/RobotNavigation.hpp"
#include "../include/RobotVision.hpp"
#include "../include/RobotArm.hpp"
#include <math.h>


int main(int argc, char **argv) {
  /**
   * initialize the ros node
   */
  ros::init(argc, argv, "tiagovid_controller");

  /**
   * create objects for the 3 classes used
   */
  RobotNavigation tiagovid_navigator;
  RobotVision tiagovid_vision;
  RobotArm tiagovid_arm;

  /**
   * define what rate we want robot to check for the object
   */
  ros::Rate loop_rate(10);
  ros::Duration(10).sleep();

  /**
   * Move the robot arm to a safe position to move around in
   */
  tiagovid_arm.moveArm(tiagovid_arm.safePose);

  /**
   * Move the robot into the other room
   */
  tiagovid_navigator.sendGoal(1.5, -4.0, -M_PI/2);
  ros::Duration(1).sleep();

  /**
   * This loop moves the robot around the map to look for the object
   */
  while(ros::ok()){

    /**
     * move to the first table
     */
    tiagovid_navigator.sendGoal(1.25, -7.0, M_PI);
    ros::Duration(1).sleep();
    tiagovid_navigator.sendGoal(0.5, -7.0, M_PI);
    ros::Duration(2).sleep();

    /**
     * Check for object until object is found or timeout
     */
    ROS_INFO("Checking for Object");
    int i = 0;
    while(ros::ok()) {
      
      loop_rate.sleep();
      ros::spinOnce();
      ++i;
      if(tiagovid_vision.seen || i > 20){
        break;
      }
    }
    if (tiagovid_vision.seen){
      std::cout << "found";
      break;
    }

    /**
     * move to the second table
     */
    tiagovid_navigator.sendGoal(1.25, -9.0, M_PI);
    ros::Duration(1).sleep();
    tiagovid_navigator.sendGoal(0.5, -9.0, M_PI);
    ros::Duration(2).sleep();

    /**
     * Check for object until object is found or timeout
     */
    ROS_INFO("Checking for Object");
    i = 0;
    while(ros::ok()) {
      
      loop_rate.sleep();
      ros::spinOnce();
      ++i;
      if(tiagovid_vision.seen || i > 20){
        break;
      }
    }
    if (tiagovid_vision.seen){
      std::cout << "found";
      break;
    }

    /**
     * move to the third table
     */
    tiagovid_navigator.sendGoal(1.25, -11.0, M_PI);
    ros::Duration(1).sleep();
    tiagovid_navigator.sendGoal(0.5, -11.0, M_PI);
    ros::Duration(2).sleep();

    /**
     * Check for object until object is found or timeout
     */
    ROS_INFO("Checking for Object");
    i = 0;
    while(ros::ok()) {
      loop_rate.sleep();
      ros::spinOnce();
      ++i;
      if(tiagovid_vision.seen || i > 20){
        break;
      }
    }
    if (tiagovid_vision.seen){
      std::cout << "found";
      break;
    }    
  }
  /**
   * once the object is found, set the pose of the robot arm
   * based on the marker location
   */
  tiagovid_vision.setPose();
  /**
   * Move the arm above the object
   */
  tiagovid_arm.moveArm(tiagovid_vision.object_above);
  /**
   * Move the arm down carefully until arm is able to grasp object
   */
  tiagovid_arm.moveArm(tiagovid_vision.object_in_grasp);
  /**
   * Gtip the object
   */
  tiagovid_arm.grip();
  /**
   * Move the arm back to the pose that is safe to move the robot in
   */
  tiagovid_arm.moveArm(tiagovid_arm.safePose);
  /**
   * Move the robot to the collection area
   */
  tiagovid_navigator.sendGoal(0.0, 0.0, M_PI);
  tiagovid_navigator.sendGoal(-2.5, .5, M_PI/2);
  /**
   * Move the arm to the position needed to drop the object into the 
   * collection area
   */
  tiagovid_arm.moveArm(tiagovid_arm.collectPose);
  /**
   * Release the object
   */
  tiagovid_arm.release();
  ros::Duration(1).sleep();
  /**
   * Move the arm back to the pose that is safe to move the robot in
   */
  tiagovid_arm.moveArm(tiagovid_arm.safePose);
  return 0;
}