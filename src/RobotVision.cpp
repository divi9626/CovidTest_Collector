// user defined header files for control class
#include "../include/RobotVision.hpp"


/**
 * @brief Robot Vision constructor which runs the initialization
 * @param none
 * @return none
 */
RobotVision::RobotVision() {
  init();
}

/**
 * @brief initializer to initialize the subscriber
 * @param none
 * @return none
 */
void RobotVision::init() {
  scan_sub = nh.subscribe("/fiducial_transforms", 1, &RobotVision::scanCallback, this);
}


/**
 * @brief callback that reads marker pose 
 * @param &msg the message being transcribed on the fiducial_transforms topic
 * @return none
 */
void RobotVision::scanCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg) {
  /**
   * Store the transform from the object frame to the camera frame
   */
  if (msg->transforms.size() > 0){
    ft = msg->transforms[0];
    seen = true;

  }

}

/**
 * @brief Sets the pose of the end effector based on pose of object
 * @param none
 * @return none
 */
void RobotVision::setPose(){
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  geometry_msgs::TransformStamped camera_to_base_link;
  /**
   * Look up the transform from the camera frame to the base_footprint frame
   */
  camera_to_base_link= tf_buffer.lookupTransform("base_footprint", "xtion_rgb_optical_frame", ros::Time(0), ros::Duration(1.0) );
  /**
   * Create a pose of the object in the camera frame
   */
  object.pose.position.x = ft.transform.translation.x;
  object.pose.position.y = ft.transform.translation.y;
  object.pose.position.z  = ft.transform.translation.z;
  /**
   * Transform the pose from the camera frame to the base_footprint frame 
   */
  tf2::doTransform(object.pose, object.pose, camera_to_base_link);
  /**
   * Create a pose above the object
   */
  object_above.pose.position.x = object.pose.position.x - 0.1;
  object_above.pose.position.y = object.pose.position.y;
  object_above.pose.position.z = object.pose.position.z + 0.3;
  object_above.pose.orientation.x = 0.707;
  object_above.pose.orientation.y = 0;
  object_above.pose.orientation.z = 0;
  object_above.pose.orientation.w = 0.707; 

  /**
   * Create a pose to grip the object
   */
  object_in_grasp.pose.position.x = object.pose.position.x - 0.1;
  object_in_grasp.pose.position.y = object.pose.position.y;
  object_in_grasp.pose.position.z = object.pose.position.z;
  object_in_grasp.pose.orientation.x = 0.707;
  object_in_grasp.pose.orientation.y = 0;
  object_in_grasp.pose.orientation.z = 0;
  object_in_grasp.pose.orientation.w = 0.707;
}

