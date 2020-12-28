# CovidTest_Collector
[![Build Status](https://travis-ci.org/divi9626/CovidTest_Collector.svg?branch=main)](https://travis-ci.org/divi9626/CovidTest_Collector)
[![License](https://img.shields.io/badge/license-MIT-green)](https://opensource.org/licenses/MIT)

## Overview
This ROS based project simulate the functionality of a robot that will locate a completed COVID test that has been placed on a table by a patient. Once the robot has located the test vial, it will pick it up, carry it over to a collection area, and safely place it in the collection area.

[![tiagovid gif](media/tiagovid.gif)]

## Dependencies
This program works on a device running Ubuntu 18.04 and ROS Melodic. It will also need Gazebo 9.0 and TIAGo robot packages. 

* To install ROS Melodic in Ubuntu 18.04, follow the steps in this [link](http://wiki.ros.org/melodic/Installation/Ubuntu).

* To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

## Install
* To install this project, first create a new ws:

```
mkdir ~/tiagovid_ws
cd ~/tiagovid_ws
```
Then copy the [rosinstall file](install/tiago_public-melodic.rosinstall) into the tiagovid_ws directory and run the following to install the necessary packages.

```
cd ~/tiagovid_ws
rosinstall src /opt/ros/melodic tiago_public-melodic.rosinstall
```
Finally, run the following to make sure all the dependencies referenced in the workspace are installed

```
rosdep install --from-paths src --ignore-src -y --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3"
```

## Build Instructions

To build this project, run the following:

```
source /opt/ros/melodic/setup.bash
catkin build -DCATKIN_ENABLE_TESTING=0
```
Note: This may take a while to build all the packages in the project.

## Running

* After completing the building step, run the following commands:
```
cd ~/tiagovid_ws
source devel/setup.bash
roslaunch Covid_Test_Collector tiagovid.launch
```
The above command will automatically start the demo.

## Personnel
* Divyam Garg, a roboticist working on his Masters at UMD.
* Loic Barret, a roboticist working on his Masters at UMD.

