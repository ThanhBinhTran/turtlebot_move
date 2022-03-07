/*
 * Tested using TurtleBot 3, ROS Neotic, Ubuntu 20.04
 * email: thanhbinh.hcmut@gmail.com
 */


#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include "robot_config.h"
#include "robot_move_lib.h"

using namespace robot_base_driver;

int main(int argc, char** argv)
{
  //init the ROS node
  ROS_INFO_STREAM("Staring robot_driver_cmd");
  ros::init(argc, argv, "robot_move"); 
  ros::NodeHandle nh;
  
  // init robot move
  robot_move turtlebot_move(nh, "cmd_vel");

  ROS_INFO_STREAM("Press ctrl + c to stop");

  while(nh.ok()) { // press ctrl + c to break this loop.

    turtlebot_move.go_straight(-1.0);  // go straigth 1 meter backward
    turtlebot_move.rotate(PI/2); // rotate 90 degree
  }

  // stop move
  turtlebot_move.stop_move();

  ROS_INFO("Finished\n");

  return 0;
}
