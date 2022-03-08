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

  ROS_INFO_STREAM("Press ctrl + c to stop if loop is invoked");

  /*
  // scenario 1
  while(nh.ok()) { // press ctrl + c to break this loop. 
    turtlebot_move.go_straight(1.0);  // go straigth 1 meter backward
    turtlebot_move.rotate(PI/2); // rotate 90 degree
  }
  */

  if (nh.ok()){
    /* 
    // scenario 2
    turtlebot_move.go_straight(4, 5);  // go straigth 4 meter ahead in 4 second
    turtlebot_move.rotate(PI/2, 5); // rotate 90 degree in 4 second
    
    turtlebot_move.go_straight(4, 4);  
    turtlebot_move.rotate(PI/2, 4); 
    
    turtlebot_move.go_straight(4, 3);  
    turtlebot_move.rotate(PI/2, 3); 

    turtlebot_move.go_straight(4, 2); 
    turtlebot_move.rotate(PI/2, 2); 
    */
    
    // scenario 3
    turtlebot_move.go_straight(-2, 1); // go 1m backward in 1 second then folow a circle 
    turtlebot_move.arc(PI*2, 2, 4); // radius = 4m, time period = 2 second, theta = 2 pi (= circle)
  }


  // stop move
  turtlebot_move.stop_move();

  ROS_INFO("Finished\n");

  return 0;
}
