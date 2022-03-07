#include "robot_move_lib.h" // header in local directory
#include <iostream> // header in standard library
#include "robot_config.h"
#include <geometry_msgs/Twist.h>


using namespace robot_base_driver;

robot_move::robot_move(ros::NodeHandle nh, std::string topic_name)
{
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(topic_name, 1);
}

robot_move::~robot_move()
{

}

void robot_move::reset_cmd_msg()
{
    set_linear(0,0,0);
    set_angular(0,0,0);
    
}


void robot_move::set_linear(float numx, float numy, float numz)
{
    cmd_msg.linear.x = numx;
    cmd_msg.linear.y = numy;
    cmd_msg.linear.z = numz;
}

void robot_move::set_angular(float numx, float numy, float numz)
{
    cmd_msg.angular.x = numx;
    cmd_msg.angular.y = numy;
    cmd_msg.angular.z = numz;
}

void robot_move::go_straight(float distance_in_m)
{
    ROS_INFO("Entering .... go_straight function: distance in m = %f", distance_in_m);
    // vel_x * run_times/ TOPIC_RATE = distance_in_m
    float vel_x = distance_in_m*TOPIC_RATE/RUN_TIMES;
    reset_cmd_msg();
    set_linear(vel_x, 0, 0);

    ros::Rate rate(TOPIC_RATE);
    // do publishing
    for(int n=RUN_TIMES; n>0; n--) {
        cmd_vel_pub.publish(cmd_msg);
        rate.sleep();
    }
}

void robot_move::rotate(float radian)
{
    ROS_INFO("Entering .... rotate function: radian = %f", radian);
    // angular_z * run_times/ TOPIC_RATE = radian
    // angulaz is radian per second
    float angular_z = radian*TOPIC_RATE/RUN_TIMES;

    reset_cmd_msg();
    set_angular(0, 0, angular_z);
    
    ros::Rate rate(TOPIC_RATE);
    // do publishing
    for(int n=RUN_TIMES; n>0; n--) {
        cmd_vel_pub.publish(cmd_msg);
        rate.sleep();
    }

}

void robot_move::stop_move()
{
    ROS_INFO("Entering .... stop_move function");
    reset_cmd_msg();
    cmd_vel_pub.publish(cmd_msg);
}
