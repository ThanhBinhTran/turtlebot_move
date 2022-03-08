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

void robot_move::do_publish(uint run_times, float topic_rate) // publish message to topic 
{
    ros::Rate rate(topic_rate);
    // do publishing
    for(int n=run_times; n>0; n--) {
        cmd_vel_pub.publish(cmd_msg);
        rate.sleep();
    }
}

/* message preparation */
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

/* movement functions */

void robot_move::stop_move()
{
    ROS_INFO("Entering .... stop_move function");
    reset_cmd_msg();
    
    // do publishing
    cmd_vel_pub.publish(cmd_msg);
}

void robot_move::go_straight(float distance_in_m, float time_period) // go straight in time_period second
{
    ROS_INFO("Entering .... go_straight function: distance = %f in %f second", distance_in_m, time_period);
    /* time_period = run_times / topic rate
       vel_x * time_period = distance_in_m   */

    float topic_rate = RUN_TIMES/time_period;
    float vel_x = distance_in_m/time_period;
    reset_cmd_msg();
    set_linear(vel_x, 0, 0);

    // do publishing
    do_publish(RUN_TIMES, topic_rate);
}

void robot_move::rotate(float radian, float time_period) // rotate radian in time_period second
{
    ROS_INFO("Entering .... rotate function: radian = %f in %f second", radian, time_period);
    /* Time_period = run_times / topic rate
       angular_z * time_period = radian; where angula_z is radian per second 
    */

    float topic_rate = RUN_TIMES/time_period;
    float angular_z = radian/time_period;

    reset_cmd_msg();
    set_angular(0, 0, angular_z);

    // do publishing
    do_publish(RUN_TIMES, topic_rate);
}

void robot_move::arc(float theta, float radius, float time_period) // robot follows arc defined by theta and radian in time_period second
{
    ROS_INFO("Entering .... arc function: theta = %f, radius=%f", theta, radius);
    /* Time_period = run_times / topic rate
       the angular velocity = theta / time period
       The circular velocity =  angular velocity  * radius */

    float topic_rate = RUN_TIMES/time_period;
    float angular_z = theta/time_period;
    float vel_x = angular_z * radius;

    // set cmd_vel message
    set_linear(vel_x, 0, 0);
    set_angular(0, 0, angular_z);

    // do publishing
    do_publish(RUN_TIMES, topic_rate);
}