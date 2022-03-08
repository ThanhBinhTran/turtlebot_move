// robot_move_lib.h
#ifndef ROBOT_MOVE_LIB_H // include guard

#define ROBOT_MOVE_LIB_H

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "robot_config.h"

namespace robot_base_driver
{
    class robot_move
    {
    public:
        ros::Publisher cmd_vel_pub;     // cmd_vel publisher
        geometry_msgs::Twist cmd_msg;   // cmd_vel geometry message
        
        // default constructor and deconstructor
        robot_move(ros::NodeHandle nh, std::string topic_name);
        ~robot_move();

        void reset_cmd_msg();   // reset cmd_vel geometry message
        void set_linear(float numx, float numy, float numz); // set linear for cmd_vel geometry message
        void set_angular(float numx, float numy, float numz); // set angular for cmd_vel geometry message

        void go_straight(float distance_in_m);  // make robot go straight (in meter); input: (+) forward, (-) backward
                                                
        void stop_move(); // stop moving
        void rotate(float radian);  // make robot rotate; input: (+) anticlockwise, (-) clockwise. 
    };

}    

#endif