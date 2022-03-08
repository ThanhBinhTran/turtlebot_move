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
        
        /*
         * constructor and deconstructor
         */
        
        robot_move(ros::NodeHandle nh, std::string topic_name);
        ~robot_move();

        /*
         * ROS and real robot functionality handler 
         */

        void do_publish(uint run_times, float topic_rate); // publish message to topic
        bool check_velocity_limitation(float linear_vel_x, float linear_vel_y, float linear_vel_z,
                float angular_vel_x, float angular_vel_y, float angular_vel_z); // return true if velocity is in limited vel ranges

        /*
         * message preparation 
         */

        void reset_cmd_msg();   // reset cmd_vel geometry message
        void set_linear(float numx, float numy, float numz); // set linear for cmd_vel geometry message
        void set_angular(float numx, float numy, float numz); // set angular for cmd_vel geometry message

        /* 
         * movement functions 
         */

        void stop_move(); // stop moving
        
        /* go straight (meter) in time_period second; distance_in_m = (+) forward, (-) backward */
        void go_straight(float distance_in_m, float time_period); 
        
        /* rotate (radian) in time_period second; radian = (+) anticlockwise, (-) clockwise. */
        void rotate(float radian, float time_period);  
        
        /* robot follows arc defined by theta and radian in time_period second.*/
        void arc(float theta, float radius, float time_period); 
    };

}    

#endif