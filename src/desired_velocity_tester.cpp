#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <algorithm>

// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>


// main
int main(int argc, char** argv)
{   
	// node parameters
	ros::init(argc,argv,"desired_velocity_generator");
	ros::NodeHandle nh;
	ros::Publisher desired_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_from_control",1);
	
	// generator parameters
	double desired_frequency = 0.5;
	double desired_mag = 0.5;
	double start_time = ros::Time::now().toSec();
	double time_from_start = 0;
	double desired_vel = 0;
	geometry_msgs::Twist desired_vel_msg;
	
	ros::Rate loop_rate(300);
	
	while (ros::ok())
	{
		time_from_start = ros::Time::now().toSec() - start_time;//update elapsed time
		desired_vel = desired_mag*std::sin(2*M_PIl*desired_frequency*time_from_start);//update desired velocitysine wave
		desired_vel_msg.linear.x = desired_vel;
		desired_vel_msg.linear.y = 0;
		desired_vel_msg.linear.z = 0;
		desired_vel_msg.angular.x = 0;
		desired_vel_msg.angular.y = 0;
		desired_vel_msg.angular.z = 0;
		desired_pub.publish(desired_vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}




