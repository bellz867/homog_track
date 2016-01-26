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
	
	/********** desired wrt world **********/
	// generator parameters
	double desired_period = 4;//period of oscillation
	double desired_frequency_hz = 1/desired_period;//frequency in hertz
	double desired_frequency = 2*M_PIl*desired_frequency_hz;//frequency in radians/sec
	double desired_radius = 0.5;// desired radius for oscillations in meters
	
	double start_time = ros::Time::now().toSec();
	double time_from_start = 0;
	double desired_vel = 0;
	
	geometry_msgs::Twist desired_vel_msg;
	bool square_wave = true;
	
	ros::Rate loop_rate(300);
	
	while (ros::ok())
	{
		time_from_start = ros::Time::now().toSec() - start_time;//update elapsed time
		desired_vel = desired_radius*desired_frequency*std::cos(desired_frequency*time_from_start);//update desired wave expressed in world frame
		
		if (square_wave)//square wave
		{
			if (std::signbit(desired_vel))// if negative will output negative magnitude
			{
				desired_vel_msg.angular.z = -1*desired_radius*desired_frequency;
			}
			else// otherwise a positive magnitude
			{
				desired_vel_msg.angular.z = desired_radius*desired_frequency;
			}
		}
		else//sine wave
		{
			desired_vel_msg.angular.z = desired_vel;
		}
		
		desired_vel_msg.linear.x = 0;
		desired_vel_msg.linear.y = 0;
		desired_vel_msg.linear.z = 0;
		desired_vel_msg.angular.x = 0;
		desired_vel_msg.angular.y = 0;
		//desired_vel_msg.angular.z = 0;
		
		desired_pub.publish(desired_vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}




