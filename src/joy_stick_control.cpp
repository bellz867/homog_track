#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <algorithm>

// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <homog_track/ImageProcessingMsg.h>
#include <homog_track/DecompMsg.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

class Joycall
{
	public:
	
		ros::NodeHandle nh;
		ros::Subscriber joy_sub, cmd_vel_sub;
		ros::Publisher takeoff_pub, land_pub, reset_pub, cmd_vel_pub;
		
		/********** output command choice and xbox controller **********/
		geometry_msgs::Twist command_from_xbox;
		geometry_msgs::Twist velocity_command;
		int a_button_land_b0 = 0, x_button_set_0_b2 = 0, b_button_reset_b1 = 0, y_button_takeoff_b3 = 0, lb_button_teleop_b4 = 0, rb_button_teleop_b5 = 0;
		double rt_stick_ud_x_a3 = 0, rt_stick_lr_y_a2 = 0, lt_stick_ud_z_a1 = 0, lt_stick_lr_th_a0 = 0;
		bool start_autonomous = false;
		bool send_0 = false;
		bool recieved_command = false;
		bool recieved_control = false;
		
		
		Joycall()
		{
			cmd_vel_sub = nh.subscribe("cmd_vel_from_control",1,&Joycall::cmd_vel_callback,this);
			cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);// publisher for the decomposed stuff
			takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff",1);
			land_pub = nh.advertise<std_msgs::Empty>("ardrone/land",1);
			reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset",1);
			joy_sub = nh.subscribe("joy",1,&Joycall::joy_callback,this);
			cmd_vel_pub.publish(geometry_msgs::Twist());// initially sending it a desired command of 0
		}
		
		/********** callback for the cmd velocity from the autonomy **********/
		void cmd_vel_callback(const geometry_msgs::Twist& msg)
		{
			velocity_command = msg;
			if (start_autonomous)
			{
				recieved_command = true;
			}
		}
		
		/********** callback for the controller **********/
		void joy_callback(const sensor_msgs::Joy& msg)
		{
			command_from_xbox = geometry_msgs::Twist();// cleaning the xbox twist message
			double joy_gain = 1;
			a_button_land_b0 = msg.buttons[0];// a button lands
			if (a_button_land_b0 > 0)
			{
				land_pub.publish(std_msgs::Empty());
			}
			x_button_set_0_b2 = msg.buttons[2];
			if (x_button_set_0_b2 > 0)
			{
				send_0 = true;
			}
			
			b_button_reset_b1 = msg.buttons[1];// b button resets
			if (b_button_reset_b1 > 0)
			{
				reset_pub.publish(std_msgs::Empty());
			}
			
			y_button_takeoff_b3 = msg.buttons[3];// y button takes off
			if (y_button_takeoff_b3 > 0)
			{
				takeoff_pub.publish(std_msgs::Empty());
			}
				
			lb_button_teleop_b4 = msg.buttons[4];// left bumper for autonomous mode
			rb_button_teleop_b5 = msg.buttons[5];// right bumper says to start controller

			
			// if the bumper is pulled and it is not in autonomous mode will put it in autonomous mode
			// if the bumper is pulled and it is in autonomous mode will take it out of autonomous mode
			start_autonomous = lb_button_teleop_b4 > 0;
			
			std::cout << "autonomous mode is: " << start_autonomous << std::endl;
			
			rt_stick_ud_x_a3 = joy_deadband(msg.axes[3]);// right thumbstick up and down controls linear x
			command_from_xbox.linear.x = joy_gain*rt_stick_ud_x_a3;
			
			rt_stick_lr_y_a2 = joy_deadband(msg.axes[2]);// right thumbstick left and right controls linear y
			command_from_xbox.linear.y = joy_gain*rt_stick_lr_y_a2;
			
			lt_stick_ud_z_a1 = joy_deadband(msg.axes[1]);// left thumbstick up and down controls linear z
			command_from_xbox.linear.z = joy_gain*lt_stick_ud_z_a1;
			
			lt_stick_lr_th_a0 = joy_deadband(msg.axes[0]);// left thumbstick left and right controls angular z
			command_from_xbox.angular.z = joy_gain*lt_stick_lr_th_a0;
			std::cout << "xbox callback" << std::endl;
			
			std::cout << "linear x: " << command_from_xbox.linear.x << std::endl;
			std::cout << "linear y: " << command_from_xbox.linear.y << std::endl;
			std::cout << "linear z: " << command_from_xbox.linear.z << std::endl;
			std::cout << "angular z: " << command_from_xbox.linear.z << std::endl;
			
			if (!start_autonomous)
			{
				if (send_0)
				{
					command_from_xbox = geometry_msgs::Twist();
					send_0 = false;
				}
				recieved_control = true;
			}
			
		}
				/********** deadband for the xbox controller **********/
		double joy_deadband(double input_value)
		{
			double filtered_value = 0;
			if (std::abs(input_value) > 0.15)
			{
				filtered_value = input_value;
			}
			return filtered_value;
		}
};
// main
int main(int argc, char** argv)
{   
	ros::init(argc,argv,"joy_stick_control_node");

	Joycall joycall;
	ros::Rate loop_rate(300);
	
	while (ros::ok())
	{
		
		if (joycall.recieved_command || joycall.recieved_control)
		{
			if (joycall.recieved_control)
			{
				joycall.cmd_vel_pub.publish(joycall.command_from_xbox);
				joycall.recieved_control = false;
			}
			if (joycall.recieved_command)
			{
				joycall.cmd_vel_pub.publish(joycall.velocity_command);
				joycall.recieved_command = false;
			}
			
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	//ro

    return 0;
}
