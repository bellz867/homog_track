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
		ros::Subscriber joy_sub, cmd_vel_sub, camera_state_sub, body_vel_sub;
		ros::Publisher takeoff_pub, land_pub, reset_pub, cmd_vel_pub, camera_state_pub;
		tf::TransformListener listener;
		
		/********** output command choice and xbox controller **********/
		geometry_msgs::Twist command_from_xbox;
		geometry_msgs::Twist velocity_command;
		geometry_msgs::Twist gimbal_state_current;
		geometry_msgs::Twist gimbal_state_desired;
		int a_button_land_b0 = 0, x_button_set_0_b2 = 0, b_button_reset_b1 = 0, y_button_takeoff_b3 = 0, lb_button_teleop_b4 = 0, rb_button_teleop_b5 = 0;
		double rt_stick_ud_x_a3 = 0, rt_stick_lr_y_a2 = 0, lt_stick_ud_z_a1 = 0, lt_stick_lr_th_a0 = 0, r_trigger = 1, dpad_u = 0, dpad_d = 0;
		bool start_autonomous = false;
		bool send_0 = false;
		bool recieved_command = false;
		bool recieved_control = false;
		bool first_run = true;
		double linear_max = 0.5;// max linear velocity
		double joy_gain = 0.5;// gain on xbox controller
		double x_bounds[2] = {-2.5, 2.0};// x world bounds meters
		double y_bounds[2] = {-1.75, 1.75};// y world bounds meters
		double z_bounds[2] = {0,3.0};// z position meters
		double bound_push_back_mag = 0.5;// push back magnitude is effort to push back if boundary is crossed
		tf::Quaternion bound_push_back_out_temp;// holds the quaternion result output after transforming
		tf::Quaternion bound_push_back_temp;// holds the quaternion form for the push back velocities so it can be transformed
		geometry_msgs::Twist bound_push_back_cmd;// push back twist message, to be created when the boundary is crossed
		tf::StampedTransform body_wrt_world;
		bool x_bounds_exceeded[2] = {false, false};// boolean to tell if the x boundary has been exceeded
		bool y_bounds_exceeded[2] = {false, false};// boolean to tell if the y boundary has been exceeded
		bool z_bounds_exceeded[2] = {false, false};// boolean to tell if the z boundary has been exceeded
		bool boundary_exceeded = false;// tells if any boundary exceeded
		double k1 = 1/7.0;// original 1/6.3
		double kp_s = 2.5;
		double kd_s = 0.075;
		tf::Matrix3x3 kp = tf::Matrix3x3(kp_s*1,0,0,0,kp_s*1,0,0,0,kp_s*1);
		tf::Matrix3x3 kd = tf::Matrix3x3(kd_s*1,0,0,0,kd_s*1,0,0,0,kd_s*1);
		geometry_msgs::Twist body_vel;// body velocity from the mocap
		ros::Time last_body_vel_time;// last time a body velocity was recieved
		ros::Time curr_body_vel_time;// current time for a recieved body velocity
		ros::Time start_body_vel_time;//start time for the body velocity
		tf::Vector3 error;//error for the pd controller
		tf::Vector3 last_error;//last error for the pd controller
		tf::Vector3 errorDot;//derivative of error for the pd controller
		bool first_body_vel = true;
		Joycall()
		{
			cmd_vel_sub = nh.subscribe("cmd_vel_from_control",1,&Joycall::cmd_vel_callback,this);
			cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);// publisher for the decomposed stuff
			takeoff_pub = nh.advertise<std_msgs::Empty>("bebop/takeoff",1);
			land_pub = nh.advertise<std_msgs::Empty>("bebop/land",1);
			reset_pub = nh.advertise<std_msgs::Empty>("bebop/reset",1);
			camera_state_sub = nh.subscribe("/bebop/camera_control",1,&Joycall::camera_state_callback,this);//get the current gimbal desired center
			camera_state_pub = nh.advertise<geometry_msgs::Twist>("/bebop/camera_control",1);// set the gimbal desired center
			joy_sub = nh.subscribe("joy",1,&Joycall::joy_callback,this);
			body_vel_sub = nh.subscribe("/bebop/body_vel",1,&Joycall::body_vel_callback,this);// callback for the body vel
			
			cmd_vel_pub.publish(geometry_msgs::Twist());// initially sending it a desired command of 0
			
			// testing showed that -55 is a pretty good starting angle
			gimbal_state_current.angular.y = -55;
		}
		
		/********** callback for the camera desired gimbal center **********/
		void camera_state_callback(const geometry_msgs::Twist& msg)
		{
			//angular y is tilt: + is down - is up
			//angular z is pan:+ is left - is right
			gimbal_state_current = msg;
			std::cout << "gimbal state: " << msg.angular.y << std::endl;
		}
		
		/********** callback for the cmd velocity from the autonomy **********/
		void cmd_vel_callback(const geometry_msgs::Twist& msg)
		{
			error.setValue(msg.linear.x - body_vel.linear.x, msg.linear.y - body_vel.linear.y, msg.linear.z - body_vel.linear.z);
			std::cout << "error x: " << error.getX() << " y: " << error.getY() << " z: " << error.getZ() << std::endl;
			
			// if some time has passed between the last body velocity time and the current body velocity time then will calculate the (feed forward PD)
			if (std::abs(curr_body_vel_time.toSec() - last_body_vel_time.toSec()) > 0.00001)
			{	
				errorDot = (1/(curr_body_vel_time - last_body_vel_time).toSec()) * (error - last_error);
				velocity_command.linear.x = k1*msg.linear.x + (kp*error).getX() + (kd*errorDot).getX();
				velocity_command.linear.y = k1*msg.linear.y + (kp*error).getY() + (kd*errorDot).getY();
				velocity_command.linear.z = k1*msg.linear.z + (kp*error).getZ() + (kd*errorDot).getZ();
				velocity_command.angular.z = -1*msg.angular.z;// z must be switched because bebop driver http://bebop-autonomy.readthedocs.org/en/latest/piloting.html
				std::cout << "velocity command linear x: " << velocity_command.linear.x << " y: " << velocity_command.linear.y << " z: " << velocity_command.linear.z << std::endl;
			}
			
			last_body_vel_time = curr_body_vel_time;// update last time body velocity was recieved
			last_error = error;
			
			if (start_autonomous)
			{
				recieved_command = true;
			}
		}
		/********** callback for the body velocity **********/
		void body_vel_callback(const geometry_msgs::TwistStamped& msg)
		{
			body_vel = msg.twist;
			curr_body_vel_time  = msg.header.stamp;
			if (first_body_vel)
			{
				start_body_vel_time = curr_body_vel_time;
				first_body_vel = false;
			}
		}
		
		/********** callback for the controller **********/
		void joy_callback(const sensor_msgs::Joy& msg)
		{
			command_from_xbox = geometry_msgs::Twist();// cleaning the xbox twist message
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
				
			r_trigger = msg.axes[5];//right trigger pulled for fine tilt control of gimbal
			dpad_u = msg.buttons[13];//dpad up value
			dpad_d = msg.buttons[14];//dpad down value
			// if the dpad is close to 1, means want to tilt camera down so add to current camera value
			// if the dpad is close to -1, means want to tilt camera up so add to current camera value
			// if the right trigger is less than 0, means want to finely adjust tilt
			if (std::abs(dpad_u) > 0.5 || std::abs(dpad_d) > 0.5)
			{
				std::cout << "here 1" << std::endl;
				if (std::abs(dpad_u) > 0.5 && r_trigger < 0)
				{
					std::cout << "here 2" << std::endl;
					gimbal_state_desired.angular.y = gimbal_state_current.angular.y + 1;
				}
				if (std::abs(dpad_u) > 0.5 && r_trigger > 0)
				{
					std::cout << "here 3" << std::endl;
					gimbal_state_desired.angular.y = gimbal_state_current.angular.y + 10;
				}
				if (std::abs(dpad_d) > 0.5 && r_trigger < 0)
				{
					std::cout << "here 4" << std::endl;
					gimbal_state_desired.angular.y = gimbal_state_current.angular.y - 1;
				}
				if (std::abs(dpad_d) > 0.5 && r_trigger > 0)
				{
					std::cout << "here 5" << std::endl;
					gimbal_state_desired.angular.y = gimbal_state_current.angular.y - 10;
				}
				
				camera_state_pub.publish(gimbal_state_desired);
			}
			if (first_run)
			{
				camera_state_pub.publish(gimbal_state_current);
				first_run = false;
				std::cout << "gimbal value: " << gimbal_state_current.angular.y << std::endl;
			}
			
			lb_button_teleop_b4 = msg.buttons[4];// left bumper for autonomous mode
			rb_button_teleop_b5 = msg.buttons[5];// right bumper says to start controller
			
			// if the bumper is pulled and it is not in autonomous mode will put it in autonomous mode
			// if the bumper is pulled and it is in autonomous mode will take it out of autonomous mode
			start_autonomous = lb_button_teleop_b4 > 0;
			
			std::cout << "autonomous mode is: " << start_autonomous << std::endl;
			
			rt_stick_ud_x_a3 = joy_deadband(msg.axes[4]);// right thumbstick up and down controls linear x
			command_from_xbox.linear.x = joy_gain*rt_stick_ud_x_a3;
			
			rt_stick_lr_y_a2 = joy_deadband(msg.axes[3]);// right thumbstick left and right controls linear y
			command_from_xbox.linear.y = joy_gain*rt_stick_lr_y_a2;
			
			lt_stick_ud_z_a1 = joy_deadband(msg.axes[1]);// left thumbstick up and down controls linear z
			command_from_xbox.linear.z = joy_gain*lt_stick_ud_z_a1;
			
			lt_stick_lr_th_a0 = joy_deadband(msg.axes[0]);// left thumbstick left and right controls angular z
			command_from_xbox.angular.z = -1*joy_gain*lt_stick_lr_th_a0;
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
		
		double cap_linear_auto(double input_value)
		{
			double filtered_value = 0;
			if (std::abs(input_value) > linear_max)
			{
				if (input_value >= 0)
				{
					filtered_value = linear_max;
				}
				else
				{
					filtered_value = -1*linear_max;
				}
			}
			else
			{
				filtered_value = input_value;
			}
			
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
		try
		{
			// getting the body in world frame
			joycall.listener.waitForTransform("world", "bebop", ros::Time(0), ros::Duration(1.0));
			joycall.listener.lookupTransform("world", "bebop", ros::Time(0), joycall.body_wrt_world);
		}
		catch (tf::TransformException ex)
		{
			std::cout << "faild to get body wrt world in joycall" << std::endl;
		}
		
		
		// check if upper x boundary exceeded
		if (joycall.body_wrt_world.getOrigin().getX() >= joycall.x_bounds[1])
		{
			joycall.bound_push_back_temp.setX(-1*joycall.bound_push_back_mag);
			joycall.x_bounds_exceeded[1] = true;
		}
		
		// check if upper x lower boundary exceeded
		if ((joycall.body_wrt_world.getOrigin().getX() <= joycall.x_bounds[0]) && !joycall.x_bounds_exceeded[1])
		{
			joycall.bound_push_back_temp.setX(joycall.bound_push_back_mag);
			joycall.x_bounds_exceeded[0] = true;
		}
		
		// check if upper y boundary exceeded
		if (joycall.body_wrt_world.getOrigin().getY() >= joycall.y_bounds[1])
		{
			joycall.bound_push_back_temp.setY(-1*joycall.bound_push_back_mag);
			joycall.y_bounds_exceeded[1] = true;
		}
		
		// check if upper y lower boundary exceeded
		if ((joycall.body_wrt_world.getOrigin().getY() <= joycall.y_bounds[0]) && !joycall.y_bounds_exceeded[1])
		{
			joycall.bound_push_back_temp.setY(joycall.bound_push_back_mag);
			joycall.y_bounds_exceeded[0] = true;
		}
		
		// check if upper z boundary exceeded
		if (joycall.body_wrt_world.getOrigin().getZ() >= joycall.z_bounds[1])
		{
			joycall.bound_push_back_temp.setZ(-1*joycall.bound_push_back_mag);
			joycall.z_bounds_exceeded[1] = true;
		}
		
		joycall.boundary_exceeded = joycall.x_bounds_exceeded[0] || joycall.x_bounds_exceeded[1] || joycall.y_bounds_exceeded[0] || joycall.y_bounds_exceeded[1] || joycall.z_bounds_exceeded[0] || joycall.z_bounds_exceeded[1];
		
		// if boundary exceeded will transform to body frame and set command
		if (joycall.boundary_exceeded)
		{
			// transform velocity to body frame and send zero rotation
			joycall.bound_push_back_out_temp = (joycall.body_wrt_world.getRotation().inverse()*joycall.bound_push_back_temp)*joycall.body_wrt_world.getRotation();
			joycall.bound_push_back_cmd.linear.x = joycall.bound_push_back_out_temp.getX(); joycall.bound_push_back_cmd.linear.y = joycall.bound_push_back_out_temp.getY(); joycall.bound_push_back_cmd.linear.z = joycall.bound_push_back_out_temp.getZ();
			joycall.bound_push_back_cmd.angular.x = 0; joycall.bound_push_back_cmd.angular.y = 0; joycall.bound_push_back_cmd.angular.z = 0;
		}
		
		if ((joycall.recieved_command || joycall.recieved_control) && !joycall.boundary_exceeded)
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
		
		if (joycall.boundary_exceeded)
		{
			joycall.cmd_vel_pub.publish(joycall.bound_push_back_cmd);
			// reset stuff
			joycall.boundary_exceeded = false;
			joycall.recieved_control = false;
			joycall.recieved_command = false;
			joycall.bound_push_back_out_temp = tf::Quaternion(0,0,0,0);
			joycall.bound_push_back_temp = tf::Quaternion(0,0,0,0);
			joycall.bound_push_back_cmd.linear.x = 0; joycall.bound_push_back_cmd.linear.y = 0; joycall.bound_push_back_cmd.linear.z = 0;
			joycall.bound_push_back_cmd.angular.x = 0; joycall.bound_push_back_cmd.angular.y = 0; joycall.bound_push_back_cmd.angular.z = 0;
			joycall.x_bounds_exceeded[0] = false;
			joycall.x_bounds_exceeded[1] = false;
			joycall.y_bounds_exceeded[0] = false;
			joycall.y_bounds_exceeded[1] = false;
			joycall.z_bounds_exceeded[0] = false;
			joycall.z_bounds_exceeded[1] = false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
