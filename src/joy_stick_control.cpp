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
#include <bebop_msgs/Ardrone3CameraStateOrientation.h>

class Joycall
{
	public:
		ros::NodeHandle nh;
		ros::Subscriber joy_sub, cmd_vel_sub, camera_state_sub, body_vel_sub, gimbal_sub;
		ros::Publisher takeoff_pub, land_pub, reset_pub, cmd_vel_pub, camera_state_pub, error_pub, errorDot_pub;
		ros::Timer watchdogTimer;
		tf::TransformListener listener;
		
		geometry_msgs::Twist error_gm, errorDot_gm;
		
		/********** output command choice and xbox controller **********/
		geometry_msgs::Twist command_from_xbox;
		geometry_msgs::Twist velocity_command;
		geometry_msgs::Twist command_out;
		
		geometry_msgs::Twist gimbal_state_current;
		geometry_msgs::Twist gimbal_state_desired;
		int a_button = 0, x_button = 0, b_button = 0, y_button = 0, left_bumper = 0, right_bumper = 0, start_button = 0, dpad_l = 0, dpad_r = 0, dpad_u = 0, dpad_d = 0;
		double right_stick_vert = 0, right_stick_horz = 0, left_stick_vert = 0, left_stick_horz = 0;
		bool start_autonomous = false;
		bool send_0 = false;
		bool recieved_command_from_tracking = false;
		bool recieved_command_from_xbox = false;
		bool first_run = true;
		
		double max_vel = 0.4;// max velocity
		
		double joy_gain = 1.0;// gain on xbox controller
		double step_gain = 0.0;
		
		double x_bounds[2] = {-1.5, 1.5};// x world bounds meters
		double y_bounds[2] = {-1.25, 1.25};// y world bounds meters
		double z_bounds[2] = {0,2.5};// z position meters
		
		double bound_push_back_mag = 0.5;// push back magnitude is effort to push back if boundary is crossed
		
		tf::Quaternion bound_push_back_out_temp;// holds the quaternion result output after transforming
		tf::Quaternion bound_push_back_temp;// holds the quaternion form for the push back velocities so it can be transformed
		geometry_msgs::Twist bound_push_back_cmd;// push back twist message, to be created when the boundary is crossed
		tf::StampedTransform body_wrt_world;
		bool x_bounds_exceeded[2] = {false, false};// boolean to tell if the x boundary has been exceeded
		bool y_bounds_exceeded[2] = {false, false};// boolean to tell if the y boundary has been exceeded
		bool z_bounds_exceeded[2] = {false, false};// boolean to tell if the z boundary has been exceeded
		bool boundary_exceeded = false;// tells if any boundary exceeded
		double kx = 1.0/17.46;// x map
		double ky = 1.0/13.75;// y map
		double kz = 1.0/1.02;// z map
		double kyaw = 1.0/1.72;// yaw map 

		double kp_yaw = 1;//original 2.5
		double kd_yaw = 0.005;//original 0.075
		
		tf::Matrix3x3 kp = tf::Matrix3x3(1.75, 0, 0,
										 0, 1.75, 0,
										 0, 0, 1);
		tf::Matrix3x3 kd = tf::Matrix3x3(0.01, 0, 0,
										 0, 0.01, 0,
										 0, 0, 0.01);
		
		geometry_msgs::Twist body_vel;// body velocity from the mocap
		ros::Time last_body_vel_time;// last time a body velocity was recieved
		ros::Time curr_body_vel_time;// current time for a recieved body velocity
		ros::Time start_body_vel_time;//start time for the body velocity
		tf::Vector3 error;//error for the pd controller
		tf::Vector3 last_error;//last error for the pd controller
		tf::Vector3 errorDot;//derivative of error for the pd controller
		double error_yaw = 0;
		double errorDot_yaw = 0;
		double last_error_yaw;
		bool first_body_vel = true;
		
		double wait_time = 1;//wait time of 1 second for the watchdog timer
		
		Joycall(double step_gain_des)
		{
			step_gain = step_gain_des;
			cmd_vel_sub = nh.subscribe("cmd_vel_from_control",1,&Joycall::cmd_vel_callback,this);
			cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);// publisher for the decomposed stuff
			takeoff_pub = nh.advertise<std_msgs::Empty>("bebop/takeoff",1);
			land_pub = nh.advertise<std_msgs::Empty>("bebop/land",1);
			reset_pub = nh.advertise<std_msgs::Empty>("bebop/reset",1);
			camera_state_pub = nh.advertise<geometry_msgs::Twist>("/bebop/camera_control",1);// set the gimbal desired center
			joy_sub = nh.subscribe("joy",1,&Joycall::joy_callback,this);
			body_vel_sub = nh.subscribe("/bebop/body_vel",1,&Joycall::body_vel_callback,this);// callback for the body vel
			gimbal_sub = nh.subscribe("bebop/states/ARDrone3/CameraState/Orientation",1,&Joycall::gimbal_state_callback,this);
            watchdogTimer = nh.createTimer(ros::Duration(wait_time),&Joycall::timeout,this,false);// Initialize watchdog timer
            error_pub = nh.advertise<geometry_msgs::Twist>("error_twist",1);
            errorDot_pub = nh.advertise<geometry_msgs::Twist>("errorDot_twist",1);
			
			cmd_vel_pub.publish(geometry_msgs::Twist());// initially sending it a desired command of 0
		}
		
		/********** callback for the watchdog timer **********/
		void timeout(const ros::TimerEvent& event)
	    {
	        cmd_vel_pub.publish(geometry_msgs::Twist());// publish 0s
	        std::cout << "\ntimer callback" << std::endl;
	    }
		
		/********** callback for the gimbal **********/
		void gimbal_state_callback(const bebop_msgs::Ardrone3CameraStateOrientation::ConstPtr& msg)
		{
			double pan = msg->pan;
			double tilt = msg->tilt;
			gimbal_state_current.angular.y = tilt;
			gimbal_state_current.angular.z = pan;
			std::cout << "\ngimbal tilt angle: " << gimbal_state_current.angular.y << " pan angle: " << gimbal_state_current.angular.z << std::endl;
		}
		
		/********** callback for the cmd velocity from the autonomy **********/
		void cmd_vel_callback(const geometry_msgs::Twist& msg)
		{
			watchdogTimer.stop();
			
			error.setValue(msg.linear.x - body_vel.linear.x, msg.linear.y - body_vel.linear.y, msg.linear.z - body_vel.linear.z);
			//std::cout << "error x: " << error.getX() << " y: " << error.getY() << " z: " << error.getZ() << std::endl;
			//std::cout << std::abs(curr_body_vel_time.toSec() - last_body_vel_time.toSec()) << std::endl;
			error_yaw = msg.angular.z - body_vel.angular.z;
			//std::cout << "error yaw: " << error_yaw << std::endl;
			
			// if some time has passed between the last body velocity time and the current body velocity time then will calculate the (feed forward PD)
			if (std::abs(curr_body_vel_time.toSec() - last_body_vel_time.toSec()) > 0.00001)
			{	
				errorDot = (1/(curr_body_vel_time - last_body_vel_time).toSec()) * (error - last_error);
				//std::cout << "errordot x: " << errorDot.getX() << " y: " << errorDot.getY() << " z: " << errorDot.getZ() << std::endl;
				
				errorDot_yaw = (1/(curr_body_vel_time - last_body_vel_time).toSec()) * (error_yaw - last_error_yaw);
				//std::cout << "error dot yaw " << errorDot_yaw << std::endl;
				velocity_command.linear.x = cap_vel_auton(kx*msg.linear.x + (kp*error).getX() + (kd*errorDot).getX());
				velocity_command.linear.y = cap_vel_auton(ky*msg.linear.y + (kp*error).getY() + (kd*errorDot).getY());
				velocity_command.linear.z = cap_vel_auton(kz*msg.linear.z + (kp*error).getZ() + (kd*errorDot).getZ());
				velocity_command.angular.z = -1*cap_vel_auton(kyaw*msg.angular.z + kp_yaw*error_yaw + kd_yaw*errorDot_yaw); // z must be switched because bebop driver http://bebop-autonomy.readthedocs.org/en/latest/piloting.html
			}
			
			last_body_vel_time = curr_body_vel_time;// update last time body velocity was recieved
			last_error = error;
			last_error_yaw = error_yaw;
			
			error_gm.linear.x = error.getX(); error_gm.linear.y = error.getY(); error_gm.linear.z = error.getZ(); error_gm.angular.z = error_yaw;
			errorDot_gm.linear.x = errorDot.getX(); errorDot_gm.linear.y = errorDot.getY(); errorDot_gm.linear.z = errorDot.getZ(); errorDot_gm.angular.z = kyaw*msg.angular.z + kp_yaw*error_yaw + kd_yaw*errorDot_yaw;
			error_pub.publish(error_gm);
			errorDot_pub.publish(errorDot_gm);
			
			if (start_autonomous)
			{
				recieved_command_from_tracking = true;
			}
			
			watchdogTimer.start();
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
			watchdogTimer.stop();
			
			command_from_xbox = geometry_msgs::Twist();// cleaning the xbox twist message
			a_button = msg.buttons[0];// a button lands
			if (a_button > 0)
			{
				land_pub.publish(std_msgs::Empty());
			}
			x_button = msg.buttons[2];// x button sends constant
			if (x_button > 0)
			{
				send_0 = true;
			}
			
			b_button = msg.buttons[1];// b button kills motors
			if (b_button > 0)
			{
				reset_pub.publish(std_msgs::Empty());
			}
			
			y_button = msg.buttons[3];// y button takes off
			if (y_button > 0)
			{
				takeoff_pub.publish(std_msgs::Empty());
			}
				
			dpad_l = msg.buttons[11];//dpad left value, fine adjustment of tilt angle negatively
			dpad_r = msg.buttons[12];//dpad right value, fine adjustment of tilt angle positively
			dpad_u = msg.buttons[13];//dpad up value, coarse adjustment of tilt angle positively
			dpad_d = msg.buttons[14];//dpad down value, coarse adjustment of tilt angle negatively
			
			int tilt_neg = -1*dpad_l - 10*dpad_d;// tilt negatively
			int tilt_pos = 1*dpad_r + 10*dpad_u;// tilt positively
			gimbal_state_desired.angular.y = gimbal_state_desired.angular.y + tilt_neg + tilt_pos;// adjust the gimbal
			camera_state_pub.publish(gimbal_state_desired);// update the angle
			
			left_bumper = msg.buttons[4];// left bumper for using the tracking as the controller output
			right_bumper = msg.buttons[5];// right bumper for using the mocap as the controller output
			
			start_autonomous = (left_bumper > 0) || (right_bumper > 0);// start the autonomous if either are greater than 0
			
			right_stick_vert = joy_deadband(msg.axes[4]);// right thumbstick vert controls linear x
			command_from_xbox.linear.x = joy_gain*right_stick_vert;
			
			right_stick_horz = joy_deadband(msg.axes[3]);// right thumbstick horz controls linear y
			command_from_xbox.linear.y = joy_gain*right_stick_horz;
			
			left_stick_vert = joy_deadband(msg.axes[1]);// left thumbstick vert controls linear z
			command_from_xbox.linear.z = joy_gain*left_stick_vert;
			
			left_stick_horz = joy_deadband(msg.axes[0]);// left thumbstick horz controls angular z
			command_from_xbox.angular.z = -1*joy_gain*left_stick_horz;
			
			if (!start_autonomous)// if not using the autonomous velocities as output will indicate recieved a new control input
			{
				if (send_0)
				{
					//command_from_xbox.linear.x = 0;
					//command_from_xbox.linear.y = 0;
					//command_from_xbox.linear.z = 0;
					//command_from_xbox.angular.z = 0;
					send_0 = false;
				}
				recieved_command_from_xbox = true;
			}
			
			watchdogTimer.start();
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
		
		/********** saturation for tracking controller commands **********/
		double cap_vel_auton(double input_value)
		{
			double filtered_value = input_value;
			if (std::abs(input_value) > max_vel)
			{
				if (input_value >= 0)
				{
					filtered_value = max_vel;
				}
				else
				{
					filtered_value = -1.0*max_vel;
				}
			}
			return filtered_value;
		}
};
// main
int main(int argc, char** argv)
{   
	ros::init(argc,argv,"joy_stick_control_node");
	
	bool write_to_file = false;
	double step_gain_ = 1.0;
	std::string output_file_name = "/home/ncr/ncr_ws/src/homog_track/testing_files/yawcommand_map_1_0.txt";
	std::fstream output_file;
	
	
	if( (std::remove( output_file_name.c_str() ) != 0) && write_to_file)
	{
		std::cout << "file does not exist" << std::endl;
	}
	else
	{
		std::cout << "file deleted" << std::endl;
	}
	
	/******************** Writing headers to file ********************/
	if (write_to_file)
	{
		output_file.open(output_file_name, std::fstream::out | std::fstream::app);
	
		if (output_file.is_open())
		{
			output_file << "time,"
						<< "x button,"
						<< "linear vel x," << "linear vel y," << "linear vel z,"
						<< "linear cmd x," << "linear cmd y," << "linear cmd z,"
						<< "angular cmd z,"
						<< "angular vel z"
						<< "\n";
			output_file.close();
		}
	}
	
	Joycall joycall(step_gain_);
	ros::Rate loop_rate(300);
	
	ros::Time start_time = ros::Time::now();
	
	while (ros::ok())
	{
		bool send_command = false;
		
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
		
		if ((joycall.recieved_command_from_tracking || joycall.recieved_command_from_xbox) && !joycall.boundary_exceeded)// if command from xbox or controller and bound not exceeded
		{
			if (joycall.recieved_command_from_xbox)
			{
				//joycall.cmd_vel_pub.publish(joycall.command_from_xbox);// publish command;
				joycall.command_out = joycall.command_from_xbox;
				//joycall.command_from_xbox = geometry_msgs::Twist();
				joycall.recieved_command_from_xbox = false;
				send_command = true;
				std::cout << "\ncommand from xbox" << std::endl;
				std::cout << "linear x: " << joycall.command_out.linear.x << std::endl;
				std::cout << "linear y: " << joycall.command_out.linear.y << std::endl;
				std::cout << "linear z: " << joycall.command_out.linear.z << std::endl;
				std::cout << "angular z: " << joycall.command_out.angular.z << std::endl;
			}
			
			if (joycall.recieved_command_from_tracking)
			{
				//joycall.cmd_vel_pub.publish(joycall.velocity_command);// publish command;
				joycall.command_out = joycall.velocity_command;
				//joycall.velocity_command = geometry_msgs::Twist();
				joycall.recieved_command_from_tracking = false;
				send_command = true;
				std::cout << "\ncommand from controller" << std::endl;
				std::cout << "linear x: " << joycall.command_out.linear.x << std::endl;
				std::cout << "linear y: " << joycall.command_out.linear.y << std::endl;
				std::cout << "linear z: " << joycall.command_out.linear.z << std::endl;
				std::cout << "angular z: " << joycall.command_out.angular.z << std::endl;
			}
			
		}
		
		if (joycall.boundary_exceeded)// set command to boundary command if boundary exceeded
		{
			//joycall.cmd_vel_pub.publish(joycall.bound_push_back_cmd);// publish command;
			joycall.command_out = joycall.bound_push_back_cmd;// output is the boundary command
			// reset stuff
			std::cout << "\nboundary exceeded" << std::endl;
			std::cout << "linear x: " << joycall.command_out.linear.x << std::endl;
			std::cout << "linear y: " << joycall.command_out.linear.y << std::endl;
			std::cout << "linear z: " << joycall.command_out.linear.z << std::endl;
			std::cout << "angular z: " << joycall.command_out.angular.z << std::endl;
			joycall.boundary_exceeded = false;
			joycall.recieved_command_from_xbox = false;
			joycall.recieved_command_from_tracking = false;
			send_command = true;
			joycall.bound_push_back_out_temp = tf::Quaternion(0,0,0,0);
			joycall.bound_push_back_temp = tf::Quaternion(0,0,0,0);
			joycall.bound_push_back_cmd = geometry_msgs::Twist();
			//joycall.bound_push_back_cmd.linear.x = 0; joycall.bound_push_back_cmd.linear.y = 0; joycall.bound_push_back_cmd.linear.z = 0;
			//joycall.bound_push_back_cmd.angular.x = 0; joycall.bound_push_back_cmd.angular.y = 0; joycall.bound_push_back_cmd.angular.z = 0;
			joycall.x_bounds_exceeded[0] = false;
			joycall.x_bounds_exceeded[1] = false;
			joycall.y_bounds_exceeded[0] = false;
			joycall.y_bounds_exceeded[1] = false;
			joycall.z_bounds_exceeded[0] = false;
			joycall.z_bounds_exceeded[1] = false;
		}
		
		if (send_command)
		{
			joycall.cmd_vel_pub.publish(joycall.command_out);// publish command;
			std::cout << "\npublish command" << std::endl;
			std::cout << "linear x: " << joycall.command_out.linear.x << std::endl;
			std::cout << "linear y: " << joycall.command_out.linear.y << std::endl;
			std::cout << "linear z: " << joycall.command_out.linear.z << std::endl;
			std::cout << "angular z: " << joycall.command_out.angular.z << std::endl << std::endl;
		}
		
		if (write_to_file)
		{
			output_file.open(output_file_name, std::fstream::out | std::fstream::app);
		}
		if (output_file.is_open())
		{
			output_file  << ros::Time::now().toSec() - start_time.toSec() << "," 
			<< joycall.x_button << "," 
			<< joycall.body_vel.linear.x << "," << joycall.body_vel.linear.y << "," << joycall.body_vel.linear.z << ","
			<< joycall.command_out.linear.x << "," << joycall.command_out.linear.y << "," << joycall.command_out.linear.z << ","
			<< -1*joycall.command_out.angular.z << ","
			<< joycall.body_vel.angular.z << ","
			<< "\n";
			output_file.close();
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
