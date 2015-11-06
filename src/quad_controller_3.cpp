/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 // This controller program was implemented by Zack Bell. This controller was developed by Anup
 * Parikh in the research paper:
 // 
 // Anup Parikh "Homography Based Visual Servo Control with Scene Reconstruction"
 //
 //
 //M*/
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <homog_track/HomogDecomposed.h>
#include <homog_track/HomogMarker.h>
#include <homog_track/HomogComplete.h>
#include <homog_track/HomogDesired.h>
#include <homog_track/CameraDesiredStart.h>
#include <math.h>
#include <algorithm> 
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

// function to push values onto a vector and if it is over some length will remove the first and then push it
template <typename sample_set_type> void shift_sample_set(std::deque<sample_set_type> &sample_set, sample_set_type new_sample, const int sample_set_length)
{
	if (sample_set.size() > sample_set_length - 1)
	{
		sample_set.pop_front();
	}
	sample_set.push_back(new_sample);
	
}

// class for subscribing to decomposed homography and outputting controller commands to drive 
// the quad to a desired frame.
class QuadController
{
	public:
		// file to write to
		std::fstream output_file;
	
		// file name
		//std::string output_file_name = "/home/zack/v1_ws/src/homog_track/testing_files/testing.txt";
	
		// display all the calculation steps
		bool display_all_calc_steps = false;
		
		// alpha red
		// reference height
		double alpha_red_d = 1;
		double z_ref_real = 2;
		// node handle
		ros::NodeHandle nh;
		
		/********** Begin Topic Declarations **********/
		ros::Subscriber homog_decomp_sub;
		ros::Subscriber homog_desired_sub;
		ros::Subscriber homog_marker_sub;
		ros::Subscriber joy_sub;
		ros::Publisher takeoff_pub;
		ros::Publisher land_pub;
		ros::Publisher reset_pub;
		ros::Publisher cmd_vel_pub;
		ros::Publisher z_ref_pub;
		
		/********** End Topic Declarations **********/
		
		/********** Begin current pose message **********/
		geometry_msgs::PoseStamped pose_F_wrt_Fstar_gm;
		geometry_msgs::Quaternion Q_gm;
		tf::Quaternion Q_tf {0,0,0,0};
		geometry_msgs::Point P_F_wrt_Fstar;
		double alpha_red = 0;
		double alpha_green = 0;
		double alpha_cyan = 0;
		double alpha_purple = 0;
		
		/********** End current pose message **********/
		
		/********** Begin desired pose message **********/
		bool updating_desired = false;
		geometry_msgs::PoseStamped pose_Fd_wrt_Fstar_gm;
		geometry_msgs::Quaternion Qd_gm;
		tf::Quaternion Qd_tf {0,0,0,0};
		geometry_msgs::Point P_Fd_wrt_Fstar;
		geometry_msgs::Point prd_gm;
		geometry_msgs::Point pgd_gm;
		geometry_msgs::Point pcd_gm;
		geometry_msgs::Point ppd_gm;
		double zrd = 1;
		geometry_msgs::Vector3 wcd_gm;
		geometry_msgs::Vector3 vcd_gm;
		
		
		/********** End deisred pose message **********/
		
		/********** Begin marker message **********/
		
		geometry_msgs::Point pr_gm;
		geometry_msgs::Point pg_gm;
		geometry_msgs::Point pc_gm;
		geometry_msgs::Point pp_gm;
		bool reference_set = false;
		
		
		/********** End marker message **********/
		
		/********** Begin xbox controller message **********/
		double a_button_land_b0 = 0;
		double b_button_reset_b1 = 0;
		double y_button_takeoff_b3 = 0;
		double lb_button_teleop_b4 = 0;
		double rt_stick_ud_x_a3 = 0;
		double rt_stick_lr_y_a2 = 0;
		double lt_stick_ud_z_a1 = 0;
		double lt_stick_lr_th_a0 = 0;
		std::vector<double> controller_input;
		
		/********** End xbox controller message **********/
		
		/********** Begin Tracking Controller Declarations **********/
		// rotational error
		tf::Quaternion Q_error {0,0,0,0};
		tf::Vector3 Q_error_v {0,0,0};
		
		/********** Gains **********/
		/********** Gains **********/
		/********** Gains **********/
		
		// K_w scalar
		double Kws = 1;
		
		// rotational gain matrix initialize to identity
		tf::Matrix3x3 Kw {Kws,0,0,
						   0,Kws,0,
						   0,0,Kws};
						   
		// linear velocity gain scalar
		//double K_vs = 1000;
		double Kvs = 1;
		
		// linear velocity gain matrix
		tf::Matrix3x3 Kv {Kvs,0,0,
						   0,Kvs,0,
						   0,0,Kvs};
						   
		// control gains for the z_star_hat_dot calculation
		double gamma_1 = std::pow(5,1);
		double gamma_2 = std::pow(0.3,1);
		//double gamma_2 = 0;
		
		// current value for zr_star_hat and its timestamp initializing to the initial guess
		double zr_star_hat = 10;
		// output for the zr_star_hat
		std_msgs::Float64 zr_star_hat_msg;				   
		
		/********** Gains **********/
		/********** Gains **********/
		/********** Gains **********/
		
		// desired rotation quaternion
		tf::Quaternion Q_wcd {0,0,0,0};
		tf::Vector3 wcd {0,0,0};
		
		// camera rotation command
		tf::Quaternion Q_wc_temp {0,0,0,0};
		tf::Vector3 wc_temp {0,0,0};
		tf::Vector3 wc {0,0,0};
		
		// finding phi
		//// the camera matrix
		//tf::Matrix3x3 K { 567.79, 0, 337.35, //first row
						  //0, 564.52, 169.54, // second row
						  //0, 0, 1}; //third row 
						
		//// matrix with the principle point coordinates of the camera
		//tf::Matrix3x3 principle_point { 0, 0, 337.35, // u0
										//0, 0, 169.54, // v0
										//0, 0, 0};
		tf::Matrix3x3 K = tf::Matrix3x3(1,0,0,0,1,0,0,0,1), principle_point = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);// matrix with the principle point coordinates of the camera
		tf::Vector3 pr = tf::Vector3(0,0,0), mr = tf::Vector3(0,0,0);// the red circle normalized components and its pixel coordinates and it as a skew symetrix
		tf::Matrix3x3 mr_mat = tf::Matrix3x3(0,0,0,0,0,0,0,0,0), ss_mr = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);
		tf::Matrix3x3 Lv = tf::Matrix3x3(0,0,0,0,0,0,0,0,0), Lv_term1 = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);// Lv
		tf::Vector3 phi_term1 = tf::Vector3(0,0,0), phi = tf::Vector3(0,0,0);// phi
		tf::Vector3 ped_dot_term1 = tf::Vector3(0,0,0), ped_dot_term2 = tf::Vector3(0,0,0), ped_dot = tf::Vector3(0,0,0);//ped_dot
		tf::Matrix3x3 Lvd = tf::Matrix3x3(0,0,0,0,0,0,0,0,0), Lvd_term1 = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);// Lvd
		tf::Vector3 vcd = tf::Vector3(0,0,0), prd = tf::Vector3(0,0,0), mrd = tf::Vector3(0,0,0);// desired velocity, desired pixels, and desired normalized 
		tf::Matrix3x3 mrd_mat = tf::Matrix3x3(0,0,0,0,0,0,0,0,0), ss_mrd = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);// matrix for Lvd calc and ss mrd
		const int number_of_samples = 20;// number of samples for stack
		const int number_of_integrating_samples = 100;// number of integrating samples
		tf::Vector3 ev = tf::Vector3(0,0,0), pe = tf::Vector3(0,0,0), ped = tf::Vector3(0,0,0); // value for calculating the current ev, that is pe-ped
		double current_timestamp = 0, last_timestamp = 0;// current timestamp in seconds and last time step in seconds
		tf::Vector3 Ev = tf::Vector3(0,0,0); tf::Vector3 Phi = tf::Vector3(0,0,0); tf::Vector3 U = tf::Vector3(0,0,0); tf::Vector3 Uvar = tf::Vector3(0,0,0);// current value for Ev, Phi, U, and Uvar
		std::deque<tf::Vector3> Ev_samples, Phi_samples, Uvar_samples;// buffer samples for learning stack terms
		std::deque<double> trapz_timestamps;// times for buffer samples
		tf::Vector3 vc = tf::Vector3(0,0,0);// linear velocity vc = 1/alpha_red*inv(Lv)*[Kv*ev + phi*zr_star_hat]
		std::deque<tf::Vector3> Us, Evs_minus_Phis;// learning stack terms
		std::deque<double> Evs_minus_Phis_svds;// norms for the Ev - Phi stack
		tf::Vector3 Ev_minus_Phi;// current Ev - Phi
		double Ev_minus_Phi_svd;// current Ev - Phi svd
		double curr_Ev_minus_Phi_svd_min;// min svd on the Ev - Phi svd stack
		std::deque<double>::iterator curr_Ev_minus_Phi_svd_min_iter; // iterator to the min norm on the stack
		int curr_Ev_minus_Phi_svd_min_index;
		double zr_star_hat_dot = 0;// current value for zr_star_hat_dot = gamma1*transpose(ev)*phi + gamma1*gamma2*sum_from_k=1_to_n( transpose(Ev(k)-Phi(k))*(-(Ev(k)-Phi(k))*zr_star_hat - U(k)) )
		double zr_star_hat_dot_sum = 0;// temp for the summation in zr_star_hat_dot
		bool first_time_update = true; // first time update occurance
		bool dont_calculate_zr_integral = true;// indicates not to calculate integral because the integral buffers are not full
		bool first_desired_recieved = false, first_marker_recieved = false; // first messages recieved
		
		/********** End Tracking Controller Declarations **********/
		
		/********** Begin output message **********/
		geometry_msgs::Twist velocity_command;
		geometry_msgs::Twist command_from_tracking;
		geometry_msgs::Twist command_from_xbox;
		
		/********** End output message **********/
		
		// constructor for the controller
		QuadController()
		{
			// subscribing to the decomposed messages,desired messages, marker messages, and xbox controller
			homog_decomp_sub = nh.subscribe("decomposed_homography",1,&QuadController::homog_decomp_callback,this);
			homog_desired_sub = nh.subscribe("desired_homogrphy",1,&QuadController::homog_desired_callback,this);
			homog_marker_sub = nh.subscribe("complete_homog_set",1,&QuadController::homog_marker_callback,this);
			joy_sub = nh.subscribe("joy",1,&QuadController::joy_callback,this);
			
			// publishers to the takeoff message, land message, reset message, and velocity message
			takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff",1);
			land_pub = nh.advertise<std_msgs::Empty>("ardrone/land",1);
			reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset",1);
			cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
			z_ref_pub = nh.advertise<std_msgs::Float64>("z_ref_hat",1);
			
			// initially sending it a desired command of 0
			cmd_vel_pub.publish(geometry_msgs::Twist());
			
			// initializing the constant part of Lv
			for (int ii = 0; ii < 3; ii++)
			{	
				for (int jj = 0; jj<3; jj++)
				{
					Lv_term1[ii][jj] = K[ii][jj] - principle_point[ii][jj];
				}
			}
			Lvd_term1 = Lv_term1;
			
			// initializing the learning parameters to 0s
			for (int ii = 0; ii < number_of_samples; ii++)
			{
				Evs_minus_Phis.push_back(tf::Vector3(0,0,0));
				Evs_minus_Phis_svds.push_back(0.0);
				Us.push_back(tf::Vector3(0,0,0));
			}
			
			//output_file.open(output_file_name, std::fstream::out | std::fstream::app);
			//if (output_file.is_open())
			//{
				//output_file << "zhat," 
							//<< "m," 
							//<< "md,"
							//<< "uv,"
							//<< "uvd,"
							//<< "q,"
							//<< "qd,"
							//<< "alphar,"
							//<< "alphard,"
							//<< "pe,"
							//<< "ped,"
							//<< "ev,"
							//<< "qTilde,"
							//<< "vcd,"
							//<< "wcd,"
							//<< "Lv,"
							//<< "Lvd,"
							//<< "pedDot,"
							//<< "wc,"
							//<< "phi,"
							//<< "vc"
							//<< "camPosDot,"
							//<< "camOrientDot,"
							//<< "desCamPosDot,"
							//<< "desCamOrientDot,"
							//<< "zhatDot"
							//<< "\n";
				//output_file.close();
			//}
		}
		
		/********** callback for the current pose message, position and orientation of F wrt Fstar **********/
		void homog_decomp_callback(const homog_track::HomogDecomposed& msg)
		{
			// getting the header and new message for camera to reference
			pose_F_wrt_Fstar_gm.header.stamp = msg.header.stamp;
			// current timestamp in seconds
			current_timestamp = pose_F_wrt_Fstar_gm.header.stamp.toSec();
			pose_F_wrt_Fstar_gm.pose = msg.pose;
			Q_gm = pose_F_wrt_Fstar_gm.pose.orientation;
			Q_tf = tf::Quaternion(Q_gm.x,Q_gm.y,Q_gm.z,Q_gm.w);
			P_F_wrt_Fstar = pose_F_wrt_Fstar_gm.pose.position;
			alpha_red = msg.alpha_red.data;
			alpha_green = msg.alpha_green.data;
			alpha_cyan = msg.alpha_cyan.data;
			alpha_purple = msg.alpha_purple.data;
			// call the velocity when a new decomp comes in
			output_velocity_command();
			
		}
		
		/********** callback for the desired pose message, position and orientation of Fd wrt Fstar **********/
		void homog_desired_callback(const homog_track::HomogDesired& msg)
		{
			pose_Fd_wrt_Fstar_gm.header.stamp = msg.header.stamp;
			pose_Fd_wrt_Fstar_gm.pose = msg.pose;
			Qd_gm = pose_Fd_wrt_Fstar_gm.pose.orientation;
			Qd_tf = tf::Quaternion(Qd_gm.x,Qd_gm.y,Qd_gm.z,Qd_gm.w);
			P_Fd_wrt_Fstar = pose_Fd_wrt_Fstar_gm.pose.position;
			prd_gm = msg.red_circle;
			pgd_gm = msg.green_circle;
			pcd_gm = msg.cyan_circle;
			ppd_gm = msg.purple_circle;
			zrd = msg.height.data;
			wcd_gm = msg.omega_cd;
			vcd_gm = msg.v_cd;
			
			if (!first_desired_recieved)
			{	
				first_desired_recieved = true;
			}
		}
		
		/********** callback for the current image points **********/
		void homog_marker_callback(const homog_track::HomogComplete& msg)
		{
			reference_set = msg.reference_set;
			pr_gm = msg.current_points.red_circle;
			pg_gm = msg.current_points.green_circle;
			pc_gm = msg.current_points.cyan_circle;
			pp_gm = msg.current_points.purple_circle;
			
			if (!first_marker_recieved)
			{
				first_marker_recieved = true;
			}
			
		}
		
		/********** callback for the controller **********/
		void joy_callback(const sensor_msgs::Joy& msg)
		{

			// cleaning the xbox twist message
			command_from_xbox = geometry_msgs::Twist();

			// breaking out the values
			a_button_land_b0 = msg.buttons[0];
			if (a_button_land_b0 > 0)
				land_pub.publish(std_msgs::Empty());
			
			b_button_reset_b1 = msg.buttons[1];
			if (b_button_reset_b1 > 0)
				reset_pub.publish(std_msgs::Empty());
			
			y_button_takeoff_b3 = msg.buttons[3];
			if (y_button_takeoff_b3 > 0)
				takeoff_pub.publish(std_msgs::Empty());
				
			lb_button_teleop_b4 = msg.buttons[4];
			
			
			rt_stick_ud_x_a3 = joy_deadband(msg.axes[3]);
			command_from_xbox.linear.x = rt_stick_ud_x_a3;
			
			rt_stick_lr_y_a2 = joy_deadband(msg.axes[2]);
			command_from_xbox.linear.y = rt_stick_lr_y_a2;
			
			lt_stick_ud_z_a1 = joy_deadband(msg.axes[1]);
			command_from_xbox.linear.z = lt_stick_ud_z_a1;
			
			lt_stick_lr_th_a0 = joy_deadband(msg.axes[0]);
			command_from_xbox.angular.z = lt_stick_lr_th_a0;
			
		}
		
		// deadband filter for the joy input
		double joy_deadband(double input_value)
		{
			double filtered_value = 0;
			std::cout << " input " << input_value << std::endl;
			std::cout << " abs input " << std::abs(input_value) << std::endl;
			if (std::abs(input_value) > 0.11)
			{
				filtered_value = input_value;
			}
			std::cout << " output " << filtered_value << std::endl;
			return filtered_value;
		}
		
		// generates the velocity command from tracking
		void generate_velocity_command_from_tracking()
		{
			// current position output, if alpha is greater than 0, it is a valid position and will continue
			std::cout << std::endl << std::endl << "current alpha red:\t" << alpha_red << std::endl;
			if (alpha_red > 0 && first_desired_recieved && first_marker_recieved)
			{
				get_wc();// getting the angular velocity command
				get_vc();// getting the linear velocity command
				update_zr_star_hat();// updating the depth estimate
				command_from_tracking.linear.x = vc.getX(); command_from_tracking.linear.y = vc.getY(); command_from_tracking.linear.z = vc.getZ(); command_from_tracking.angular.z = wc.getZ();// output velocity from tracking message
			}
			else
			{
				std::cout << "negative alpha_red" << std::endl;
				command_from_tracking.linear.x = 0; command_from_tracking.linear.y = 0; command_from_tracking.linear.z = 0; command_from_tracking.angular.z = 0;// output velocity from tracking message as zeros
			}
		}
		
		/********** Calculate the value for the angular velocity command wc, angular velocity of F wrt Fstar**********/
		void get_wc()
		{
			bool display_wc_calc_steps = true;// display all the calculation steps
			tf::Quaternion Qerror_temp = Qd_tf.inverse()*Q_tf;// rotational error
			tf::Vector3 Qerror_ijk_temp(Qerror_temp.getX(),Qerror_temp.getY(),Qerror_temp.getZ());// temp to hold the rotational error ijk terms
			tf::Vector3 wcd_temp(wcd_gm.x, wcd_gm.y, wcd_gm.z);
			wcd = wcd_temp;// getting the desired angular velocity
			tf::Quaternion Qwcd_temp(wcd.getX(),wcd.getY(),wcd.getZ(),0.0);// getting the desired angular velocity as a quaternion to use for the camera angular velocity command
			tf::Vector3 wc_term1_temp;
			wc_term1_temp = -1*(Kw*Qerror_ijk_temp); // getting the first term of the wc calc
			tf::Quaternion Qwc_term2_temp = (Qerror_temp.inverse()*Qwcd_temp)*Qerror_temp;// calculating the double product for the reporjection of wcd as a quaternion
			tf::Vector3 wc_term2_temp(Qwc_term2_temp.getX(), Qwc_term2_temp.getY(), Qwc_term2_temp.getZ());// getting the second term as a vector		  
			tf::Vector3 wc_temp = wc_term1_temp + wc_term2_temp;
			wc = wc_temp;// update the velocity
			// display the calcs
			if (display_wc_calc_steps || display_all_calc_steps)
			{
				std::cout << "wc CALC STEPS:" << std::endl;
				// desired rotation
				std::cout << "Rotation, Fd wrt Fstar:" 
						  << "\n x: " << Qd_tf.getX() 
						  << " y: " << Qd_tf.getY()
						  << " z: " << Qd_tf.getZ()
						  << " w: " << Qd_tf.getW()
						  << std::endl;
				// desired rotation inverse
				std::cout << "Inverse of Rotation, Fd wrt Fstar:" 
						  << "\n x: " << Qd_tf.inverse().getX() 
						  << " y: " << Qd_tf.inverse().getY()
						  << " z: " << Qd_tf.inverse().getZ()
						  << " w: " << Qd_tf.inverse().getW()
						  << std::endl;
				// current rotation
				std::cout << "Rotation, F wrt Fstar:" 
						  << "\n x: " << Q_tf.getX() 
						  << " y: " << Q_tf.getY()
						  << " z: " << Q_tf.getZ()
						  << " w: " << Q_tf.getW()
						  << std::endl;
				// rotational error
				std::cout << "Rotational error:" 
						  << "\n x: " << Qerror_temp.getX() 
						  << " y: " << Qerror_temp.getY()
						  << " z: " << Qerror_temp.getZ()
						  << " w: " << Qerror_temp.getW()
						  << std::endl;	
				// rotational error ijk terms
				std::cout << "Rotational error ijk terms:" 
						  << "\n x: " << Qerror_ijk_temp.getX() 
						  << " y: " << Qerror_ijk_temp.getY()
						  << " z: " << Qerror_ijk_temp.getZ()
						  << std::endl;
				// desired angular velocity
				std::cout << "Angular velocity, Fd wrt Fstar expressed in Fd:" 
						  << "\n x: " << wcd.getX() 
						  << " y: " << wcd.getY()
						  << " z: " << wcd.getZ()
						  << std::endl;			
				// desired angular velocity as a quaternion
				std::cout << "Angular velocity as quaternion, Fd wrt Fstar expressed in Fd:" 
						  << "\n x: " << Qwcd_temp.getX() 
						  << " y: " << Qwcd_temp.getY()
						  << " z: " << Qwcd_temp.getZ()
						  << " w: " << Qwcd_temp.getW()
						  << std::endl;
				// first term for the wc calc
				std::cout << "First term for the wc calc:" 
						  << "\n x: " << wc_term1_temp.getX() 
						  << " y: " << wc_term1_temp.getY()
						  << " z: " << wc_term1_temp.getZ()
						  << std::endl;
				// second term for the wc calc
				std::cout << "Second term for wc calc:" 
						  << "\n x: " << Qwc_term2_temp.getX() 
						  << " y: " << Qwc_term2_temp.getY()
						  << " z: " << Qwc_term2_temp.getZ()
						  << " w: " << Qwc_term2_temp.getW()
						  << std::endl;
				// second term for the wc calc ijk part
				std::cout << "Second term for wc calc ijk terms:" 
						  << "\n x: " << wc_term2_temp.getX() 
						  << " y: " << wc_term2_temp.getY()
						  << " z: " << wc_term2_temp.getZ()
						  << std::endl;
				// wc calc
				std::cout << "Wc as a vector:" 
						  << "\n x: " << wc.getX() 
						  << " y: " << wc.getY()
						  << " z: " << wc.getZ()
						  << std::endl;
			}
			
		}
		
		/********** Calculate the value for the linear velocity command vc, linear velocity of F wrt Fstar**********/
		void get_vc()
		{
			bool display_vc_calc_steps = true;// output the calculation steps
			update_pixels();// update the pixels and everything directly depending on them
			get_Lv();// getting Lvs
			get_Lvd();// getting Lvd
			get_ev();// getting ev
			get_phi();// getting little phi
			tf::Vector3 vc_term1_temp = (1/alpha_red)*(Lv.inverse()*(Kv*ev));// term 1 for the vc calculation
			tf::Vector3 vc_term2_temp = (1/alpha_red)*(Lv.inverse()*(phi*zr_star_hat));// term 2 for the vc calculation
			tf::Vector3 vc_temp = vc_term1_temp + vc_term2_temp;// sum them together for vc
			vc = vc_temp;
			// display the calcs
			if (display_vc_calc_steps || display_all_calc_steps)
			{
				std::cout << "vc CALC STEPS:" << std::endl;
				// camera pixels
				std::cout << "alpha red: " << alpha_red << std::endl; 
				std::cout << "Red pixel coordinates:" 
						  << "\n x: " << pr.getX() 
						  << " y: " << pr.getY()
						  << " z: " << pr.getZ()
						  << std::endl;	  
				std::cout << "Red normalized euclidean coordinates:" 
						  << "\n x: " << mr.getX() 
						  << " y: " << mr.getY()
						  << " z: " << mr.getZ()
						  << std::endl;
				std::cout << "Red skew symmetric normalized matrix:"
						  << "\n row 0:"
						  << " x: " << ss_mr.getRow(0).getX()
						  << " y: " << ss_mr.getRow(0).getY()
						  << " z: " << ss_mr.getRow(0).getZ()
						  << "\n row 1:"
						  << " x: " << ss_mr.getRow(1).getX()
						  << " y: " << ss_mr.getRow(1).getY()
						  << " z: " << ss_mr.getRow(1).getZ()
						  << "\n row 2:"
						  << " x: " << ss_mr.getRow(2).getX()
						  << " y: " << ss_mr.getRow(2).getY()
						  << " z: " << ss_mr.getRow(2).getZ()
						  <<std::endl;
				std::cout << "Red normalized matrix for term 2 of Lv:"
						  << "\n row 0:"
						  << " x: " << mr_mat.getRow(0).getX()
						  << " y: " << mr_mat.getRow(0).getY()
						  << " z: " << mr_mat.getRow(0).getZ()
						  << "\n row 1:"
						  << " x: " << mr_mat.getRow(1).getX()
						  << " y: " << mr_mat.getRow(1).getY()
						  << " z: " << mr_mat.getRow(1).getZ()
						  << "\n row 2:"
						  << " x: " << mr_mat.getRow(2).getX()
						  << " y: " << mr_mat.getRow(2).getY()
						  << " z: " << mr_mat.getRow(2).getZ()
						  <<std::endl;
				std::cout << "Current Lv:"
						  << "\n row 0:"
						  << " x: " << Lv.getRow(0).getX()
						  << " y: " << Lv.getRow(0).getY()
						  << " z: " << Lv.getRow(0).getZ()
						  << "\n row 1:"
						  << " x: " << Lv.getRow(1).getX()
						  << " y: " << Lv.getRow(1).getY()
						  << " z: " << Lv.getRow(1).getZ()
						  << "\n row 2:"
						  << " x: " << Lv.getRow(2).getX()
						  << " y: " << Lv.getRow(2).getY()
						  << " z: " << Lv.getRow(2).getZ()
						  <<std::endl;
				// desired pixels
				std::cout << "desired alpha red: " << alpha_red_d << std::endl; 
				std::cout << "Desired red pixel coordinates:" 
						  << "\n x: " << prd.getX() 
						  << " y: " << prd.getY()
						  << " z: " << prd.getZ()
						  << std::endl;
				std::cout << "Desired red normalized euclidean coordinates:" 
						  << "\n x: " << mrd.getX() 
						  << " y: " << mrd.getY()
						  << " z: " << mrd.getZ()
						  << std::endl;
				std::cout << "Desired red skew symmetric normalized matrix:"
						  << "\n row 0:"
						  << " x: " << ss_mrd.getRow(0).getX()
						  << " y: " << ss_mrd.getRow(0).getY()
						  << " z: " << ss_mrd.getRow(0).getZ()
						  << "\n row 1:"
						  << " x: " << ss_mrd.getRow(1).getX()
						  << " y: " << ss_mrd.getRow(1).getY()
						  << " z: " << ss_mrd.getRow(1).getZ()
						  << "\n row 2:"
						  << " x: " << ss_mrd.getRow(2).getX()
						  << " y: " << ss_mrd.getRow(2).getY()
						  << " z: " << ss_mrd.getRow(2).getZ()
						  <<std::endl;		  
				std::cout << "Desired red normalized matrix for term 2 of Lvd:"
						  << "\n row 0:"
						  << " x: " << mrd_mat.getRow(0).getX()
						  << " y: " << mrd_mat.getRow(0).getY()
						  << " z: " << mrd_mat.getRow(0).getZ()
						  << "\n row 1:"
						  << " x: " << mrd_mat.getRow(1).getX()
						  << " y: " << mrd_mat.getRow(1).getY()
						  << " z: " << mrd_mat.getRow(1).getZ()
						  << "\n row 2:"
						  << " x: " << mrd_mat.getRow(2).getX()
						  << " y: " << mrd_mat.getRow(2).getY()
						  << " z: " << mrd_mat.getRow(2).getZ()
						  <<std::endl;
				std::cout << "Current Lvd:"
						  << "\n row 0:"
						  << " x: " << Lvd.getRow(0).getX()
						  << " y: " << Lvd.getRow(0).getY()
						  << " z: " << Lvd.getRow(0).getZ()
						  << "\n row 1:"
						  << " x: " << Lvd.getRow(1).getX()
						  << " y: " << Lvd.getRow(1).getY()
						  << " z: " << Lvd.getRow(1).getZ()
						  << "\n row 2:"
						  << " x: " << Lvd.getRow(2).getX()
						  << " y: " << Lvd.getRow(2).getY()
						  << " z: " << Lvd.getRow(2).getZ()
						  <<std::endl;
				// ev
				std::cout << "pe:" 
						  << "\n x: " << pe.getX() 
						  << " y: " << pe.getY()
						  << " z: " << pe.getZ()
						  << std::endl;
				std::cout << "ped:" 
						  << "\n x: " << ped.getX() 
						  << " y: " << ped.getY()
						  << " z: " << ped.getZ()
						  << std::endl;
				std::cout << "Little ev:" 
						  << "\n x: " << ev.getX() 
						  << " y: " << ev.getY()
						  << " z: " << ev.getZ()
						  << std::endl;
				// phi
				std::cout << "Linear velocity of Fd wrt Fstar expressed in Fd:" 
						  << "\n x: " << vcd.getX() 
						  << " y: " << vcd.getY()
						  << " z: " << vcd.getZ()
						  << std::endl;
				std::cout << "Angular velocity of Fd wrt Fstar expressed in Fd:" 
						  << "\n x: " << wcd.getX() 
						  << " y: " << wcd.getY()
						  << " z: " << wcd.getZ()
						  << std::endl;	 
				std::cout << "Term 1 of the desired optical flow ped dot:" 
						  << "\n x: " << ped_dot_term1.getX() 
						  << " y: " << ped_dot_term1.getY()
						  << " z: " << ped_dot_term1.getZ()
						  << std::endl;
				std::cout << "Term 2 of the desired optical flow ped dot:" 
						  << "\n x: " << ped_dot_term2.getX() 
						  << " y: " << ped_dot_term2.getY()
						  << " z: " << ped_dot_term2.getZ()
						  << std::endl;
				std::cout << "Current ped_dot:" 
						  << "\n x: " << ped_dot.getX() 
						  << " y: " << ped_dot.getY()
						  << " z: " << ped_dot.getZ()
						  << std::endl;
				std::cout << "Little phi term 1 calc:" 
						  << "\n x: " << phi_term1.getX() 
						  << " y: " << phi_term1.getY()
						  << " z: " << phi_term1.getZ()
						  << std::endl;
				std::cout << "Little phi:" 
						  << "\n x: " << phi.getX() 
						  << " y: " << phi.getY()
						  << " z: " << phi.getZ()
						  << std::endl;
				std::cout << "z star actual: " << z_ref_real << std::endl;
				std::cout << "z star estimate: " << zr_star_hat << std::endl;
				std::cout << "Term 1 for the linear velocity command:" 
						  << "\n x: " << vc_term1_temp.getX() 
						  << " y: " << vc_term1_temp.getY()
						  << " z: " << vc_term1_temp.getZ()
						  << std::endl;
				std::cout << "Term 2 for the linear velocity command:" 
						  << "\n x: " << vc_term2_temp.getX() 
						  << " y: " << vc_term2_temp.getY()
						  << " z: " << vc_term2_temp.getZ()
						  << std::endl;	 
				std::cout << "Linear velocity command:" 
						  << "\n x: " << vc.getX() 
						  << " y: " << vc.getY()
						  << " z: " << vc.getZ()
						  << std::endl;
			}
		}
		/********** update the pixels and everything with them **********/
		void update_pixels()
		{
			tf::Vector3 pr_temp(pr_gm.x, pr_gm.y, pr_gm.z); 
			pr = pr_temp;// pixel coordinates as a vector
			tf::Vector3 mr_temp = K.inverse()*pr_temp; 
			mr = mr_temp;//normalized pixel coordinates
			tf::Matrix3x3 ss_mr_temp(           0, -1*mr.getZ(),    mr.getY(),
										    mr.getZ(),            0, -1*mr.getX(),
										 -1*mr.getY(),    mr.getX(),           0);
			ss_mr = ss_mr_temp;// skew symmetrix mr
			tf::Vector3 prd_temp(prd_gm.x,prd_gm.y,prd_gm.z);
			prd = prd_temp;// desired pixel coordinates
			tf::Vector3 mrd_temp = K.inverse()*prd_temp; 
			mrd = mrd_temp;// desried feature point normalized euclidean position
			tf::Matrix3x3 ss_mrd_temp(0,             -1*mrd.getZ(), mrd.getY(),
										  mrd.getZ(),    0,             -1*mrd.getX(),
										  -1*mrd.getY(), mrd.getX(),    0);
			ss_mrd = ss_mrd_temp;// skew symmetric mrd
		}
		
		/********** Calculate the value for Lv **********/
		void get_Lv()
		{
			tf::Matrix3x3 mr_mat_temp(1, 0, -1*mr.getX(),
									  0, 1, -1*mr.getY(),
									  0, 0, 1);
			mr_mat = mr_mat_temp;// matrix for the negated normalized x and y
			tf::Matrix3x3 Lv_temp = Lv_term1*mr_mat;
			Lv = Lv_temp;// calculating Lv
		}
		
		/********** calculating the value for Lvd **********/
		void get_Lvd()
		{	
			tf::Matrix3x3 mrd_mat_temp(1, 0, -1*mrd.getX(),
									   0, 1, -1*mrd.getY(),
									   0, 0, 1);
			mrd_mat = mrd_mat_temp;// matrix for the negated x and y component	
			tf::Matrix3x3 Lvd_temp = Lvd_term1*mrd_mat; 
			Lvd = Lvd_temp;// Lvd calc
		}
		
		/********** Calculate the value for ev **********/
		void get_ev()
		{
			tf::Vector3 pe_temp(pr.getX(), pr.getY(), -1*std::log(alpha_red)); 
			pe = pe_temp;// getting pe
			tf::Vector3 ped_temp(prd.getX(), prd.getY(), -1*std::log(alpha_red_d)); 
			ped = ped_temp;// getting ped
			tf::Vector3 ev_temp = pe - ped; 
			ev = ev_temp; // ev calc
		}
		
		/********** Calculate the value for ph **********/
		void get_phi()
		{
				get_ped_dot(); // getting ped_dot
				tf::Vector3 phi_term1_temp = (Lv*ss_mr)*wc; // getting the first term of the phi equation
				phi_term1 = phi_term1_temp;
				tf::Vector3 phi_temp = phi_term1 - ped_dot;
				phi = phi_temp;// summing the phi term1 with ped_dot for phi
		}
		
		/********** calculating the value for ped_dot **********/
		void get_ped_dot()
		{
			tf::Vector3 vcd_temp(vcd_gm.x,vcd_gm.y,vcd_gm.z);
			vcd = vcd_temp;// getting the desired linear velocity as a Vector3 to use with tf
			tf::Vector3 ped_dot_term1_temp = (-1*alpha_red_d/z_ref_real)*(Lvd*vcd);// getting term 1 for the ped_dot equation
			ped_dot_term1 = ped_dot_term1_temp;
			tf::Vector3 ped_dot_term2_temp = (Lvd*ss_mrd)*wcd;// getting term 2 for the ped_dot equation
			ped_dot_term2 = ped_dot_term2_temp;
			tf::Vector3 ped_dot_temp = ped_dot_term1_temp + ped_dot_term2_temp;
			ped_dot = ped_dot_temp;// ped_dot
		}
	
		/********** update the estimate for the reference zr_star_hat **********/
		void update_zr_star_hat()
		{
			bool display_zr_star_hat_calc_steps = true;// output the calcs
			calculate_zr_star_hat_dot();// calculate zr_star_hat_dot
			
			// get the time difference and update
			// if it is the first time updating will just equate the time stamp and leave but if it is beyond that will calculate using euler integration
			if (first_time_update)
			{
				first_time_update = false;
				last_timestamp = current_timestamp;
			}
			std::cout << "zr star hat before update: " << zr_star_hat << std::endl;
			std::cout << "zr star hat dot: " << zr_star_hat_dot << std::endl;
			std::cout << "zr star hat time difference scaled: " << (current_timestamp - last_timestamp) << std::endl;
			zr_star_hat += zr_star_hat_dot * (current_timestamp - last_timestamp);
			
			if (display_zr_star_hat_calc_steps || display_all_calc_steps)
			{
				std::cout << "Current Ev:"
						  << "\n x: " << Ev.getX() 
						  << " y: " << Ev.getY()
						  << " z: " << Ev.getZ()
						  << std::endl;
				std::cout << "Current Phi:" 
						  << "\n x: " << Phi.getX() 
						  << " y: " << Phi.getY()
						  << " z: " << Phi.getZ()
						  << std::endl;
				std::cout << "Current Uvar:"
						  << "\n x: " << Uvar.getX()
						  << " y: " << Uvar.getY()
						  << " z: " << Uvar.getZ()
						  << std::endl;
				std::cout << "Current U:"
						  << "\n x: " << U.getX()
						  << " y: " << U.getY()
						  << " z: " << U.getZ()
						  << std::endl;
				if 	(Phi_samples.size() > number_of_integrating_samples - 1)
				{
					std::cout << "stack min norm: " << *curr_Ev_minus_Phi_svd_min_iter << std::endl;
					std::cout << "index of the stack min norm: " << curr_Ev_minus_Phi_svd_min_index << std::endl;				
					std::cout << "new norm: " << Ev_minus_Phi_svd << std::endl;
				}
				std::cout << "current zr_star_hat_dot from ev and phi: " << gamma_1*ev.dot(phi) << std::endl;
				std::cout << "current zr_star_hat_dot from sum: " << gamma_1*gamma_2*zr_star_hat_dot_sum << std::endl;
				std::cout << "zr star hat dot: " << zr_star_hat_dot << std::endl;
				std::cout << "zr star hat time difference: " << (current_timestamp - last_timestamp) << std::endl;
				std::cout << "zr star hat: " << zr_star_hat << std::endl;
			}
			
			last_timestamp = current_timestamp;

		}
		
		/********** update the estimate for zr_star_hat_dot **********/
		void calculate_zr_star_hat_dot()
		{
			shift_sample_set(trapz_timestamps, current_timestamp, number_of_integrating_samples);// update the time for trapz
			get_Ev();// get the new Ev
			get_Phi();// get the new Phi
			get_U();// get the new U
			
			if (Phi_samples.size() > number_of_integrating_samples - 1)
			{
				Ev_minus_Phi = Ev - Phi;// Ev - Phi
				Ev_minus_Phi_svd = Ev_minus_Phi.length2();// Ev - Phi, sum of square of each element
				curr_Ev_minus_Phi_svd_min_iter = std::min_element(Evs_minus_Phis_svds.begin(),Evs_minus_Phis_svds.end());// finding the minimum value
				curr_Ev_minus_Phi_svd_min_index = std::distance(Evs_minus_Phis_svds.begin(),curr_Ev_minus_Phi_svd_min_iter);// getting the index of the iterator

				// if the current norm is greater than the minimum norm of the history will remove the old min and add the new one to the end
				if ( Ev_minus_Phi_svd > *curr_Ev_minus_Phi_svd_min_iter )
				{
					std::cout << "switching out index: " << curr_Ev_minus_Phi_svd_min_index << std::endl;
					// replace the previous min from the history
					Evs_minus_Phis.erase(Evs_minus_Phis.begin() + curr_Ev_minus_Phi_svd_min_index);
					Evs_minus_Phis_svds.erase(Evs_minus_Phis_svds.begin() + curr_Ev_minus_Phi_svd_min_index);
					Us.erase(Us.begin() + curr_Ev_minus_Phi_svd_min_index);
					shift_sample_set(Evs_minus_Phis, Ev_minus_Phi, number_of_samples);
					shift_sample_set(Evs_minus_Phis_svds, Ev_minus_Phi_svd, number_of_samples);
					shift_sample_set(Us, U, number_of_samples);
				}
				else
				{
					std::cout << "no switch" << std::endl;
				}
				// getting the summation
				zr_star_hat_dot_sum = 0;
				tf::Vector3 zr_star_hat_dot_term2_temp = tf::Vector3(0,0,0);
				// sum_from_k=1_to_n( transpose(Ev(k)-Phi(k))*(-(Ev(k)-Phi(k))*z_star_hat - U(k)) )
				for (int ii = 0; ii < Evs_minus_Phis.size(); ii++)
				{
					zr_star_hat_dot_term2_temp = -(Evs_minus_Phis.at(ii)*zr_star_hat) - Us.at(ii);
					zr_star_hat_dot_sum += (Evs_minus_Phis.at(ii)).dot(zr_star_hat_dot_term2_temp);
				}

			}
			else
			{
				zr_star_hat_dot_sum = 0;
			}
			//z_star_hat_dot = gamma_1*transpose(ev)*phi + gamma1*gamma2*zr_star_hat_dot_sum
			zr_star_hat_dot = gamma_1*ev.dot(phi) + gamma_1*gamma_2*zr_star_hat_dot_sum;
			
		}
		
		/********** calculating the value for Ev **********/
		void get_Ev()
		{
			shift_sample_set(Ev_samples, ev, number_of_samples);// shifting the buffer
			// getting Ev, it is the difference between the last and first, if only 1 element, just use the first
			if (Ev_samples.size() > 1)
			{
				Ev = Ev_samples.back() - Ev_samples.front();
			}
		}
		
		/********** calculating the value for big Phi **********/
		void get_Phi()
		{
			shift_sample_set(Phi_samples, phi, number_of_integrating_samples);// phi has already been calculated earlier so just need to put onto the deque then integrate
			calculate_integral(Phi, Phi_samples, trapz_timestamps);// getting the integral to calculate Phi
		}
		
		/********** calculating the value for Uvar **********/
		void get_U()
		{
			Uvar = alpha_red*(Lv*vc);// using a new variable uvar to represent the multiplication used to get the integral for U
			shift_sample_set(Uvar_samples,Uvar,number_of_integrating_samples);// putting the Uvars onto the Uvar deque
			calculate_integral(U, Uvar_samples, trapz_timestamps);// getting U, the integral of Uvar_samples
		}
		
		/********** function to calculate the integral of a Vector3 using trapizoidal rule **********/
		void calculate_integral(tf::Vector3 &integral, std::deque<tf::Vector3> &function_values, std::deque<double> &time_values)
		{
			integral.setValue(0,0,0);
			//tf::Vector3 temp_integral {0,0,0};
			// if the two deques are the same size will do the integral
			if (function_values.size() == time_values.size())
			{
				// if there is more than one element in the deque then will do the integral over the points, otherwise will just return it for the one
				if (function_values.size() > 1)
				{
					//std::cout << "start integrating";
					for (int ii = 0; ii < function_values.size() - 1; ii++)
					{
						//temp_integral = 
						integral += 0.5*(time_values[ii+1] - time_values[ii]) * (function_values[ii+1] + function_values[ii]);
						//std::cout << "\ncurrent integral:" 
								  //<< "\n\tx:\t" << integral.getX();
								
					}
					//std::cout << "\nend integrating" << std::endl;
				}
				
			}
			
		}
		
		// outputs the desired command which will be either the tracking controller or the xbox
		// controller
		// NOTE The coordinate frame of the body of the quad relative to the camera is xb = -yc, yb = -xc, and zb = -zc
		// 		angular zb is angular zc, that is angular zb is positive about zc, strange
		void output_velocity_command()
		{
			// cleaning up the velocity command
			velocity_command = geometry_msgs::Twist();
			zr_star_hat_msg.data = zr_star_hat;
			//generate_velocity_command_from_tracking();
			//velocity_command = command_from_tracking;
			
			//// if the left bumper is pulled will output the command from the xbox controller otherwise will output from the tracking controller
			//if (lb_button_teleop_b4 > 0)
			//{
				//std::cout << "command from xbox\n" << command_from_xbox << std::endl;
				//velocity_command = command_from_xbox;
			//}
			//else
			//{
				//std::cout << "command from controller\n" << command_from_xbox << std::endl;
				//generate_velocity_command_from_tracking();
				//velocity_command = command_from_tracking;
			//}
			//std::cout << "command from controller:\n" << command_from_xbox << std::endl;
			generate_velocity_command_from_tracking();
			velocity_command = command_from_tracking;
			cmd_vel_pub.publish(velocity_command);
			z_ref_pub.publish(zr_star_hat_msg);
			
			//output_file.open(output_file_name, std::fstream::out | std::fstream::app);
			//if (output_file.is_open())
			//{
				//output_file << zr_star_hat << "," 
							//<< mr 
							//<< mrd
							//<< pr
							//<< prd
							//<< Q_tf
							//<< Qd_tf
							//<< alpha_red
							//<< alpha_red_d
							//<< pe
							//<< ped
							//<< ev
							//<< Q_error
							//<< vcd
							//<< wcd
							//<< Lv
							//<< Lvd
							//<< ped_dot
							//<< wc
							//<< phi
							//<< vc
							//<< "camPosDot,"
							//<< "camOrientDot,"
							//<< "desCamPosDot,"
							//<< "desCamOrientDot,"
							//<< "zhatDot"
							//<< "\n";
				//output_file.close();
			//}
			
		}
		
		
};

// main
int main(int argc, char** argv)
{   
	// initialize node
	ros::init(argc,argv,"quad_controller_node");
	
	// initialize the controller    
	QuadController quad_controller;
	
    ros::spin();
	
    
    return 0;
}
