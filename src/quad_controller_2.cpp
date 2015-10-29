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
		geometry_msgs::PoseStamped pose_fc_gm;
		geometry_msgs::Quaternion Q_fc;
		tf::Quaternion Q_fc_tf {0,0,0,0};
		geometry_msgs::Point P_fc;
		double alpha_red = 0;
		double alpha_green = 0;
		double alpha_cyan = 0;
		double alpha_purple = 0;
		
		/********** End current pose message **********/
		
		/********** Begin desired pose message **********/
		bool updating_desired = false;
		geometry_msgs::PoseStamped pose_df_gm;
		geometry_msgs::Quaternion Q_df;
		tf::Quaternion Q_df_tf {0,0,0,0};
		geometry_msgs::Point P_df;
		geometry_msgs::Point p_r_d;
		geometry_msgs::Point p_g_d;
		geometry_msgs::Point p_c_d;
		geometry_msgs::Point p_p_d;
		double z_rd = 1;
		geometry_msgs::Vector3 wcd_gm;
		geometry_msgs::Vector3 vcd_gm;
		
		
		/********** End deisred pose message **********/
		
		/********** Begin marker message **********/
		
		geometry_msgs::Point32 p_r;
		geometry_msgs::Point32 p_g;
		geometry_msgs::Point32 p_c;
		geometry_msgs::Point32 p_p;
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
		double K_ws = 0.1;
		
		// rotational gain matrix initialize to identity
		tf::Matrix3x3 K_w {K_ws,0,0,
						   0,K_ws,0,
						   0,0,K_ws};
						   
		// linear velocity gain scalar
		double K_vs = 1000;
		
		// linear velocity gain matrix
		tf::Matrix3x3 K_v {K_vs,0,0,
						   0,K_vs,0,
						   0,0,0.001*K_vs};
						   
		// control gains for the z_star_hat_dot calculation
		double gamma_1 = std::pow(10.0,-4.0);
		double gamma_2 = std::pow(10.0,-2.0);						   
		
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
		// the camera matrix
		tf::Matrix3x3 K { 567.79, 0, 337.35, //first row
						  0, 564.52, 169.54, // second row
						  0, 0, 1}; //third row 
						
		// matrix with the principle point coordinates of the camera
		tf::Matrix3x3 principle_point { 0, 0, 337.35, // u0
										0, 0, 169.54, // v0
										0, 0, 0};
		
		// the red circle normalized components and its pixel coordinates and it as a skew symetrix
		tf::Vector3 pr {0,0,0};
		tf::Vector3 mr  {0,0,0};
		tf::Matrix3x3 mr_mat {0,0,0,0,0,0,0,0,0};
		tf::Matrix3x3 ss_mr {0,0,0,0,0,0,0,0,0};
		
		// Lv
		tf::Matrix3x3 Lv {0,0,0,0,0,0,0,0,0};
		tf::Matrix3x3 Lv_temp {0,0,0,0,0,0,0,0,0};
		
		// phi
		tf::Vector3 phi {0,0,0};
		tf::Vector3 phi_tempL {0,0,0};
		tf::Vector3 ped_dot {0,0,0};
		tf::Vector3 ped_dot_tempL {0,0,0};
		tf::Vector3 ped_dot_tempR {0,0,0};
		
		// ped_dot z_rd defined previously
		double alpha_rd = 1;
		
		// Lvd
		tf::Matrix3x3 Lvd {0,0,0,0,0,0,0,0,0};
		tf::Matrix3x3 Lvd_temp {0,0,0,0,0,0,0,0,0};
		
		// the desired red circle normalized and its pixel coordinates and it as a skew symmetric
		tf::Vector3 vcd {0,0,0};
		tf::Vector3 prd {0,0,0};
		tf::Vector3 mrd {0,0,0};
		tf::Matrix3x3 mrd_mat {0,0,0,0,0,0,0,0,0};
		tf::Matrix3x3 ss_mrd {0,0,0,0,0,0,0,0,0};
		
		// finding the depth estimate
		// initial guess to the depth
		double zr_star_hat_0 = 2;
		
		// current timestamp in seconds
		double current_timestamp = 0;
		
		// number of samples for everything
		const int number_of_samples = 20;
		
		// number of integrating samples
		const int number_of_integrating_samples = 10;
		
		// deques for saving the Ev, Phi, U, and the time stamp at each one for the estimate for the estimate
		// desired sample length
		//const int number_learning_samples = 20;
		std::deque<tf::Vector3> Evs_minus_Phis;
		std::deque<double> Evs_minus_Phis_norms;
		tf::Vector3 Ev_minus_Phi;
		double Ev_minus_Phi_norm;
		double curr_Ev_minus_Phi_norm_min;
		std::deque<double>::iterator curr_Ev_minus_Phi_norm_min_iter;
		int curr_Ev_minus_Phi_norm_min_index;
		tf::Vector3 zr_star_hat_dot_R;
		
		std::deque<tf::Vector3> Us;
		std::deque<double> learning_timestamps;
		
		// current value for Ev, Phi, U, and Uvar
		tf::Vector3 Ev {0,0,0};
		tf::Vector3 Phi {0,0,0};
		tf::Vector3 U {0,0,0};
		tf::Vector3 Uvar {0,0,0};
		
		//double learning_timestamp;
		
		// deques for calculating Ev
		//const int number_Ev_samples = 20;
		std::deque<tf::Vector3> Ev_samples;
		
		// value for calculating the current ev, that is pe-ped
		tf::Vector3 ev {0,0,0};
		tf::Vector3 pe {0,0,0};
		tf::Vector3 ped {0,0,0};
		
		// deques for calculating the integral Phi
		//const int number_Phi_samples = 20;
		std::deque<tf::Vector3> Phi_samples;
		std::deque<double> Phi_timestamps;
		
		// deques for calculating the integral U, create a new variable that is alpha*Lv*vc called Uvar
		//const int number_U_samples = 20;
		std::deque<tf::Vector3> Uvar_samples;
		std::deque<double> Uvar_timestamps;
		
		
		// linear velocity vc = 1/alpha_red*inv(Lv)*[Kv*ev + phi*zr_star_hat]
		tf::Vector3 vc {0,0,0};
		
		// temp vector to hold [Kv*ev + phi*zr_star_hat]
		tf::Vector3 vc_tempL {0,0,0};
		
		// holds the inverse of Lv for the calc of vc
		tf::Matrix3x3 Lv_inv {0,0,0,0,0,0,0,0,0};
		
		
		
		// deque for calculating the estimate as an integral of the rate of change
		std::deque<double> zr_star_hat_dot_samples;
		std::deque<double> zr_star_hat_dot_timestamps;
		
		// current value for zr_star_hat_dot = gamma1*transpose(ev)*phi + gamma1*gamma2*sum_from_k=1_to_n( transpose(Ev(k)-Phi(k))*(-(Ev(k)-Phi(k))*zr_star_hat - U(k)) )
		double zr_star_hat_dot = 0;
		
		// temp for the summation in zr_star_hat_dot
		double zr_star_hat_dot_sum = 0;
		
		// starting value for zr_hat_dot
		double zr_star_hat_dot_0 = 10;
		
		// current value for zr_star_hat and its timestamp initializing to the initial guess
		double zr_star_hat = 0;
		
		std_msgs::Float64 current_zref;
		
		// indicates not to calculate integral
		bool dont_calculate_zr_integral = true;
		
		
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
					Lv_temp[ii][jj] = K[ii][jj] - principle_point[ii][jj];
				}
			}
			Lvd_temp = Lv_temp;
			
			// initializing the learning parameters to 0s
			for (int ii = 0; ii < number_of_samples; ii++)
			{
				Evs_minus_Phis.push_back(tf::Vector3(0,0,0));
				Evs_minus_Phis_norms.push_back(0.0);
				Us.push_back(tf::Vector3(0,0,0));
				learning_timestamps.push_back(0.0);
			}
		}
		
		// callback for the current pose message
		void homog_decomp_callback(const homog_track::HomogDecomposed& msg)
		{
			// getting the header and new message for camera to reference
			pose_fc_gm.header.stamp = msg.header.stamp;
			// current timestamp in seconds
			current_timestamp = pose_fc_gm.header.stamp.toSec();
			pose_fc_gm.pose = msg.pose;
			Q_fc = pose_fc_gm.pose.orientation;
			Q_fc_tf = tf::Quaternion(Q_fc.x,Q_fc.y,Q_fc.z,Q_fc.w);
			P_fc = pose_fc_gm.pose.position;
			alpha_red = msg.alpha_red.data;
			alpha_green = msg.alpha_green.data;
			alpha_cyan = msg.alpha_cyan.data;
			alpha_purple = msg.alpha_purple.data;
			//std::cout << "current pose:\n" << pose_fc_gm.pose << std::endl;
			//std::cout << "current alpha red:\t" << alpha_red << std::endl;
			//std::cout << "current alpha green: " << alpha_green << std::endl;
			//std::cout << "current alpha cyan: " << alpha_cyan << std::endl;
			//std::cout << "current alpha purple: " << alpha_purple << std::endl;
	
			
		}
		
		// callback for the desired pose message
		void homog_desired_callback(const homog_track::HomogDesired& msg)
		{
			pose_df_gm.header.stamp = msg.header.stamp;
			pose_df_gm.pose = msg.pose;
			Q_df = pose_df_gm.pose.orientation;
			Q_df_tf = tf::Quaternion(Q_df.x,Q_df.y,Q_df.z,Q_df.w);
			P_df = pose_df_gm.pose.position;
			p_r_d = msg.red_circle;
			p_g_d = msg.green_circle;
			p_c_d = msg.cyan_circle;
			p_p_d = msg.purple_circle;
			z_rd = msg.height.data;
			wcd_gm = msg.omega_cd;
			vcd_gm = msg.v_cd;

		}
		
		// callback for the marker message
		void homog_marker_callback(const homog_track::HomogComplete& msg)
		{
			reference_set = msg.reference_set;
			p_r = msg.current_points.red_circle;
			
			p_g = msg.current_points.green_circle;
			p_c = msg.current_points.cyan_circle;
			p_p = msg.current_points.purple_circle;
		}
		
		// callback for the joy message
		void joy_callback(const sensor_msgs::Joy& msg)
		{
			// cleaning up the old controller deque
			//controller_input.erase(controller_input.begin(),controller_input.end());
			
			// cleaning the xbox twist message
			command_from_xbox = geometry_msgs::Twist();
			
			//std::cout << "joy message\n" << msg << std::endl;
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
			
			//std::cout << "command from xbox\n" << command_from_xbox << std::endl;
			
			//controller_input.push_back(a_button_land_b0);
			//controller_input.push_back(b_button_reset_b1);
			//controller_input.push_back(y_button_takeoff_b3);
			//controller_input.push_back(lb_button_teleop_b4);
			//controller_input.push_back(rt_stick_ud_x_a3);
			//controller_input.push_back(rt_stick_lr_y_a2);
			//controller_input.push_back(lt_stick_ud_z_a1);
			//controller_input.push_back(lt_stick_lr_th_a0);
			//for (int ii = 0 ; ii < controller_input.size(); ii++)
				//std::cout << ii << "\t" << controller_input[ii] << std::endl;
			//std::cout << std::endl;
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
			// current position output
			std::cout << std::endl << std::endl << "current alpha red:\t" << alpha_red << std::endl;
			if (alpha_red > 0)
			{
				std::cout << "current pose:\n" << pose_fc_gm.pose << std::endl;
				//std::cout << "current red pixel:\n" << p_r << std::endl;
				// desired position output
				std::cout << "desired pose:\n" << pose_df_gm.pose << std::endl;
				std::cout << "desired time:\n" << current_timestamp << std::endl;
				//std::cout << "desired red pixel:\n" << p_r_d << std::endl;
				
				/********** begin calculating wc **********/
				// rotational error
				Q_error = Q_df_tf.inverse()*Q_fc_tf;
				std::cout << "Current rotational error:" 
						  << "\n x: " << Q_error.getX() 
						  << " y: " << Q_error.getY()
						  << " z: " << Q_error.getZ()
						  << " w: " << Q_error.getW()
						  << std::endl;
				
				// getting the desired angular velocity as a vector 3 to use with tf
				wcd.setValue(wcd_gm.x, wcd_gm.y, wcd_gm.z);
				std::cout << "Wcd:" 
						  << "\n x: " << wcd.getX() 
						  << " y: " << wcd.getY()
						  << " z: " << wcd.getZ()
						  << std::endl;
				// getting the desired angular velocity as a quaternion to use for the camera angular velocity command
				Q_wcd = tf::Quaternion(wcd.getX(),wcd.getY(),wcd.getZ(),0.0);
				std::cout << "Wcd quaternion:" 
						  << "\n x: " << Q_wcd.getX() 
						  << " y: " << Q_wcd.getY()
						  << " z: " << Q_wcd.getZ()
						  << " w: " << Q_wcd.getW()
						  << std::endl;
						  
				// calculating the double product for the reporjection of wcd as a quaternion
				Q_wc_temp = (Q_error.inverse()*Q_wcd)*Q_error;
				std::cout << "Temp quaternion for the wcd part of wc calc:" 
						  << "\n x: " << Q_wc_temp.getX() 
						  << " y: " << Q_wc_temp.getY()
						  << " z: " << Q_wc_temp.getZ()
						  << " w: " << Q_wc_temp.getW()
						  << std::endl;
						  
				wc_temp.setValue(Q_wc_temp.getX(), Q_wc_temp.getY(), Q_wc_temp.getZ());
				
				std::cout << "Temp vector for the wcd part of wc calc:" 
						  << "\n x: " << wc_temp.getX() 
						  << " y: " << wc_temp.getY()
						  << " z: " << wc_temp.getZ()
						  << std::endl;
				
				// temp to hold the Q_error vector
				Q_error_v.setValue(Q_error.getX(),Q_error.getY(),Q_error.getZ());
				std::cout << "Q error vector:" 
						  << "\n x: " << Q_error_v.getX() 
						  << " y: " << Q_error_v.getY()
						  << " z: " << Q_error_v.getZ()
						  << std::endl;
				
				// temp for the left side of the calc
				tf::Vector3 wc_left_side;
				wc_left_side.setX(-1.0*(K_w.getRow(0).dot(Q_error_v)));
				wc_left_side.setY(-1.0*(K_w.getRow(1).dot(Q_error_v)));
				wc_left_side.setZ(-1.0*(K_w.getRow(2).dot(Q_error_v)));
				
				std::cout << "Temp vector for the q error vector term of the wc calc:" 
						  << "\n x: " << wc.getX() 
						  << " y: " << wc.getY()
						  << " z: " << wc.getZ()
						  << std::endl;
				
				
				// adding the temp to get the camera angular velocity command
				wc = wc_left_side + wc_temp;
				std::cout << "sum of the wc:" 
						  << "\n x: " << (wc_left_side + wc_temp).getX() 
						  << " y: " << (wc_left_side + wc_temp).getY()
						  << " z: " << (wc_left_side + wc_temp).getZ()
						  << std::endl;
						  
				tf::Vector3 test1_vec(1,2,3);
				tf::Vector3 test2_vec(4,5,6);
				std::cout << "sum of the test:" 
						  << "\n x: " << (test1_vec + test2_vec).getX() 
						  << " y: " << (test1_vec + test2_vec).getY()
						  << " z: " << (test1_vec + test2_vec).getZ()
						  << std::endl;
				
				//wc.setX( + wc_temp.getX());
				//wc.setY() + wc_temp.getY());
				//wc.setZ + wc_temp.getZ());
				
				//std::cout << "Current wc:" 
						  //<< "\n\tx:\t" << wc.getX() 
						  //<< "\n\ty:\t" << wc.getY()
						  //<< "\n\tz:\t" << wc.getZ()
						  //<< std::endl;
				/********** end calculating wc **********/
				
				/********** begin calculating phi **********/
				////////// getting Lv //////////
				pr.setValue(p_r.x, p_r.y, p_r.z);
				std::cout << "Current pr:" 
						  << "\n x: " << pr.getX() 
						  << " y: " << pr.getY()
						  << " z: " << pr.getZ()
						  << std::endl;
				mr = K.inverse()*pr;
				std::cout << "Current mr:" 
						  << "\n x: " << mr.getX() 
						  << " y: " << mr.getY()
						  << " z: " << mr.getZ()
						  << std::endl;
				
				mr_mat.setValue(1, 0, -1*mr.getX(),
								0, 1, -1*mr.getY(),
								0, 0, 1);
				Lv = Lv_temp*mr_mat;
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
						  
				// skew symmetrix mr
				ss_mr.setValue(0, -1*mr.getZ(), mr.getY(),
							   mr.getZ(), 0, -1*mr.getX(),
							   -1*mr.getY(), mr.getX(), 0);
				
				////////// getting ped_dot //////////
				// getting Lvd
				prd.setValue(p_r_d.x,p_r_d.y,p_r_d.z);
				std::cout << "Current prd:" 
						  << "\n x: " << prd.getX() 
						  << " y: " << prd.getY()
						  << " z: " << prd.getZ()
						  << std::endl;
				mrd = K.inverse()*prd;
				std::cout << "Current mrd:" 
						  << "\n x: " << mrd.getX() 
						  << " y: " << mrd.getY()
						  << " z: " << mrd.getZ()
						  << std::endl;
				
				mrd_mat.setValue(1, 0, -1*mrd.getX(),
								0, 1, -1*mrd.getY(),
								0, 0, 1);
								
				Lvd = Lvd_temp*mrd_mat;
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
				
				// getting the desired linear velocity as a Vector3 to use with tf
				vcd.setValue(vcd_gm.x,vcd_gm.y,vcd_gm.z);
				std::cout << "Current vcd:" 
						  << "\n x: " << vcd.getX() 
						  << " y: " << vcd.getY()
						  << " z: " << vcd.getZ()
						  << std::endl;
						  
				// skew symmetric mrd
				ss_mrd.setValue(0, -1*mrd.getZ(), mrd.getY(),
							   mrd.getZ(), 0, -1*mrd.getX(),
							   -1*mrd.getY(), mrd.getX(), 0);
				
				// getting the left side of the ped_dot equation
				ped_dot_tempL.setX( (-1*alpha_rd/z_rd)*(Lvd.getRow(0).dot(vcd)) );
				ped_dot_tempL.setY( (-1*alpha_rd/z_rd)*(Lvd.getRow(1).dot(vcd)) );
				ped_dot_tempL.setZ( (-1*alpha_rd/z_rd)*(Lvd.getRow(2).dot(vcd)) );
				std::cout << "Current ped_dot temp left vcd component:" 
						  << "\n x: " << ped_dot_tempL.getX() 
						  << " y: " << ped_dot_tempL.getY()
						  << " z: " << ped_dot_tempL.getZ()
						  << std::endl;
				
				// getting the right side of the ped_dot equation
				ped_dot_tempR.setX( (Lvd*ss_mrd).getRow(0).dot(wcd) );
				ped_dot_tempR.setY( (Lvd*ss_mrd).getRow(1).dot(wcd) );
				ped_dot_tempR.setZ( (Lvd*ss_mrd).getRow(2).dot(wcd) );
				std::cout << "Current ped_dot temp right wcd component:" 
						  << "\n x: " << ped_dot_tempR.getX() 
						  << " y: " << ped_dot_tempR.getY()
						  << " z: " << ped_dot_tempR.getZ()
						  << std::endl;
				
				// summing them by element for ped_dot
				ped_dot = ped_dot_tempL + ped_dot_tempR;
				std::cout << "Current ped_dot:" 
						  << "\n x: " << ped_dot.getX() 
						  << " y: " << ped_dot.getY()
						  << " z: " << ped_dot.getZ()
						  << std::endl;
						  
				// getting the left side of the phi equation
				phi_tempL.setX( (Lv*ss_mr).getRow(0).dot(wc) );
				phi_tempL.setY( (Lv*ss_mr).getRow(1).dot(wc) );
				phi_tempL.setZ( (Lv*ss_mr).getRow(2).dot(wc) );
				
				
				// summing the phi temp left with ped_dot for phi
				phi = phi_tempL + ped_dot;
				std::cout << "Current phi:" 
						  << "\n x: " << phi.getX() 
						  << " y: " << phi.getY()
						  << " z: " << phi.getZ()
						  << std::endl;
				/********** end calculating phi **********/
				
				/********** begin calculating the depth esitimate **********/
				////////// begin finding Ev //////////
				// first need ev = pe - ped
				// getting pe
				pe.setValue(pr.getX(), pr.getY(), -1*std::log(alpha_red));
				std::cout << "Current pe:" 
						  << "\n x: " << pe.getX() 
						  << " y: " << pe.getY()
						  << " z: " << pe.getZ()
						  << std::endl;
				// getting ped
				ped.setValue(prd.getX(), prd.getY(), -1*std::log(alpha_rd));
				std::cout << "Current ped:" 
						  << "\n x: " << ped.getX() 
						  << " y: " << ped.getY()
						  << " z: " << ped.getZ()
						  << std::endl;
				// ev calc
				ev = pe - ped;
				std::cout << "Current ev:" 
						  << "\n x: " << ev.getX() 
						  << " y: " << ev.getY()
						  << " z: " << ev.getZ()
						  << std::endl;
				
				// pushing ev onto the Ev_samples and if it has exceeded the max length will push it on and take the first one off
				shift_sample_set(Ev_samples, ev, number_of_samples);
				
				//std::cout << "Ev samples:";
				//for (tf::Vector3 ii : Ev_samples)
				//{
					//std::cout << "\n\tx:\t" << ii.getX()
							  //<< "\n\ty:\t" << ii.getY()
							  //<< "\n\tz:\t" << ii.getZ();
				//}
				//std::cout << std::endl;
				
				// getting Ev, it is the difference between the last and first, if only 1 element, just use the first
				if (Ev_samples.size() > 1)
				{
					Ev = Ev_samples.back() - Ev_samples.front();
				}
				else
				{
					Ev = Ev_samples.front();
				}
				
				std::cout << "Current Ev:" 
						  << "\n x: " << Ev.getX() 
						  << " y: " << Ev.getY()
						  << " z: " << Ev.getZ()
						  << std::endl;
				
				
				
				//std::cout << "Evs:";
				//for (tf::Vector3 ii : Evs)
				//{
					//std::cout << "\n\tx:\t" << ii.getX()
							  //<< "\n\ty:\t" << ii.getY()
							  //<< "\n\tz:\t" << ii.getZ();
				//}
				//std::cout << std::endl;
				
				////////// end finding Ev //////////
				
				
				////////// begin finding Phi //////////
				// phi has already been calculated earlier so just need to put onto the deque then integrate
				shift_sample_set(Phi_samples, phi, number_of_integrating_samples);
				shift_sample_set(Phi_timestamps, current_timestamp, number_of_integrating_samples);
				
				//std::cout << "Phi samples:";
				//for (tf::Vector3 ii : Phi_samples)
				//{
					//std::cout << "\n\tx:\t" << ii.getX()
							  //<< "\n\ty:\t" << ii.getY()
							  //<< "\n\tz:\t" << ii.getZ();
				//}
				//std::cout << std::endl;
				
				//std::cout << "Phi timestamps:";
				//for (double ii : Phi_timestamps)
				//{
					//std::cout << "\n\tt:\t" << ii;
				//}
				//std::cout << std::endl;
				
				// getting the integral to calculate Phi
				calculate_integral(Phi, Phi_samples, Phi_timestamps);
				std::cout << "Current Phi:" 
						  << "\n x: " << Phi.getX() 
						  << " y: " << Phi.getY()
						  << " z: " << Phi.getZ()
						  << std::endl;
				
				//std::cout << "Phis:";
				//for (tf::Vector3 ii : Phis)
				//{
					//std::cout << "\n\tx:\t" << ii.getX()
							  //<< "\n\ty:\t" << ii.getY()
							  //<< "\n\tz:\t" << ii.getZ();
				//}
				//std::cout << std::endl;
				
				////////// end finding Phi //////////
				
				////////// begin finding vc //////////
				// inverse of Lv
				Lv_inv = Lv.inverse();
				
				// temp calc of [Kv*ev + phi*zr_star_hat]
				vc_tempL.setX(K_v.getRow(0).dot(ev) + zr_star_hat*phi.getX());
				vc_tempL.setY(K_v.getRow(1).dot(ev) + zr_star_hat*phi.getY());
				vc_tempL.setZ(K_v.getRow(2).dot(ev) + zr_star_hat*phi.getZ());
				//std::cout << " Kv(0)*ev:\t" << K_v.getRow(0).dot(ev) << std::endl;
				
				
				// calc of vc
				vc.setX((1.0/alpha_red)*(Lv_inv.getRow(0).dot(vc_tempL)));
				vc.setY((1.0/alpha_red)*(Lv_inv.getRow(1).dot(vc_tempL)));
				vc.setZ((1.0/alpha_red)*(Lv_inv.getRow(2).dot(vc_tempL)));
				std::cout << "Current vc:" 
						  << "\n x: " << vc.getX() 
						  << "\n\ty: " << vc.getY()
						  << "\n\tz: " << vc.getZ()
						  << std::endl;
				std::cout << "Current wc:" 
						  << "\n x: " << wc.getX() 
						  << " y: " << wc.getY()
						  << " z: " << wc.getZ()
						  << std::endl;
				
				////////// end finding vc //////////
				
				////////// begin finding U //////////
				// using a new variable uvar to represent the multiplication used to get the integral for U
				// Uvar = alpha_red*Lv*vc
				Uvar.setX(alpha_red*(Lv.getRow(0).dot(vc)));
				Uvar.setY(alpha_red*(Lv.getRow(1).dot(vc)));
				Uvar.setZ(alpha_red*(Lv.getRow(2).dot(vc)));
				
				// putting the Uvars onto the Uvar deque
				shift_sample_set(Uvar_samples,Uvar,number_of_integrating_samples);
				shift_sample_set(Uvar_timestamps, current_timestamp, number_of_integrating_samples);
				
				// getting U, the integral of Uvar_samples
				calculate_integral(U, Uvar_samples, Uvar_timestamps);
				std::cout << "Current U:"
						  << "\n x: " << U.getX()
						  << " y: " << U.getY()
						  << " z: " << U.getZ()
						  << std::endl;
				
				
				////////// end finding U //////////
				
				// find the new estimate
				calculate_zr_star_hat_dot();
				
				// putting the zr_star_hat_dot onto the deque
				shift_sample_set(zr_star_hat_dot_samples, zr_star_hat_dot, number_of_integrating_samples);
				shift_sample_set(zr_star_hat_dot_timestamps, current_timestamp, number_of_integrating_samples);
				
				// updating zr_star_hat to be the integral of zr_star_hat_dot
				calculate_integral(zr_star_hat, zr_star_hat_dot_samples, zr_star_hat_dot_timestamps);
				std::cout << "current zr_star_hat: " << zr_star_hat <<std::endl;
				

				/********** end calculating the depth esitimate **********/
				
				// The coordinate frame of the body of the quad relative to the camera is xb = -yc, yb = -xc, and zb = -zc
				// angular zb is angular zc, that is angular zb is positive about zc, strange
				
				// output velocity from tracking message
				command_from_tracking.linear.x = -1.0*vc.getY();
				command_from_tracking.linear.y = -1.0*vc.getX();
				command_from_tracking.linear.z = -1.0*vc.getZ();
				command_from_tracking.angular.z = wc.getZ();
				
			}
			else
			{
				std::cout << "negative alpha_red" << std::endl;
				// output velocity from tracking message as zeros
				command_from_tracking.linear.x = 0;
				command_from_tracking.linear.y = 0;
				command_from_tracking.linear.z = 0;
				command_from_tracking.angular.z = 0;
			}
		}
		// calculates zr_star_hat_dot, if ||EV(t)-PHI(t) || is greater than the minimum
		// of the norms of the history stack will add it on
		void calculate_zr_star_hat_dot()
		{
			//// find the minimum norm of the stack
			//std::cout << "before addition" << std::endl;
			
			//for (int ii = 0; ii < Evs_minus_Phis.size(); ii++)
			//{
				//std::cout << "value at " << ii
						  //<< "\nEvs - Phis:" 
						  //<< "\n\tx:\t" << Evs_minus_Phis[ii].getX() 
						  //<< "\n\ty:\t" << Evs_minus_Phis[ii].getY()
						  //<< "\n\tz:\t" << Evs_minus_Phis[ii].getZ()
						  //<< std::endl;
				//std::cout << "Norm:\t" << Evs_minus_Phis_norms[ii] 
						  //<< std::endl;
				//std::cout << "Us:" 
						  //<< "\n\tx:\t" << Us[ii].getX() 
						  //<< "\n\ty:\t" << Us[ii].getY()
						  //<< "\n\tz:\t" << Us[ii].getZ()
						  //<< std::endl;
			//}
			
			// Evs - Phi
			Ev_minus_Phi = Ev - Phi;
			Ev_minus_Phi_norm = Ev_minus_Phi.length();
			
			std::cout << "Evs - Phis:" 
					  << "/n x: " << Ev_minus_Phi.getX() 
					  << " y: " << Ev_minus_Phi.getY()
					  << " z: " << Ev_minus_Phi.getZ()
					  << std::endl;
			std::cout << "Norm: " << Ev_minus_Phi_norm
					  << std::endl;
			std::cout << "Us:" 
					  << "\n x: " << U.getX() 
					  << " y: " << U.getY()
					  << " z: " << U.getZ()
					  << std::endl;
			
			// finding the minimum value
			curr_Ev_minus_Phi_norm_min = *std::min_element(Evs_minus_Phis_norms.begin(),Evs_minus_Phis_norms.end());
			std::cout << "current min norm: " << curr_Ev_minus_Phi_norm_min << std::endl;
			
			// getting an iterator to the minimum value
			curr_Ev_minus_Phi_norm_min_iter = std::find(Evs_minus_Phis_norms.begin(),Evs_minus_Phis_norms.end(),curr_Ev_minus_Phi_norm_min);
			std::cout << "current min norm from iterator: " << *curr_Ev_minus_Phi_norm_min_iter << std::endl;
			
			// getting the index of the iterator
			curr_Ev_minus_Phi_norm_min_index = std::distance(Evs_minus_Phis_norms.begin(),curr_Ev_minus_Phi_norm_min_iter);
			std::cout << "current index of the min norm: " << curr_Ev_minus_Phi_norm_min_index << std::endl;
			
			// if the current norm is greater than the minimum norm of the history will remove the old min and add the new one to the end
			if ( Ev_minus_Phi_norm > *curr_Ev_minus_Phi_norm_min_iter )
			{
				// removing the previous min from the history
				Evs_minus_Phis.erase(Evs_minus_Phis.begin() + curr_Ev_minus_Phi_norm_min_index);
				Evs_minus_Phis_norms.erase(Evs_minus_Phis_norms.begin() + curr_Ev_minus_Phi_norm_min_index);
				Us.erase(Us.begin() + curr_Ev_minus_Phi_norm_min_index);
				
				// adding the new norm values
				shift_sample_set(Evs_minus_Phis, Ev_minus_Phi, number_of_samples);
				shift_sample_set(Evs_minus_Phis_norms, Ev_minus_Phi_norm, number_of_samples);
				shift_sample_set(Us, U, number_of_samples);
			}
			
			//std::cout << "after addition" << std::endl;
			//for (int ii = 0; ii < Evs_minus_Phis.size(); ii++)
			//{
				//std::cout << "value at " << ii
						  //<< "\nEvs - Phis:" 
						  //<< "\n\tx:\t" << Evs_minus_Phis[ii].getX() 
						  //<< "\n\ty:\t" << Evs_minus_Phis[ii].getY()
						  //<< "\n\tz:\t" << Evs_minus_Phis[ii].getZ()
						  //<< std::endl;
				//std::cout << "Norm:\t" << Evs_minus_Phis_norms[ii] 
						  //<< std::endl;
				//std::cout << "Us:" 
						  //<< "\n\tx:\t" << Us[ii].getX() 
						  //<< "\n\ty:\t" << Us[ii].getY()
						  //<< "\n\tz:\t" << Us[ii].getZ()
						  //<< std::endl;
			//}
			
			// getting the summation
			zr_star_hat_dot_sum = 0;
			
			std::cout << "current zr_star_hat: " << zr_star_hat << std::endl;
			
			// sum_from_k=1_to_n( transpose(Ev(k)-Phi(k))*(-(Ev(k)-Phi(k))*z_star_hat - U(k)) )
			for (int ii = 0; ii < Evs_minus_Phis.size(); ii++)
			{
				zr_star_hat_dot_R = -1.0*(Evs_minus_Phis[ii])*zr_star_hat - Us[ii];
				//std::cout << "current zr_star_hat_dot_R at:\t" << ii 
						  //<< "\n\tx:\t" << zr_star_hat_dot_R.getX() 
						  //<< "\n\ty:\t" << zr_star_hat_dot_R.getY()
						  //<< "\n\tz:\t" << zr_star_hat_dot_R.getZ()
						  //<< std::endl;

				zr_star_hat_dot_sum += (Evs_minus_Phis[ii]).dot(zr_star_hat_dot_R);
				
			}
			
			//z_star_hat_dot = gamma_1*transpose(ev)*phi + gamma1*gamma2*zr_star_hat_dot_sum
			zr_star_hat_dot = gamma_1*ev.dot(phi) + gamma_1*gamma_2*zr_star_hat_dot_sum;
			std::cout << "current zr_star_hat_dot: " << zr_star_hat_dot << std::endl;
			std::cout << "current zr_star_hat_dot from ev and phi: " << gamma_1*ev.dot(phi) << std::endl;
			std::cout << "current zr_star_hat_dot from sum: " << gamma_1*gamma_2*zr_star_hat_dot_sum << std::endl;
			
			// putting the zr_star_hat_dot onto the deque
			shift_sample_set(zr_star_hat_dot_samples, zr_star_hat_dot, number_of_samples);
			shift_sample_set(zr_star_hat_dot_timestamps, current_timestamp, number_of_samples);
			
			
			
		}
		
		// function to calculate the integral of using the trapizoidal rule
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
		
		// function to calculate the integral of using the trapizoidal rule
		void calculate_integral(double &integral, std::deque<double> &function_values, std::deque<double> &time_values)
		{
			integral = 0;
			// if the two deques are the same size will do the integral otherwise wont and will return all -1's in the integral
			if (function_values.size() == time_values.size())
			{
				// if there is more than one element in the deque then will do the integral over the points, otherwise will just return it for the one
				if (function_values.size() > 1)
				{
					std::cout << "start integrating";
					for (int ii = 0; ii < function_values.size() - 1; ii++)
					{
						integral += 0.5*(time_values[ii+1] - time_values[ii]) * (function_values[ii+1] + function_values[ii]);
						//std::cout << "\ncurrent integral:\t" << integral 
								  //<< "\n\tlast time:\t" << time_values[ii+1]
								  //<< "\n\tcurrent time:\t" << time_values[ii]
								  //<< "\n\tlast value:\t" << function_values[ii+1]
								  //<< "\n\tcurrent value:\t" << function_values[ii];
					}
					std::cout << "\nend integrating" << std::endl;
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
			current_zref.data = zr_star_hat;
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
			z_ref_pub.publish(current_zref);
		}
		
		
};

// main
int main(int argc, char** argv)
{   
	// initialize node
	ros::init(argc,argv,"quad_controller_node");
	
	// initialize the controller    
	QuadController quad_controller;
	
	ros::Rate loop_rate(30);
	
	while (ros::ok())
	{
		// generate a new command based on the decomp
		quad_controller.output_velocity_command();
		// release and sleep until next time image is available
		ros::spinOnce();
		loop_rate.sleep();
	}
	
    
    return 0;
}
