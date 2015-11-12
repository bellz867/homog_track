#include <iostream>
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
#include <homog_track/HomogMarker.h>
#include <homog_track/HomogDecomposed.h>
#include <homog_track/HomogComplete.h>
#include <homog_track/HomogReference.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

// function to push values onto a deque and if it is over some length will remove the first and then push it
template <typename sample_set_type> void shift_sample_set(std::deque<sample_set_type> &sample_set, sample_set_type &new_sample)
{
	sample_set.pop_front();
	sample_set.push_back(new_sample);
}

// class with everything
class Simulator
{
	public:
		bool display_calc_steps = true;
		double loop_rate_hz = 30;
		double step_size = 1/loop_rate_hz;
		double integration_window = 0.2;
		bool first_run = true;
		ros::Time last_time;// last time the camera was updated
		ros::Time current_time;// current time the camera was updated
		ros::Time start_time;// start time
		
		/********** Topic Declarations **********/
		tf::TransformBroadcaster br;
		tf::TransformListener listener;
		
		/********** File Declarations **********/
		bool write_to_file = true;
		std::fstream output_file;
		std::string output_file_name = "/home/zack/v1_ws/src/homog_track/testing_files/test_all_slow_exp_cpp.txt"; // file name
		
		/********** Gains **********/
		double Kws = 1;// K_w scalar
		tf::Matrix3x3 Kw = tf::Matrix3x3(Kws,0,0,
										 0,Kws,0,
										 0,0,Kws);// rotational gain matrix initialize to identity
		double Kvs = 1;
		tf::Matrix3x3 Kv = tf::Matrix3x3(Kvs,0,0,
										 0,Kvs,0,
										 0,0,Kvs);// linear velocity gain matrix
		double gamma_1 = std::pow(5,1);// control gains for the z_star_hat_dot calculation
		double gamma_2 = std::pow(0.3,1);
		//double gamma_2 = 0;
		double zr_star_hat = 10;// current value for zr_star_hat
		
		/********** Points Declarations **********/
		tf::Vector3 P_red_wrt_world, P_green_wrt_world, P_cyan_wrt_world, P_purple_wrt_world;// homography points wrt world
		tf::Transform red_wrt_world, green_wrt_world, cyan_wrt_world, purple_wrt_world;//transforms for the marker points
		
		/********** Reference Declarations **********/
		tf::Vector3 mr_bar_ref, mg_bar_ref, mc_bar_ref, mp_bar_ref;// points wrt reference
		tf::Vector3 pr_ref, pg_ref, pc_ref, pp_ref;// pixels wrt reference
		tf::Transform reference_wrt_world; // transform for reference camera
		
		/********** Camera Declarations **********/
		tf::Vector3 mr_bar, mg_bar, mc_bar, mp_bar;// points wrt camera
		tf::Vector3 pr, pg, pc, pp;// pixels wrt camera
		tf::Vector3 mr;
		tf::Transform camera_wrt_world; // transform for camera
		tf::Matrix3x3 K = tf::Matrix3x3(1,0,0,
										0,1,0,
										0,0,1);// camera matrix
		
		/********** Decomp Declarations **********/
		tf::Transform camera_wrt_reference;// transform of camera wrt reference
		tf::Quaternion Q_cf = tf::Quaternion(0,0,0,0), Q_cf_last = tf::Quaternion(0,0,0,0), Q_cf_negated = tf::Quaternion(0,0,0,0);// camera wrt reference, last camera wrt reference, and negated camera wrt reference
		double Q_norm_current_diff, Q_norm_negated_diff;// norms to determine which rotation is closer to last
		double alpha_red, alpha_green, alpha_cyan, alpha_purple;// alphas for the camera
		
		/********** Desired Declarations **********/
		tf::Vector3 mrd_bar, mgd_bar, mcd_bar, mpd_bar;// points wrt desired
		tf::Vector3 mrd;
		tf::Vector3 prd, pgd, pcd, ppd;// pixels wrt desired
		tf::Transform desired_wrt_world;// transform for desired wrt world
		tf::Quaternion Q_dw = tf::Quaternion(0,0,0,0), Q_df = tf::Quaternion(0,0,0,0), Q_df_negated = tf::Quaternion(0,0,0,0), Q_df_last = tf::Quaternion(0,0,0,0);// desired wrt reference, last desired wrt reference, and negated desired wrt reference
		cv::Mat wcd_cv;// desired angular velocity
		tf::Vector3 vcd;// desired linear velocity
		double desired_radius;// desired radius
		double desired_period;// desired period
		double alpha_red_d;
		
		/********** Controller Declarations **********/	
		/********** Camera Controller **********/
		tf::Quaternion Q_wc_temp;
		tf::Vector3 wc_temp;
		cv::Mat wc_cv;
		tf::Vector3 wc;// rotation of camera wrt reference
		tf::Matrix3x3 mr_mat, ss_mr;//matrix for Lv calc and ss
		
		/********** Desired Controller **********/
		tf::Quaternion Q_wcd ;// desired angular velocity as a quaternion
		tf::Vector3 wcd;
		tf::Matrix3x3 mrd_mat, ss_mrd;// matrix for Lvd calc and ss mrd
		
		/********** Wc calc terms Controller **********/
		tf::Quaternion Q_error;// rotational error
		tf::Vector3 Q_error_v;// rotational error vector porition
		
		/********** Vc calc terms Controller **********/
		tf::Vector3 ev = tf::Vector3(0,0,0), pe = tf::Vector3(0,0,0), ped = tf::Vector3(0,0,0); // value for calculating the current ev, that is pe-ped
		tf::Matrix3x3 Lv = tf::Matrix3x3(0,0,0,0,0,0,0,0,0), Lv_term1 = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);// Lv
		tf::Matrix3x3 Lvd = tf::Matrix3x3(0,0,0,0,0,0,0,0,0), Lvd_term1 = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);// Lvd
		tf::Vector3 ped_dot_term1 = tf::Vector3(0,0,0), ped_dot_term2 = tf::Vector3(0,0,0), ped_dot = tf::Vector3(0,0,0);//ped_dot
		tf::Vector3 phi_term1 = tf::Vector3(0,0,0), phi = tf::Vector3(0,0,0);// phi
		
		/********** Stack and buffers Controller **********/
		const int number_of_samples = 20;// number of samples for stack
		const int number_of_integrating_samples = loop_rate_hz*0.2;// number of integrating samples
		tf::Vector3 Ev = tf::Vector3(0,0,0); tf::Vector3 Phi = tf::Vector3(0,0,0); tf::Vector3 U = tf::Vector3(0,0,0); tf::Vector3 Uvar = tf::Vector3(0,0,0);// current value for Ev, Phi, U, and Uvar
		tf::Matrix3x3 principle_point = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);// matrix with the principle point coordinates of the camera
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
		
		/********** zr hat estimate Controller **********/
		double zr_star_hat_dot = 0;// current value for zr_star_hat_dot = gamma1*transpose(ev)*phi + gamma1*gamma2*sum_from_k=1_to_n( transpose(Ev(k)-Phi(k))*(-(Ev(k)-Phi(k))*zr_star_hat - U(k)) )
		double zr_star_hat_dot_sum = 0;// temp for the summation in zr_star_hat_dot
		bool first_time_update = true; // first time update occurance
		bool first_desired_recieved = false, first_marker_recieved = false, first_decomp_recieved = false; // first messages recieved
		bool desired_recieved = false, marker_recieved = false, decomp_recieved = false; // messages recieved
		
	
		
		Simulator()
		{	
			/******************** Writing headers to file ********************/
			if (write_to_file)
			{
				output_file.open(output_file_name, std::fstream::out | std::fstream::app);
			
				if (output_file.is_open())
				{
					output_file << "prx," << "pry," << "prz,"
								<< "prdx," << "prdy," << "prdz,"
								<< "mrx," << "mry," << "mrz,"
								<< "mrdx," << "mrdy," << "mrdz,"
								<< "pex," << "pey," << "pez,"
								<< "pedx," << "pedy," << "pedz,"
								<< "qw," << "qx," << "qy," << "qz,"
								<< "qdw," << "qdx," << "qdy," << "qdz,"
								<< "qtildew," << "qtildex," << "qtildey," << "qtildez,"
								<< "evx," << "evy," << "evz,"
								<< "phix," << "phiy," << "phiz,"
								<< "vcx," << "vcy," << "vcz,"
								<< "wcx," << "wcy," << "wcz,"
								<< "vcdx," << "vcdy," << "vcdz,"
								<< "wcdx," << "wcdy," << "wcdz,"
								<< "Evx," << "Evy," << "Evz,"
								<< "Phix," << "Phiy," << "Phiz,"
								<< "Uvarx," << "Uvary," << "Uvarz,"
								<< "EvPhix," << "EvPhix," << "EvPhix,"
								<< "EvPhisvd,"
								<< "Ux," << "Uy," << "Uz,"
								<< "z_hat_dot,"
								<< "z_hat"
								<< "\n";
					output_file.close();
				}
			}
			
			
			/********** markers wrt world **********/
			P_red_wrt_world = tf::Vector3(-0.05,-0.05,0); P_green_wrt_world = tf::Vector3(-0.05,0.05,0); P_cyan_wrt_world = tf::Vector3(0.05,0.05,0); P_purple_wrt_world = tf::Vector3(0.05,-0.05,0);// homography points wrt world
			red_wrt_world.setIdentity(); red_wrt_world.setOrigin(P_red_wrt_world);//red
			green_wrt_world.setIdentity(); green_wrt_world.setOrigin(P_green_wrt_world);//green
			cyan_wrt_world.setIdentity(); cyan_wrt_world.setOrigin(P_cyan_wrt_world);//cyan
			purple_wrt_world.setIdentity(); purple_wrt_world.setOrigin(P_purple_wrt_world);//purple

			/********** reference wrt world  **********/
			double z_ref = 2; //reference height
			reference_wrt_world.setOrigin(tf::Vector3(0,0,z_ref));//origin
			tf::Matrix3x3 R_fw(0,1,0,
								  1,0,0,
								  0,0,-1);// rotation of reference wrt world
			tf::Quaternion Q_fw;// as a quaternion
			R_fw.getRotation(Q_fw);// initialize quaternion
			reference_wrt_world.setRotation(Q_fw);// set the rotation
			
			/********** markers wrt reference **********/
			tf::Vector3 temp_v;
			tf::Quaternion temp_Q;
			temp_v = P_red_wrt_world-reference_wrt_world.getOrigin(); temp_Q = ((reference_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*reference_wrt_world.getRotation(); mr_bar_ref = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-reference_wrt_world.getOrigin(); temp_Q = ((reference_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*reference_wrt_world.getRotation(); mg_bar_ref = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-reference_wrt_world.getOrigin(); temp_Q = ((reference_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*reference_wrt_world.getRotation(); mc_bar_ref = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-reference_wrt_world.getOrigin(); temp_Q = ((reference_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*reference_wrt_world.getRotation(); mp_bar_ref = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			pr_ref = K*((1/mr_bar_ref.getZ())*mr_bar_ref); pg_ref = K*((1/mg_bar_ref.getZ())*mg_bar_ref); pc_ref = K*((1/mc_bar_ref.getZ())*mc_bar_ref); pp_ref = K*((1/mp_bar_ref.getZ())*mp_bar_ref);// reference points as pixels
			
			/********** camera wrt world  **********/
			double z_init = 3; //starting camera height
			double cam_a0 = M_PIl/2;
			camera_wrt_world.setOrigin(tf::Vector3(1,1,z_init));//origin
			tf::Matrix3x3 R_cf(std::cos(cam_a0),-std::sin(cam_a0),0,
								  std::sin(cam_a0),std::cos(cam_a0),0,
								  0,0,1);// rotation of camera wrt reference
			tf::Matrix3x3 R_cw = R_fw*R_cf;// rotation of camera wrt world
			tf::Quaternion Q_cw;// as a quaternion
			R_cw.getRotation(Q_cw);// initialize quaternion
			camera_wrt_world.setRotation(Q_cw);// set the rotation
			
			/********** pixels wrt camera **********/
			temp_v = P_red_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mr_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mg_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mc_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mp_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			pr = K*((1/mr_bar.getZ())*mr_bar); pg = K*((1/mg_bar.getZ())*mg_bar); pc = K*((1/mc_bar.getZ())*mc_bar); pp = K*((1/mp_bar.getZ())*mp_bar);// camera points as pixels
			alpha_red = mr_bar_ref.getZ()/mr_bar.getZ();
			
			/********** desired wrt world **********/
			double zd_init = 2; //starting desired height
			double cam_des_a0 = 0;// starting desired angle
			desired_radius = 1;// desired radius in meters
			desired_period = 30;
			desired_wrt_world.setOrigin(tf::Vector3(desired_radius,0,zd_init));//origin
			tf::Matrix3x3 R_df(std::cos(cam_des_a0),-std::sin(cam_des_a0),0,
							   std::sin(cam_des_a0),std::cos(cam_des_a0),0,
							   0,0,1);// rotation of desired wrt reference
			tf::Matrix3x3 R_dw = R_fw*R_df;// rotation of camera wrt world
			tf::Quaternion Q_dw;// as a quaternion
			R_dw.getRotation(Q_dw);// initialize quaternion
			desired_wrt_world.setRotation(Q_dw);// set the rotation
			wcd_cv = cv::Mat::zeros(3,1,CV_64F);
			wcd_cv.at<double>(2,0) = -2*M_PIl/desired_period;
			//wcd.at<double>(2,0) = 0;
			wcd = tf::Vector3(0,0,wcd_cv.at<double>(2,0));
			vcd = tf::Vector3(2*M_PIl*desired_radius/desired_period,0,0);
			//vcd = tf::Vector3(0,0,0);
			
			/********** pixels wrt desired  **********/
			temp_v = P_red_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mrd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mgd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mcd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mpd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			prd = K*((1/mrd_bar.getZ())*mrd_bar); pgd = K*((1/mgd_bar.getZ())*mgd_bar); pcd = K*((1/mcd_bar.getZ())*mcd_bar); ppd = K*((1/mpd_bar.getZ())*mpd_bar);// camera points as pixels
			alpha_red_d = mr_bar_ref.getZ()/mrd_bar.getZ();
			
			/********* update time and reference*********/
			last_time = ros::Time::now();
			br.sendTransform(tf::StampedTransform(reference_wrt_world, last_time
							,"world", "reference_image"));
				
			/********* camera wrt reference *********/
			br.sendTransform(tf::StampedTransform(camera_wrt_world, last_time
							,"world", "current_image"));
			tf::StampedTransform camera_wrt_reference;
			listener.waitForTransform("reference_image", "current_image"
									 ,last_time, ros::Duration(1.0));
			listener.lookupTransform("reference_image", "current_image"
									 ,last_time, camera_wrt_reference);
			Q_cf = camera_wrt_reference.getRotation();
			Q_cf_negated = tf::Quaternion(-Q_cf.getX(),-Q_cf.getY(),-Q_cf.getZ(),-Q_cf.getW()); // getting the negated version of the quaternion for the check
			// checking if the quaternion has flipped
			double Q_norm_camera_diff = std::sqrt(std::pow(Q_cf.getX() - Q_cf_last.getX(),2.0)
										  + std::pow(Q_cf.getY() - Q_cf_last.getY(),2.0) 
										  + std::pow(Q_cf.getZ() - Q_cf_last.getZ(),2.0) 
										  + std::pow(Q_cf.getW() - Q_cf_last.getW(),2.0));
			double Q_norm_camera_neg_diff = std::sqrt(std::pow(Q_cf_negated.getX() - Q_cf_last.getX(),2.0)
										  + std::pow(Q_cf_negated.getY() - Q_cf_last.getY(),2.0) 
										  + std::pow(Q_cf_negated.getZ() - Q_cf_last.getZ(),2.0) 
										  + std::pow(Q_cf_negated.getW() - Q_cf_last.getW(),2.0));
			if (Q_norm_camera_diff > Q_norm_camera_neg_diff)
			{
				Q_cf = Q_cf_negated;
			}
			Q_cf_last = Q_cf;// updating the last
			camera_wrt_reference.setRotation(Q_cf);
			
			/********* desired wrt reference *********/
			br.sendTransform(tf::StampedTransform(desired_wrt_world, last_time
							,"world","desired_image")); 
			tf::StampedTransform desired_wrt_reference;
			listener.waitForTransform("reference_image", "desired_image"
									 ,last_time, ros::Duration(1.0));
			listener.lookupTransform("reference_image", "desired_image"
									,last_time, desired_wrt_reference);
			Q_df = desired_wrt_reference.getRotation();
			Q_df_negated = tf::Quaternion(-Q_df.getX(),-Q_df.getY(),-Q_df.getZ(),-Q_df.getW()); // getting the negated version of the quaternion for the check
			// checking if the quaternion has flipped
			double Q_norm_desired_diff = std::sqrt(std::pow(Q_df.getX() - Q_df_last.getX(),2.0)
										  + std::pow(Q_df.getY() - Q_df_last.getY(),2.0) 
										  + std::pow(Q_df.getZ() - Q_df_last.getZ(),2.0) 
										  + std::pow(Q_df.getW() - Q_df_last.getW(),2.0));
			double Q_norm_desired_neg_diff = std::sqrt(std::pow(Q_df_negated.getX() - Q_df_last.getX(),2.0)
										  + std::pow(Q_df_negated.getY() - Q_df_last.getY(),2.0) 
										  + std::pow(Q_df_negated.getZ() - Q_df_last.getZ(),2.0) 
										  + std::pow(Q_df_negated.getW() - Q_df_last.getW(),2.0));
			if (Q_norm_desired_diff > Q_norm_desired_neg_diff)
			{
				Q_df = Q_df_negated;
			}
			
			Q_df_last = Q_df;// updating the last
			desired_wrt_reference.setRotation(Q_df);
			
			if (Q_norm_camera_diff > Q_norm_camera_neg_diff)
			{
				Q_cf = Q_cf_negated;
			}
			Q_cf_last = Q_cf;// updating the last
			camera_wrt_reference.setRotation(Q_cf);
			
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
			
			// initializing the integrating samples to 0
			for (int ii = 0; ii < number_of_integrating_samples; ii++)
			{
				trapz_timestamps.push_back(0.0);
				Ev_samples.push_back(tf::Vector3(0,0,0));
				Phi_samples.push_back(tf::Vector3(0,0,0));
				Uvar_samples.push_back(tf::Vector3(0,0,0));
			}
			
		}
		
		// update camera
		void update_camera_pixels()
		{
			double time_diff = current_time.toSec() - last_time.toSec();// get the time difference
			wc_cv = cv::Mat::zeros(3,1,CV_64F); wc_cv.at<double>(2,0) = wc.getZ();// angular velocity of camera wrt reference expressed in camera
			tf::Quaternion Q_cw = camera_wrt_world.getRotation();// rotation of camera wrt world
			cv::Mat Q_cw_old = cv::Mat::zeros(4,1,CV_64F);
			Q_cw_old.at<double>(0,0) = Q_cw.getW(); Q_cw_old.at<double>(1,0) = Q_cw.getX(); Q_cw_old.at<double>(2,0) = Q_cw.getY(); Q_cw_old.at<double>(3,0) = Q_cw.getZ();
			cv::Mat B = cv::Mat::zeros(4,3,CV_64F);// differential matrix
			B.at<double>(0,0) = -Q_cw.getX(); B.at<double>(0,1) = -Q_cw.getY(); B.at<double>(0,2) = -Q_cw.getZ();
			B.at<double>(1,0) = Q_cw.getW(); B.at<double>(1,1) = -Q_cw.getZ(); B.at<double>(1,2) = Q_cw.getY();
			B.at<double>(2,0) = Q_cw.getZ(); B.at<double>(2,1) = Q_cw.getW(); B.at<double>(2,2) = -Q_cw.getX();
			B.at<double>(3,0) = -Q_cw.getY(); B.at<double>(3,1) = Q_cw.getX(); B.at<double>(3,2) = Q_cw.getW();
			cv::Mat Q_cw_dot = 0.5*(B*wc_cv);// rate of change of rotation of camera wrt world
			cv::Mat Q_cw_new = cv::Mat::zeros(4,1,CV_64F);
			Q_cw_new = Q_cw_old + Q_cw_dot*time_diff;//update the quaternion
			double Q_cw_new_norm = std::sqrt(std::pow(Q_cw_new.at<double>(0,0),2)+
											 std::pow(Q_cw_new.at<double>(1,0),2)+
											 std::pow(Q_cw_new.at<double>(2,0),2)+
											 std::pow(Q_cw_new.at<double>(3,0),2));
			Q_cw_new = (1.0/Q_cw_new_norm)*Q_cw_new;//normalize
			Q_cw = tf::Quaternion(Q_cw_new.at<double>(1,0),Q_cw_new.at<double>(2,0),Q_cw_new.at<double>(3,0),Q_cw_new.at<double>(0,0));//updating
			camera_wrt_world.setRotation(Q_cw);//updating the transform
			tf::Quaternion Q_P_cw_dot = (Q_cw*tf::Quaternion(vc.getX(),vc.getY(),vc.getZ(),0.0))*Q_cw.inverse();//express vc in world frame
			tf::Vector3 P_cw_dot = tf::Vector3(Q_P_cw_dot.getX(),Q_P_cw_dot.getY(),Q_P_cw_dot.getZ());
			tf::Vector3 P_cw = camera_wrt_world.getOrigin();//origin of camera wrt world
			tf::Vector3 P_cw_new = P_cw + P_cw_dot*time_diff;//update origin
			camera_wrt_world.setOrigin(P_cw_new);//updating the transform
			
			/********** update pixels wrt camera  **********/
			tf::Vector3 temp_v;
			tf::Quaternion temp_Q;
			temp_v = P_red_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mr_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mg_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mc_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mp_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			pr = K*((1/mr_bar.getZ())*mr_bar); pg = K*((1/mg_bar.getZ())*mg_bar); pc = K*((1/mc_bar.getZ())*mc_bar); pp = K*((1/mp_bar.getZ())*mp_bar);// camera points as pixels
			alpha_red = mr_bar_ref.getZ()/mr_bar.getZ();
			
			/********* camera wrt reference *********/
			br.sendTransform(tf::StampedTransform(camera_wrt_world, current_time
							,"world", "current_image"));
			tf::StampedTransform camera_wrt_reference;
			listener.waitForTransform("reference_image", "current_image"
									 ,current_time, ros::Duration(1.0));
			listener.lookupTransform("reference_image", "current_image"
									 ,current_time, camera_wrt_reference);
			Q_cf = camera_wrt_reference.getRotation();
			Q_cf_negated = tf::Quaternion(-Q_cf.getX(),-Q_cf.getY(),-Q_cf.getZ(),-Q_cf.getW()); // getting the negated version of the quaternion for the check
			// checking if the quaternion has flipped
			double Q_norm_camera_diff = std::sqrt(std::pow(Q_cf.getX() - Q_cf_last.getX(),2.0)
										  + std::pow(Q_cf.getY() - Q_cf_last.getY(),2.0) 
										  + std::pow(Q_cf.getZ() - Q_cf_last.getZ(),2.0) 
										  + std::pow(Q_cf.getW() - Q_cf_last.getW(),2.0));
			double Q_norm_camera_neg_diff = std::sqrt(std::pow(Q_cf_negated.getX() - Q_cf_last.getX(),2.0)
										  + std::pow(Q_cf_negated.getY() - Q_cf_last.getY(),2.0) 
										  + std::pow(Q_cf_negated.getZ() - Q_cf_last.getZ(),2.0) 
										  + std::pow(Q_cf_negated.getW() - Q_cf_last.getW(),2.0));
			if (Q_norm_camera_diff > Q_norm_camera_neg_diff)
			{
				Q_cf = Q_cf_negated;
			}
			Q_cf_last = Q_cf;// updating the last
			camera_wrt_reference.setRotation(Q_cf);
			
		}
		
		// update desired
		void update_desired_pixels()
		{
			double time_diff = current_time.toSec() - last_time.toSec();// get the time difference
			Q_dw = desired_wrt_world.getRotation();// rotation of camera wrt world
			cv::Mat Q_dw_old = cv::Mat::zeros(4,1,CV_64F);
			Q_dw_old.at<double>(0,0) = Q_dw.getW(); Q_dw_old.at<double>(1,0) = Q_dw.getX(); Q_dw_old.at<double>(2,0) = Q_dw.getY(); Q_dw_old.at<double>(3,0) = Q_dw.getZ();
			cv::Mat Bd = cv::Mat::zeros(4,3,CV_64F);// differential matrix
			Bd.at<double>(0,0) = -Q_dw.getX(); Bd.at<double>(0,1) = -Q_dw.getY(); Bd.at<double>(0,2) = -Q_dw.getZ();
			Bd.at<double>(1,0) = Q_dw.getW(); Bd.at<double>(1,1) = -Q_dw.getZ(); Bd.at<double>(1,2) = Q_dw.getY();
			Bd.at<double>(2,0) = Q_dw.getZ(); Bd.at<double>(2,1) = Q_dw.getW(); Bd.at<double>(2,2) = -Q_dw.getX();
			Bd.at<double>(3,0) = -Q_dw.getY(); Bd.at<double>(3,1) = Q_dw.getX(); Bd.at<double>(3,2) = Q_dw.getW();
			cv::Mat Q_dw_dot = 0.5*(Bd*wcd_cv);// rate of change of rotation of camera wrt world
			cv::Mat Q_dw_new = cv::Mat::zeros(4,1,CV_64F);
			Q_dw_new = Q_dw_old + Q_dw_dot*time_diff;//update the quaternion
			double Q_dw_new_norm = std::sqrt(std::pow(Q_dw_new.at<double>(0,0),2)+
											 std::pow(Q_dw_new.at<double>(1,0),2)+
											 std::pow(Q_dw_new.at<double>(2,0),2)+
											 std::pow(Q_dw_new.at<double>(3,0),2));
			Q_dw_new = (1.0/Q_dw_new_norm)*Q_dw_new;//normalize
			Q_dw = tf::Quaternion(Q_dw_new.at<double>(1,0),Q_dw_new.at<double>(2,0),Q_dw_new.at<double>(3,0),Q_dw_new.at<double>(0,0));//updating
			desired_wrt_world.setRotation(Q_dw);//updating the transform
			tf::Quaternion Q_P_dw_dot = (Q_dw*tf::Quaternion(vcd.getX(),vcd.getY(),vcd.getZ(),0.0))*Q_dw.inverse();//express vc in world frame
			tf::Vector3 P_dw_dot = tf::Vector3(Q_P_dw_dot.getX(),Q_P_dw_dot.getY(),Q_P_dw_dot.getZ());
			tf::Vector3 P_dw = desired_wrt_world.getOrigin();//origin of camera wrt world
			tf::Vector3 P_dw_new = P_dw + P_dw_dot*time_diff;//update origin
			desired_wrt_world.setOrigin(P_dw_new);//updating the transform

			/********** update pixels wrt desired  **********/
			tf::Vector3 temp_v;
			tf::Quaternion temp_Q;
			temp_v = P_red_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mrd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mgd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mcd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mpd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			prd = K*((1/mrd_bar.getZ())*mrd_bar); pgd = K*((1/mgd_bar.getZ())*mgd_bar); pcd = K*((1/mcd_bar.getZ())*mcd_bar); ppd = K*((1/mpd_bar.getZ())*mpd_bar);// camera points as pixels
			alpha_red_d = mr_bar_ref.getZ()/mrd_bar.getZ();
			
			/********* desired wrt reference *********/
			br.sendTransform(tf::StampedTransform(desired_wrt_world, current_time
							,"world","desired_image"));
										 
			tf::StampedTransform desired_wrt_reference;
			listener.waitForTransform("reference_image", "desired_image"
									 ,current_time, ros::Duration(1.0));
			listener.lookupTransform("reference_image", "desired_image"
									,current_time, desired_wrt_reference);
			Q_df = desired_wrt_reference.getRotation();
			Q_df_negated = tf::Quaternion(-Q_df.getX(),-Q_df.getY(),-Q_df.getZ(),-Q_df.getW()); // getting the negated version of the quaternion for the check
			// checking if the quaternion has flipped
			double Q_norm_desired_diff = std::sqrt(std::pow(Q_df.getX() - Q_df_last.getX(),2.0)
										  + std::pow(Q_df.getY() - Q_df_last.getY(),2.0) 
										  + std::pow(Q_df.getZ() - Q_df_last.getZ(),2.0) 
										  + std::pow(Q_df.getW() - Q_df_last.getW(),2.0));
			double Q_norm_desired_neg_diff = std::sqrt(std::pow(Q_df_negated.getX() - Q_df_last.getX(),2.0)
										  + std::pow(Q_df_negated.getY() - Q_df_last.getY(),2.0) 
										  + std::pow(Q_df_negated.getZ() - Q_df_last.getZ(),2.0) 
										  + std::pow(Q_df_negated.getW() - Q_df_last.getW(),2.0));
			if (Q_norm_desired_diff > Q_norm_desired_neg_diff)
			{
				Q_df = Q_df_negated;
			}
			
			Q_df_last = Q_df;// updating the last
			desired_wrt_reference.setRotation(Q_df);
		}
		
		// generates the velocity command from tracking
		void generate_velocity_command_from_tracking()
		{
			get_wc();// getting the angular velocity command
			get_vc();// getting the linear velocity command
			update_zr_star_hat();// updating the depth estimate
		}
		
		/********** Calculate the value for the angular velocity command wc, angular velocity of F wrt Fstar**********/
		void get_wc()
		{
			tf::Quaternion Qerror_temp = Q_df.inverse()*Q_cf;// rotational error
			Q_error = Qerror_temp;// rotational error
			tf::Vector3 Qerror_ijk_temp(Qerror_temp.getX(),Qerror_temp.getY(),Qerror_temp.getZ());// temp to hold the rotational error ijk terms
			tf::Quaternion Qwcd_temp(wcd.getX(),wcd.getY(),wcd.getZ(),0.0);// getting the desired angular velocity as a quaternion to use for the camera angular velocity command
			tf::Vector3 wc_term1_temp;
			wc_term1_temp = -1*(Kw*Qerror_ijk_temp); // getting the first term of the wc calc
			tf::Quaternion Qwc_term2_temp = (Qerror_temp.inverse()*Qwcd_temp)*Qerror_temp;// calculating the double product for the reporjection of wcd as a quaternion
			tf::Vector3 wc_term2_temp(Qwc_term2_temp.getX(), Qwc_term2_temp.getY(), Qwc_term2_temp.getZ());// getting the second term as a vector		  
			tf::Vector3 wc_temp = wc_term1_temp + wc_term2_temp;
			wc = wc_temp;// update the velocity
		}
		
		/********** Calculate the value for the linear velocity command vc, linear velocity of F wrt Fstar**********/
		void get_vc()
		{
			update_pixels();// update the pixels and everything directly depending on them
			get_Lv();// getting Lvs
			get_Lvd();// getting Lvd
			get_ev();// getting ev
			get_phi();// getting little phi
			tf::Vector3 vc_term1_temp = (1/alpha_red)*(Lv.inverse()*(Kv*ev));// term 1 for the vc calculation
			tf::Vector3 vc_term2_temp = (1/alpha_red)*(Lv.inverse()*(phi*zr_star_hat));// term 2 for the vc calculation
			tf::Vector3 vc_temp = vc_term1_temp + vc_term2_temp;// sum them together for vc
			vc = vc_temp;
			
		}
		/********** update the pixels and everything with them **********/
		void update_pixels()
		{
			tf::Vector3 mr_temp = K.inverse()*pr; 
			mr = mr_temp;//normalized pixel coordinates
			tf::Matrix3x3 ss_mr_temp(           0, -1*mr.getZ(),    mr.getY(),
										    mr.getZ(),            0, -1*mr.getX(),
										 -1*mr.getY(),    mr.getX(),           0);
			ss_mr = ss_mr_temp;// skew symmetrix mr
			tf::Vector3 mrd_temp = K.inverse()*prd; 
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
			tf::Vector3 ped_dot_term1_temp = (-1*alpha_red_d/mr_bar_ref.getZ())*(Lvd*vcd);// getting term 1 for the ped_dot equation
			ped_dot_term1 = ped_dot_term1_temp;
			tf::Vector3 ped_dot_term2_temp = (Lvd*ss_mrd)*wcd;// getting term 2 for the ped_dot equation
			ped_dot_term2 = ped_dot_term2_temp;
			tf::Vector3 ped_dot_temp = ped_dot_term1_temp + ped_dot_term2_temp;
			ped_dot = ped_dot_temp;// ped_dot
		}
	
		/********** update the estimate for the reference zr_star_hat **********/
		void update_zr_star_hat()
		{
			double time_from_last = current_time.toSec() - last_time.toSec();
			calculate_zr_star_hat_dot();// calculate zr_star_hat_dot
			std::cout << "zr star hat dot: " << zr_star_hat_dot << std::endl;
			std::cout << "time from last: " << time_from_last << std::endl;
			std::cout << "zr star hat before: " << zr_star_hat << std::endl;
			zr_star_hat += zr_star_hat_dot*time_from_last;
			std::cout << "zr star hat after: " << zr_star_hat << std::endl;
		}
		
		/********** update the estimate for zr_star_hat_dot **********/
		void calculate_zr_star_hat_dot()
		{
			double time_from_begin = current_time.toSec() - start_time.toSec();
			shift_sample_set(trapz_timestamps, time_from_begin);// update the time for trapz
			get_Ev();// get the new Ev
			get_Phi();// get the new Phi
			get_U();// get the new U
			Ev_minus_Phi = Ev - Phi;// Ev - Phi
			Ev_minus_Phi_svd = Ev_minus_Phi.length2();// Ev - Phi, sum of square of each element
			curr_Ev_minus_Phi_svd_min_iter = std::min_element(Evs_minus_Phis_svds.begin(),Evs_minus_Phis_svds.end());// finding the minimum value
			curr_Ev_minus_Phi_svd_min_index = std::distance(Evs_minus_Phis_svds.begin(),curr_Ev_minus_Phi_svd_min_iter);// getting the index of the iterator

			if (time_from_begin > integration_window)
			{
				
				// if the current norm is greater than the minimum norm of the history will remove the old min and add the new one to the end
				if ( Ev_minus_Phi_svd > *curr_Ev_minus_Phi_svd_min_iter )
				{
					// replace the previous min from the history
					Evs_minus_Phis.at(curr_Ev_minus_Phi_svd_min_index) = Ev_minus_Phi;
					Evs_minus_Phis_svds.at(curr_Ev_minus_Phi_svd_min_index) = Ev_minus_Phi_svd;
					Us.at(curr_Ev_minus_Phi_svd_min_index) = U;
				}
				// getting the summation
				zr_star_hat_dot_sum = 0;
				tf::Vector3 zr_star_hat_dot_term2_temp = tf::Vector3(0,0,0);
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
			
			std::cout << "zr star hat dot sum: " << zr_star_hat_dot_sum << std::endl;
			std::cout << "ev.dot(phi): " << ev.dot(phi) << std::endl;
			zr_star_hat_dot = gamma_1*ev.dot(phi) + gamma_1*gamma_2*zr_star_hat_dot_sum;
			
		}
		
		/********** calculating the value for Ev **********/
		void get_Ev()
		{
			shift_sample_set(Ev_samples, ev);// shifting the buffer
			// getting Ev, it is the difference between the last and first, if only 1 element, just use the first
			if (Ev_samples.size() > 1)
			{
				Ev = Ev_samples.back() - Ev_samples.front();
			}
		}
		
		/********** calculating the value for big Phi **********/
		void get_Phi()
		{
			shift_sample_set(Phi_samples, phi);// phi has already been calculated earlier so just need to put onto the deque then integrate
			calculate_integral(Phi, Phi_samples, trapz_timestamps);// getting the integral to calculate Phi
		}
		
		/********** calculating the value for Uvar **********/
		void get_U()
		{
			Uvar = alpha_red*(Lv*vc);// using a new variable uvar to represent the multiplication used to get the integral for U
			shift_sample_set(Uvar_samples,Uvar);// putting the Uvars onto the Uvar deque
			calculate_integral(U, Uvar_samples, trapz_timestamps);// getting U, the integral of Uvar_samples
		}
		
		/********** function to calculate the integral of a Vector3 using trapizoidal rule **********/
		void calculate_integral(tf::Vector3 &integral, std::deque<tf::Vector3> &function_values, std::deque<double> &time_values)
		{
			tf::Vector3 temp_integral = tf::Vector3(0,0,0);
			if (function_values.size() > 1)
			{
				for (int ii = 0; ii < function_values.size() - 1; ii++)
				{
					temp_integral += 0.5*((time_values.at(ii+1) - time_values.at(ii)) * (function_values.at(ii+1) + function_values.at(ii)));
				}
			}
			integral = temp_integral;
		}
		
		/********** velocity command **********/
		void output_velocity_command()
		{
			if (alpha_red > 0)
			{
				generate_velocity_command_from_tracking();
						
				if (write_to_file)
				{
					output_file.open(output_file_name, std::fstream::out | std::fstream::app);
				}
				if (output_file.is_open())
				{
					output_file << pr.getX() << "," << pr.getY() << "," << pr.getZ() << ","
								<< prd.getX() << "," << prd.getY() << "," << prd.getZ() << ","
								<< mr.getX() << "," << mr.getY() << "," << mr.getZ() << ","
								<< mrd.getX() << "," << mrd.getY() << "," << mrd.getZ() << ","
								<< pe.getX() << "," << pe.getY() << "," << pe.getZ() << ","
								<< ped.getX() << "," << ped.getY() << "," << ped.getZ() << ","
								<< Q_cf.getW() << "," << Q_cf.getX() << "," << Q_cf.getY() << "," << Q_cf.getZ() << ","
								<< Q_df.getW() << "," << Q_df.getX() << "," << Q_df.getY() << "," << Q_df.getZ() << ","
								<< Q_error.getW() << "," << Q_error.getX() << "," << Q_error.getY() << "," << Q_error.getZ() << ","
								<< ev.getX() << "," << ev.getY() << "," << ev.getZ() << ","
								<< phi.getX() << "," << phi.getY() << "," << phi.getZ() << ","
								<< vc.getX() << "," << vc.getY() << "," << vc.getZ() << ","
								<< wc.getX() << "," << wc.getY() << "," << wc.getZ() << ","
								<< vcd.getX() << "," << vcd.getY() << "," << vcd.getZ() << ","
								<< wcd.getX() << "," << wcd.getY() << "," << wcd.getZ() << ","
								<< Ev.getX() << "," << Ev.getY() << "," << Ev.getZ() << ","
								<< Phi.getX() << "," << Phi.getY() << "," << Phi.getZ() << ","
								<< Uvar.getX() << "," << Uvar.getY() << "," << Uvar.getZ() << ","
								<< Ev_minus_Phi.getX() << "," << Ev_minus_Phi.getY() << "," << Ev_minus_Phi.getZ() << ","
								<< Ev_minus_Phi_svd << ","
								<< U.getX() << "," << U.getY() << "," << U.getZ() << ","
								<< zr_star_hat_dot << ","
								<< zr_star_hat
								<< "\n";
					output_file.close();
				}
			}
		}

};

// main
int main(int argc, char** argv)
{   
	// initialize node
	ros::init(argc,argv,"simulator_node");
	
	// initialize image converter    
	Simulator simulator;

	simulator.start_time = ros::Time::now();
	ros::Rate loop_rate(simulator.loop_rate_hz);
	while (ros::ok())
	{
		/********* Update the transforms wrt world and current time **********/
		simulator.current_time = ros::Time::now();
		simulator.br.sendTransform(tf::StampedTransform(simulator.reference_wrt_world, simulator.current_time
								  ,"world", "reference_image"));
		simulator.br.sendTransform(tf::StampedTransform(simulator.camera_wrt_world, simulator.current_time
								  ,"world","current_image"));
		simulator.br.sendTransform(tf::StampedTransform(simulator.desired_wrt_world, simulator.current_time
								  ,"world","desired_image"));
		simulator.br.sendTransform(tf::StampedTransform(simulator.red_wrt_world, simulator.current_time
								  ,"world", "red_feature"));
		simulator.br.sendTransform(tf::StampedTransform(simulator.green_wrt_world, simulator.current_time
								  ,"world","green_feature"));
		simulator.br.sendTransform(tf::StampedTransform(simulator.cyan_wrt_world, simulator.current_time
								  ,"world", "cyan_feature"));
		simulator.br.sendTransform(tf::StampedTransform(simulator.purple_wrt_world, simulator.current_time
								  ,"world","purple_feature"));

		simulator.output_velocity_command();
		simulator.update_camera_pixels();
		simulator.update_desired_pixels();
		
		ros::spinOnce();
		loop_rate.sleep();
		
		simulator.last_time = simulator.current_time;
	}

    return 0;
}
