
#include <iostream>
#include <string>
#include <vector>

// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <homog_track/HomogMarker.h>
#include <homog_track/HomogComplete.h>
#include <homog_track/HomogDecomposed.h>
#include <homog_track/HomogDesired.h>
#include <homog_track/CameraDesiredStart.h>
#include <math.h>


class DesiredCamera
{
	public:	
		// node handle
		ros::NodeHandle nh;
		
		// publisher
		ros::Publisher desired_camera_pub;
		
		// desired start service
		ros::ServiceServer desired_start_service;
		
		// tf broadcaster
		tf::TransformBroadcaster br;
		
		/********** Desired Declarations **********/
		// start desired publishing
		bool start_publishing = false;
		
		// desired radius in meters
		double desired_radius = 1;
		cv::Mat vir_P_d;
		
		// desired period
		double desired_period = 30;
		
		// rotation and translation of virtual desired image wrt reference
		cv::Mat vir_R_df;
		tf::Matrix3x3 vir_R_df_tf;
		tf::Quaternion vir_Q_df_tf;
		tf::Quaternion Q_df_tf_negated;
		tf::Quaternion Q_df_tf_last;
		cv::Mat vir_P_df;
		tf::Vector3 vir_P_df_tf;
		tf::Transform desired_wrt_reference;
		
		// angular rate of change of the circle is then
		//double vir_yaw_dot = -2.0*M_PIl/desired_period;
		double vir_yaw_dot = 0;
		
		// linear velocities
		cv::Mat vir_P_df_dot;
		
		// rate of change matrix
		cv::Mat vir_R_df_dot;
		
		// yaw angle
		double vir_yaw = 0;
		
		// pose message for the desired pose		
		geometry_msgs::Quaternion vir_Q_df_gm;
		geometry_msgs::Point vir_P_df_gm;
		geometry_msgs::Pose pose_df_gm;
		
		// last time the virtual reference was updated
		ros::Time last_virtual_update_time;
		
		// current time
		double current_time;
		
		// time difference
		double time_diff = 0;
		
		/********** Begin Point Declarations **********/
		// positions of the feature points wrt virtual reference image
		// d*           = 1 m
		// red:   (x,y) = (-0.05, -0.05) m
		// green: (x,y) = ( 0.05, -0.05) m
		// cyan:  (x,y) = ( 0.05,  0.05) m
		// blue:  (x,y) = (-0.05,  0.05) m
		
		// if the camera reference is directly centered as defined by these points then 
		// n*           = [0,0,1]^T
		
		// the four points normalized will be
		// m*n_red      = [-0.05, -0.05, 1]^T
		// m*n_green    = [ 0.05, -0.05, 1]^T
		// m*n_cyan     = [ 0.05,  0.05, 1]^T
		// m*n_purple   = [-0.05,  0.05, 1]^T
		
		// converting the normalized coordinates into pixel coordinates using p* = (K)(m*_n)
		// p*_red      = K[-0.05, -0.05, 1]^T
		// p*_green    = K[ 0.05, -0.05, 1]^T
		// p*_cyan     = K[ 0.05,  0.05, 1]^T
		// p*_purple   = K[-0.05,  0.05, 1]^T
		
		// distance along the z to the marker from the reference in meters
		double z_ref = 2;
		
		// real coordinate values for the circles in the reference
		cv::Mat mr_ref_bar;
		cv::Mat mg_ref_bar;
		cv::Mat mc_ref_bar;
		cv::Mat mp_ref_bar;
		
		// normalized
		cv::Mat mr_ref_norm;
		cv::Mat mg_ref_norm;
		cv::Mat mc_ref_norm;
		cv::Mat mp_ref_norm;
		
		// as pixels
		cv::Mat pr_ref;
		cv::Mat pg_ref;
		cv::Mat pc_ref;
		cv::Mat pp_ref;
		
		// real coordinate values for the virtual desired camera
		cv::Mat vir_mrd_bar;
		cv::Mat vir_mgd_bar;
		cv::Mat vir_mcd_bar;
		cv::Mat vir_mpd_bar;
		
		// normalized
		cv::Mat vir_mrd_norm;
		cv::Mat vir_mgd_norm;
		cv::Mat vir_mcd_norm;
		cv::Mat vir_mpd_norm;
		
		// as pixels
		cv::Mat vir_prd;
		cv::Mat vir_pgd;
		cv::Mat vir_pcd;
		cv::Mat vir_ppd;
		
		// vecotrs for the angular desired velocity and the lineard desired velocity
		geometry_msgs::Vector3 omega_cd_gm;
		geometry_msgs::Vector3 vcd_gm;
		
		// the camera matrix
		cv::Mat K;
		
		// geometry message for the pixel location
		geometry_msgs::Point vir_prd_gm;
		geometry_msgs::Point vir_pgd_gm;
		geometry_msgs::Point vir_pcd_gm;
		geometry_msgs::Point vir_ppd_gm;
		
		// message
		homog_track::HomogDesired desired_msg;
		
		DesiredCamera()
		{
			// publisher for the desired reference
			desired_camera_pub = nh.advertise<homog_track::HomogDesired>("desired_homogrphy",1);
			
			// initialize the service
			desired_start_service = nh.advertiseService("start_desired", &DesiredCamera::set_desired_service_handler,this);
			
			//// camera matrix for the ardrone
			//K = cv::Mat::zeros(3,3,CV_64F);
			////first row
			//K.at<double>(0,0) = 567.79;
			//K.at<double>(0,2) = 337.35;
			//// second row
			//K.at<double>(1,1) = 564.52;
			//K.at<double>(1,2) = 169.54;
			//// third row
			//K.at<double>(2,2) = 1;
			K = cv::Mat::eye(3,3,CV_64F);
			
			// real coordinate values for the circles in the reference
			mr_ref_bar = cv::Mat::zeros(3,1,CV_64F);
			mg_ref_bar = cv::Mat::zeros(3,1,CV_64F);
			mc_ref_bar = cv::Mat::zeros(3,1,CV_64F);
			mp_ref_bar = cv::Mat::zeros(3,1,CV_64F);
			
			// red
			mr_ref_bar.at<double>(0,0) =  -0.05;
			mr_ref_bar.at<double>(1,0) = -0.05;
			mr_ref_bar.at<double>(2,0) = z_ref;
			// green
			mg_ref_bar.at<double>(0,0) = 0.05;
			mg_ref_bar.at<double>(1,0) = -0.05;
			mg_ref_bar.at<double>(2,0) = z_ref;
			// cyan
			mc_ref_bar.at<double>(0,0) = 0.05;
			mc_ref_bar.at<double>(1,0) = 0.05;
			mc_ref_bar.at<double>(2,0) = z_ref;
			// purple
			mp_ref_bar.at<double>(0,0) = -0.05;
			mp_ref_bar.at<double>(1,0) = 0.05;
			mp_ref_bar.at<double>(2,0) = z_ref;
			
			// normalized
			mr_ref_norm = (1.0/z_ref)*mr_ref_bar;
			mg_ref_norm = (1.0/z_ref)*mg_ref_bar;
			mc_ref_norm = (1.0/z_ref)*mc_ref_bar;
			mp_ref_norm = (1.0/z_ref)*mp_ref_bar;
			
			// as pixels
			pr_ref = K*mr_ref_norm;
			pg_ref = K*mg_ref_norm;
			pc_ref = K*mc_ref_norm;
			pp_ref = K*mp_ref_norm;
			
			/********** marker values for a virtual desired camera  **********/
			///////////// rotation and translation of virtual desired camera wrt virtual reference camera /////////
			vir_R_df = cv::Mat::zeros(3,3,CV_64F);
			
			// rotation of the virtual desired camera wrt the virtual reference
			vir_R_df.at<double>(0,0) = std::cos(vir_yaw);
			vir_R_df.at<double>(1,0) = std::sin(vir_yaw);
			vir_R_df.at<double>(0,1) = -1*std::sin(vir_yaw);
			vir_R_df.at<double>(1,1) = std::cos(vir_yaw);
			vir_R_df.at<double>(2,2) = 1;
			
			// translation of virtual desired camera wrt the reference camera 
			vir_P_d = cv::Mat::zeros(3,1,CV_64F);
			vir_P_df = cv::Mat::zeros(3,1,CV_64F);
			vir_P_d.at<double>(1,0) = desired_radius;
			vir_P_df = vir_R_df*vir_P_d;
			
			// linear velocity of the desired camera
			vir_P_df_dot = cv::Mat::zeros(3,1,CV_64F);
			vir_R_df_dot = cv::Mat::zeros(3,3,CV_64F);
			vir_R_df_dot.at<double>(0,0) = -1*std::sin(vir_yaw);
			vir_R_df_dot.at<double>(1,0) = std::cos(vir_yaw);
			vir_R_df_dot.at<double>(0,1) = -1*std::cos(vir_yaw);
			vir_R_df_dot.at<double>(1,1) = -1*std::sin(vir_yaw);
			vir_P_df_dot = vir_yaw_dot*(vir_R_df_dot*vir_P_d);
			
			// new position is vir_mid_bar = -1*vir_R_df.transpose()*vir_P_df + vir_R_df.transpose()*mi_ref_bar
			
			// position of the feature points for wrt the virtual camera
			vir_mrd_bar = cv::Mat::zeros(3,1,CV_64F);
			vir_mgd_bar = cv::Mat::zeros(3,1,CV_64F);
			vir_mcd_bar = cv::Mat::zeros(3,1,CV_64F);
			vir_mpd_bar = cv::Mat::zeros(3,1,CV_64F);
			
			// getting the position of the feature points wrt virtual desired camera
			vir_mrd_bar = -1*((vir_R_df.t())*vir_P_df) + (vir_R_df.t())*mr_ref_bar;
			vir_mgd_bar = -1*((vir_R_df.t())*vir_P_df) + (vir_R_df.t())*mg_ref_bar;
			vir_mcd_bar = -1*((vir_R_df.t())*vir_P_df) + (vir_R_df.t())*mc_ref_bar;
			vir_mpd_bar = -1*((vir_R_df.t())*vir_P_df) + (vir_R_df.t())*mp_ref_bar;
			
			std::cout << "reference red:\n" << mr_ref_bar << std::endl;
			std::cout << "reference green:\n" << mg_ref_bar << std::endl;
			std::cout << "reference cyan:\n" << mc_ref_bar << std::endl;
			std::cout << "reference purple:\n" << mp_ref_bar << std::endl;
			
			std::cout << "position of desired camera wrt reference:\n" << vir_P_df << std::endl;
			std::cout << "rotation of desired camera wrt reference:\n" << vir_R_df << std::endl;
			std::cout << "desired camera red bar:\n" << vir_mrd_bar << std::endl;
			std::cout << "desired camera green bar:\n" << vir_mgd_bar << std::endl;
			std::cout << "desired camera cyan bar:\n" << vir_mcd_bar << std::endl;
			std::cout << "desired camera purple bar:\n" << vir_mpd_bar << std::endl;
			
			// normalizing
			vir_mrd_bar = (1.0/vir_mrd_bar.at<double>(2,0))*vir_mrd_bar;
			vir_mgd_bar = (1.0/vir_mgd_bar.at<double>(2,0))*vir_mgd_bar;
			vir_mcd_bar = (1.0/vir_mcd_bar.at<double>(2,0))*vir_mcd_bar;
			vir_mpd_bar = (1.0/vir_mpd_bar.at<double>(2,0))*vir_mpd_bar;
			
			std::cout << "virtual desired red norm:\n" << vir_mrd_bar << std::endl;
			std::cout << "virtual desired green norm:\n" << vir_mgd_bar << std::endl;
			std::cout << "virtual desired cyan norm:\n" << vir_mcd_bar << std::endl;
			std::cout << "virtual desired purple norm:\n" << vir_mpd_bar << std::endl;
			
			// as pixels
			vir_prd = K*vir_mrd_bar;
			vir_pgd = K*vir_mgd_bar;
			vir_pcd = K*vir_mcd_bar;
			vir_ppd = K*vir_mpd_bar;
			
			std::cout << "virtual desired red pixels:\n" << vir_prd << std::endl;
			std::cout << "virtual desired green pixels:\n" << vir_pgd << std::endl;
			std::cout << "virtual desired cyan pixels:\n" << vir_pcd << std::endl;
			std::cout << "virtual desired purple pixels:\n" << vir_ppd << std::endl;
			
		}
		
		// service callback/handler
		bool set_desired_service_handler(homog_track::CameraDesiredStart::Request &req, homog_track::CameraDesiredStart::Response &res)
		{
			// if the boolean is false setting the boolean to true so it will start tracking and setting
			// the start time to now otherwise stopping the desired reference
			if (!start_publishing)
			{
				start_publishing = true;
				last_virtual_update_time = ros::Time::now();
				res.running = true;
			}
			else
			{
				start_publishing = false;
				res.running = false;
			}
			
			return true;
		}
		
		
		// updates the current desired red marker to pixel coordinates
		void update_pixels()
		{
			// checking if the start publishing has been set and if it has will output the current desired
			// based on the transformation update. otherwise will output -1's
			if (start_publishing)
			{
				// update current time and get the time difference
				current_time = ros::Time::now().toSec();
				time_diff = current_time - last_virtual_update_time.toSec();
				std::cout << "time diff:\t" << time_diff << std::endl;

				// updating the yaw
				vir_yaw += vir_yaw_dot*time_diff;
				
				std::cout << "virtual yaw:\t" << vir_yaw << std::endl;
				
				///////////// rotation and translation of virtual desired camera wrt virtual reference camera /////////
				// rotation of the virtual desired camera wrt the virtual reference
				vir_R_df = cv::Mat::zeros(3,3,CV_64F);
				vir_R_df.at<double>(0,0) = std::cos(vir_yaw);
				vir_R_df.at<double>(1,0) = std::sin(vir_yaw);
				vir_R_df.at<double>(0,1) = -1*std::sin(vir_yaw);
				vir_R_df.at<double>(1,1) = std::cos(vir_yaw);
				vir_R_df.at<double>(2,2) = 1;
				
				// translation of virtual desired camera wrt the reference camera 
				vir_P_df = vir_R_df*vir_P_d;
				
				// linear velocity of the desired camera
				vir_P_df_dot = cv::Mat::zeros(3,1,CV_64F);
				vir_R_df_dot = cv::Mat::zeros(3,3,CV_64F);
				vir_R_df_dot.at<double>(0,0) = -1*std::sin(vir_yaw);
				vir_R_df_dot.at<double>(1,0) = std::cos(vir_yaw);
				vir_R_df_dot.at<double>(0,1) = -1*std::cos(vir_yaw);
				vir_R_df_dot.at<double>(1,1) = -1*std::sin(vir_yaw);
				vir_P_df_dot = vir_yaw_dot*(vir_R_df_dot*vir_P_d);
				
				// new position is vir_mid_bar = -1*vir_R_df.transpose()*vir_P_df + vir_R_df.transpose()*mi_ref_bar
				
				// position of the feature points for wrt the virtual camera
				vir_mrd_bar = cv::Mat::zeros(3,1,CV_64F);
				vir_mgd_bar = cv::Mat::zeros(3,1,CV_64F);
				vir_mcd_bar = cv::Mat::zeros(3,1,CV_64F);
				vir_mpd_bar = cv::Mat::zeros(3,1,CV_64F);
				
				// getting the position of the feature points wrt virtual desired camera
				vir_mrd_bar = -1*((vir_R_df.t())*vir_P_df) + (vir_R_df.t())*mr_ref_bar;
				vir_mgd_bar = -1*((vir_R_df.t())*vir_P_df) + (vir_R_df.t())*mg_ref_bar;
				vir_mcd_bar = -1*((vir_R_df.t())*vir_P_df) + (vir_R_df.t())*mc_ref_bar;
				vir_mpd_bar = -1*((vir_R_df.t())*vir_P_df) + (vir_R_df.t())*mp_ref_bar;
				
				std::cout << "reference red:\n" << mr_ref_bar << std::endl;
				std::cout << "reference green:\n" << mg_ref_bar << std::endl;
				std::cout << "reference cyan:\n" << mc_ref_bar << std::endl;
				std::cout << "reference purple:\n" << mp_ref_bar << std::endl;
				
				std::cout << "position of desired camera wrt reference:\n" << vir_P_df << std::endl;
				std::cout << "rotation of desired camera wrt reference:\n" << vir_R_df << std::endl;
				std::cout << "desired camera red bar:\n" << vir_mrd_bar << std::endl;
				std::cout << "desired camera green bar:\n" << vir_mgd_bar << std::endl;
				std::cout << "desired camera cyan bar:\n" << vir_mcd_bar << std::endl;
				std::cout << "desired camera purple bar:\n" << vir_mpd_bar << std::endl;
				
				// normalizing
				vir_mrd_bar = (1.0/vir_mrd_bar.at<double>(2,0))*vir_mrd_bar;
				vir_mgd_bar = (1.0/vir_mgd_bar.at<double>(2,0))*vir_mgd_bar;
				vir_mcd_bar = (1.0/vir_mcd_bar.at<double>(2,0))*vir_mcd_bar;
				vir_mpd_bar = (1.0/vir_mpd_bar.at<double>(2,0))*vir_mpd_bar;
				
				std::cout << "virtual desired red norm:\n" << vir_mrd_bar << std::endl;
				std::cout << "virtual desired green norm:\n" << vir_mgd_bar << std::endl;
				std::cout << "virtual desired cyan norm:\n" << vir_mcd_bar << std::endl;
				std::cout << "virtual desired purple norm:\n" << vir_mpd_bar << std::endl;
				
				// as pixels
				vir_prd = K*vir_mrd_bar;
				vir_pgd = K*vir_mgd_bar;
				vir_pcd = K*vir_mcd_bar;
				vir_ppd = K*vir_mpd_bar;
				
				std::cout << "virtual desired red pixels:\n" << vir_prd << std::endl;
				std::cout << "virtual desired green pixels:\n" << vir_pgd << std::endl;
				std::cout << "virtual desired cyan pixels:\n" << vir_pcd << std::endl;
				std::cout << "virtual desired purple pixels:\n" << vir_ppd << std::endl;
				
				// update the transform of the current desired virtual image wrt the reference image
				vir_R_df_tf[0][0] = vir_R_df.at<double>(0,0);
				vir_R_df_tf[0][1] = vir_R_df.at<double>(0,1);
				vir_R_df_tf[0][2] = vir_R_df.at<double>(0,2);
				vir_R_df_tf[1][0] = vir_R_df.at<double>(1,0);
				vir_R_df_tf[1][1] = vir_R_df.at<double>(1,1);
				vir_R_df_tf[1][2] = vir_R_df.at<double>(1,2);
				vir_R_df_tf[2][0] = vir_R_df.at<double>(2,0);
				vir_R_df_tf[2][1] = vir_R_df.at<double>(2,1);
				vir_R_df_tf[2][2] = vir_R_df.at<double>(2,2);
				vir_R_df_tf.getRotation(vir_Q_df_tf);
				
				// getting the negated version of the quaternion for the check
				Q_df_tf_negated = tf::Quaternion(-vir_Q_df_tf.getX(),-vir_Q_df_tf.getY(),-vir_Q_df_tf.getZ(),-vir_Q_df_tf.getW());
				
				// checking if the quaternion has flipped
				double Q_norm_current_diff = std::sqrt(std::pow(vir_Q_df_tf.getX() - Q_df_tf_last.getX(),2.0)
											  + std::pow(vir_Q_df_tf.getY() - Q_df_tf_last.getY(),2.0) 
											  + std::pow(vir_Q_df_tf.getZ() - Q_df_tf_last.getZ(),2.0) 
											  + std::pow(vir_Q_df_tf.getW() - Q_df_tf_last.getW(),2.0));
				
				double Q_norm_negated_diff = std::sqrt(std::pow(Q_df_tf_negated.getX() - Q_df_tf_last.getX(),2.0)
											  + std::pow(Q_df_tf_negated.getY() - Q_df_tf_last.getY(),2.0) 
											  + std::pow(Q_df_tf_negated.getZ() - Q_df_tf_last.getZ(),2.0) 
											  + std::pow(Q_df_tf_negated.getW() - Q_df_tf_last.getW(),2.0));
				
				//std::cout << "negated difference:\t" << Q_norm_negated_diff << std::endl;
				
				if (Q_norm_current_diff > Q_norm_negated_diff)
				{
					vir_Q_df_tf = Q_df_tf_negated;
				}
				
				// updating the last
				Q_df_tf_last = vir_Q_df_tf;
				
				vir_P_df_tf.setValue(vir_P_df.at<double>(0,0), vir_P_df.at<double>(0,1), vir_P_df.at<double>(0,2));
				desired_wrt_reference.setOrigin(vir_P_df_tf);
				desired_wrt_reference.setRotation(vir_Q_df_tf);
				
				// updating the feature points
				vir_prd_gm.x = vir_prd.at<double>(0,0);
				vir_prd_gm.y = vir_prd.at<double>(1,0);
				vir_prd_gm.z = vir_prd.at<double>(2,0);
				
				vir_pgd_gm.x = vir_pgd.at<double>(0,0);
				vir_pgd_gm.y = vir_pgd.at<double>(1,0);
				vir_pgd_gm.z = vir_pgd.at<double>(2,0);
				
				vir_pcd_gm.x = vir_pcd.at<double>(0,0);
				vir_pcd_gm.y = vir_pcd.at<double>(1,0);
				vir_pcd_gm.z = vir_pcd.at<double>(2,0);
				
				vir_ppd_gm.x = vir_ppd.at<double>(0,0);
				vir_ppd_gm.y = vir_ppd.at<double>(1,0);
				vir_ppd_gm.z = vir_ppd.at<double>(2,0);
				
				// converting the tf quaternion to a geometry message quaternion
				vir_Q_df_gm.x = vir_Q_df_tf.getX();
				vir_Q_df_gm.y = vir_Q_df_tf.getY();
				vir_Q_df_gm.z = vir_Q_df_tf.getZ();
				vir_Q_df_gm.w = vir_Q_df_tf.getW();
				//std::cout << "current output quaternion" << "\n\tx:\t" << vir_Q_df_gm.x 
														 //<< "\n\ty:\t" << vir_Q_df_gm.y
														 //<< "\n\tz:\t" << vir_Q_df_gm.z
														 //<< "\n\tw:\t" << vir_Q_df_gm.w
														 //<< std::endl;
				
				// converting the tf vector3 to a point
				vir_P_df_gm.x = vir_P_df_tf.getX();
				vir_P_df_gm.y = vir_P_df_tf.getY();
				vir_P_df_gm.z = vir_P_df_tf.getZ();
				//std::cout << "current output position" << "\n\tx:\t" << vir_P_df_gm.x 
													   //<< "\n\ty:\t" << vir_P_df_gm.y
													   //<< "\n\tz:\t" << vir_P_df_gm.z
													   //<< std::endl;
				
				// setting the desired message
				pose_df_gm.position = vir_P_df_gm;
				pose_df_gm.orientation = vir_Q_df_gm;
				desired_msg.pose = pose_df_gm;
				desired_msg.header.stamp = ros::Time::now();
				desired_msg.header.frame_id = "desired_frame_normalized";
				desired_msg.height.data = z_ref;
				desired_msg.updating_desired = start_publishing;
				desired_msg.red_circle = vir_prd_gm;
				desired_msg.green_circle = vir_pgd_gm;
				desired_msg.cyan_circle = vir_pcd_gm;
				desired_msg.purple_circle = vir_ppd_gm;
				
				cv::Mat vir_wcd_fd_temp = cv::Mat::zeros(3,1,CV_64F);
				cv::Mat vir_wcd_df_temp = cv::Mat::zeros(3,1,CV_64F);
				vir_wcd_df_temp.at<double>(2,0) = vir_yaw_dot;
				vir_wcd_fd_temp = (vir_R_df.t())*vir_wcd_df_temp;
				omega_cd_gm.x = vir_wcd_fd_temp.at<double>(0,0);
				omega_cd_gm.y = vir_wcd_fd_temp.at<double>(1,0);
				omega_cd_gm.z = vir_wcd_fd_temp.at<double>(2,0);
				desired_msg.omega_cd = omega_cd_gm;
				
				cv::Mat vir_P_fd_dot_temp = cv::Mat::zeros(3,1,CV_64F);
				vir_P_fd_dot_temp = (vir_R_df.t())*vir_P_df_dot;
				vcd_gm.x = vir_P_fd_dot_temp.at<double>(0,0);
				vcd_gm.y = vir_P_fd_dot_temp.at<double>(1,0);
				vcd_gm.z = vir_P_fd_dot_temp.at<double>(2,0);
				desired_msg.v_cd = vcd_gm;
				
				// publish tf
				br.sendTransform(tf::StampedTransform(desired_wrt_reference, last_virtual_update_time
											 ,"reference_image","desired_image"));
			}
			else
			{
				// updating the markers the markers
				vir_prd_gm.x = -1;
				vir_prd_gm.y = -1;
				vir_prd_gm.z = -1;
				
				vir_pgd_gm.x = -1;
				vir_pgd_gm.y = -1;
				vir_pgd_gm.z = -1;
				
				vir_pcd_gm.x = -1;
				vir_pcd_gm.y = -1;
				vir_pcd_gm.z = -1;
				
				vir_ppd_gm.x = -1;
				vir_ppd_gm.y = -1;
				vir_ppd_gm.z = -1;
				desired_msg.updating_desired = start_publishing;
			}
			
			last_virtual_update_time = ros::Time::now();
			
			// publish the marker
			desired_camera_pub.publish(desired_msg);
			
		}
};

// main
int main(int argc, char** argv)
{   
	// initialize node
	ros::init(argc,argv,"desired_camera_node");
	
	// initialize the controller    
	DesiredCamera desired_camera;

	ros::Rate loop_rate(300);
    
    while(ros::ok())
    {
		// updating the desired camera pixels
		desired_camera.update_pixels();
		
		// release 
		ros::spinOnce();
		loop_rate.sleep();
	}

    
    return 0;
}
