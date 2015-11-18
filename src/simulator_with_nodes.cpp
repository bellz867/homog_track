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

// function to push values onto a deque and if it is over some length will remove the first and then push it
template <typename sample_set_type> void shift_sample_set(std::deque<sample_set_type> &sample_set, sample_set_type &new_sample)
{
	sample_set.pop_front();
	sample_set.push_back(new_sample);
}

/********** Processes incoming images and outputs the pixels **********/
class ImageProcessing
{
	public:
		/********** node and topics **********/
		ros::NodeHandle nh;// handle for the image processing
		ros::Publisher pixel_pub;// pixel publisher
		ros::Subscriber cam_vel_sub;// camera velocity subscriber
		tf::TransformBroadcaster br;
		tf::TransformListener listener;
		
		/********** Time Values **********/
		ros::Time last_time;// last time the camera was updated
		ros::Time current_time;// current time the camera was updated
		ros::Time start_time;// start time
		double loop_rate_hz;
		bool camera_updated = false;
		
		/********** Points **********/
		tf::Vector3 P_red_wrt_world, P_green_wrt_world, P_cyan_wrt_world, P_purple_wrt_world;// homography points wrt world
		tf::Transform red_wrt_world, green_wrt_world, cyan_wrt_world, purple_wrt_world;//transforms for the marker points
		
		/********** Reference **********/
		tf::Vector3 mr_bar_ref, mg_bar_ref, mc_bar_ref, mp_bar_ref;// points wrt reference
		tf::Vector3 pr_ref, pg_ref, pc_ref, pp_ref;// pixels wrt reference
		tf::Transform reference_wrt_world; // transform for reference camera
		
		/********** Camera **********/
		tf::Vector3 mr_bar, mg_bar, mc_bar, mp_bar;// points wrt camera
		tf::Vector3 mr, mg, mc, mp;// normalized points wrt camera
		tf::Vector3 pr, pg, pc, pp;// points pixels wrt camera
		geometry_msgs::Point pr_gm, pg_gm, pc_gm, pp_gm;// points pixels wrt camera for message
		homog_track::ImageProcessingMsg pixels_out;
		tf::Transform camera_wrt_world; // transform for camera
		
		/********** Camera wrt reference **********/
		tf::Transform camera_wrt_reference;// transform of camera wrt reference
		tf::Quaternion Q_cf = tf::Quaternion(0,0,0,0), Q_cf_last = tf::Quaternion(0,0,0,0), Q_cf_negated = tf::Quaternion(0,0,0,0);// camera wrt reference, last camera wrt reference, and negated camera wrt reference
		double Q_norm_current_diff, Q_norm_negated_diff;// norms to determine which rotation is closer to last
		
		/********* Cam info **********/
		tf::Matrix3x3 A = tf::Matrix3x3(1,0,0,
										0,1,0,
										0,0,1);// camera matrix
										
		/********** Velocity Commands **********/
		tf::Vector3 vc, wc;//linear and angular velocity commands
		cv::Mat wc_cv;
		
		ImageProcessing(double loop_rate_des)
		{
			loop_rate_hz = loop_rate_des;
			/********** topics **********/
			pixel_pub = nh.advertise<homog_track::ImageProcessingMsg>("feature_pixels", 1);
			cam_vel_sub = nh.subscribe("/cmd_vel", 1, &ImageProcessing::update_camera_pixels, this);
			
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
			pr_ref = A*((1/mr_bar_ref.getZ())*mr_bar_ref); pg_ref = A*((1/mg_bar_ref.getZ())*mg_bar_ref); pc_ref = A*((1/mc_bar_ref.getZ())*mc_bar_ref); pp_ref = A*((1/mp_bar_ref.getZ())*mp_bar_ref);// reference points as pixels
			
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
			pr = A*((1/mr_bar.getZ())*mr_bar); pg = A*((1/mg_bar.getZ())*mg_bar); pc = A*((1/mc_bar.getZ())*mc_bar); pp = A*((1/mp_bar.getZ())*mp_bar);// camera points as pixels
			pr_gm.x = pr.getX(); pr_gm.y = pr.getY(); pr_gm.z = pr.getZ();// red
			pg_gm.x = pg.getX(); pg_gm.y = pg.getY(); pg_gm.z = pg.getZ();// grenn
			pc_gm.x = pc.getX(); pc_gm.y = pc.getY(); pc_gm.z = pc.getZ();// cyan
			pp_gm.x = pp.getX(); pp_gm.y = pp.getY(); pp_gm.z = pp.getZ();// purple
			
			//std::cout << "image processing camera before" << std::endl;
			//std::cout << pixels_out << std::endl;
			
			pixels_out.pr = pr_gm; pixels_out.pg = pg_gm; pixels_out.pc = pc_gm; pixels_out.pp = pp_gm;// out message pixels
			
			//std::cout << "image processing camera after" << std::endl;
			//std::cout << pixels_out << std::endl;
			
			last_time = ros::Time::now();
			pixels_out.header.stamp = last_time;// out message current time
			start_time = last_time;;
			camera_updated = true;
			br.sendTransform(tf::StampedTransform(camera_wrt_world, last_time
								  ,"world","current_image"));
		}
		
		/********** velocity callback **********/
		void update_camera_pixels(const geometry_msgs::Twist& msg)
		{
			current_time = ros::Time::now();//update time
			vc.setX(msg.linear.x); vc.setY(msg.linear.y); vc.setZ(msg.linear.z);//linear velocity
			wc.setZ(msg.angular.z);// angular velocity
			
			//std::cout << "vc:\n x: " << vc.getX() << " y: " << vc.getY() << " z: " << vc.getZ() << std::endl;
			//std::cout << "wc:\n x: " << wc.getX() << " y: " << wc.getY() << " z: " << wc.getZ() << std::endl;
			
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
			pr = A*((1/mr_bar.getZ())*mr_bar); pg = A*((1/mg_bar.getZ())*mg_bar); pc = A*((1/mc_bar.getZ())*mc_bar); pp = A*((1/mp_bar.getZ())*mp_bar);// camera points as pixels
			pr_gm.x = pr.getX(); pr_gm.y = pr.getY(); pr_gm.z = pr.getZ();// red
			pg_gm.x = pg.getX(); pg_gm.y = pg.getY(); pg_gm.z = pg.getZ();// grenn
			pc_gm.x = pc.getX(); pc_gm.y = pc.getY(); pc_gm.z = pc.getZ();// cyan
			pp_gm.x = pp.getX(); pp_gm.y = pp.getY(); pp_gm.z = pp.getZ();// purple
			
			//std::cout << "image processing camera before" << std::endl;
			//std::cout << pixels_out << std::endl;
			
			pixels_out.pr = pr_gm; pixels_out.pg = pg_gm; pixels_out.pc = pc_gm; pixels_out.pp = pp_gm;// out message pixels
			
			//std::cout << "image processing camera after" << std::endl;
			//std::cout << pixels_out << std::endl;
			
			last_time = current_time;
			pixels_out.header.stamp = current_time;// out message current time
			camera_updated = true;
			br.sendTransform(tf::StampedTransform(camera_wrt_world, current_time
								  ,"world","current_image"));
			std::cout << "pixels updated" << std::endl;
		}
};

/********** Homography Decomposition **********/
class HomogDecomp
{
	public:
		/********** node and topics **********/
		ros::NodeHandle nh;// handle for the image processing
		ros::Subscriber pixel_sub;// pixel subscriber
		ros::Publisher homog_decomp_pub;// publisher for the decomposed homography
		
		/********** Points **********/
		tf::Vector3 P_red_wrt_world, P_green_wrt_world, P_cyan_wrt_world, P_purple_wrt_world;// homography points wrt world
		tf::Transform red_wrt_world, green_wrt_world, cyan_wrt_world, purple_wrt_world;//transforms for the marker points
		
		/********** Reference Declarations **********/
		tf::Vector3 mr_bar_ref, mg_bar_ref, mc_bar_ref, mp_bar_ref;// points wrt reference
		tf::Vector3 pr_ref, pg_ref, pc_ref, pp_ref;// pixels wrt reference
		geometry_msgs::Point pr_ref_gm, pg_ref_gm, pc_ref_gm, pp_ref_gm;// reference pixels wrt camera for message
		tf::Transform reference_wrt_world; // transform for reference camera
		std::vector<cv::Point2d> ref_pixels;// reference points for the homography
		cv::Mat pr_ref_m, pg_ref_m, pc_ref_m, pp_ref_m; // reference points as pixels
		cv::Mat mr_ref_norm, mg_ref_norm, mc_ref_norm, mp_ref_norm; // reference points normalized
		
		/********** Camera Declarations **********/
		tf::Vector3 mr_bar, mg_bar, mc_bar, mp_bar;// points wrt camera
		tf::Vector3 mr, mg, mc, mp;// normalized points wrt camera
		tf::Vector3 pr, pg, pc, pp;// points pixels wrt camera
		geometry_msgs::Point pr_gm, pg_gm, pc_gm, pp_gm;// points pixels wrt camera for message
		tf::Transform camera_wrt_world; // transform for camera
		std::vector<cv::Point2d> pixels;// current points for the homography
		cv::Mat pr_m, pg_m, pc_m, pp_m;// current points as pixels
		cv::Mat mr_norm, mg_norm, mc_norm, mp_norm;// current points normalized
		
		/********** Decomp Declarations **********/
		std::vector<cv::Mat> curr_points_m, ref_points_m;// vector for the matrix of current points
		tf::Matrix3x3 A_tf = tf::Matrix3x3(1,0,0,
										0,1,0,
										0,0,1);// camera matrix
		cv::Mat A;// the camera matrix as a cv
		tf::Transform camera_wrt_reference;// transform of camera wrt reference
		tf::Quaternion Q_cf = tf::Quaternion(0,0,0,0), Q_cf_last = tf::Quaternion(0,0,0,0), Q_cf_negated = tf::Quaternion(0,0,0,0);// camera wrt reference, last camera wrt reference, and negated camera wrt reference
		double Q_norm_current_diff, Q_norm_negated_diff;// norms to determine which rotation is closer to last
		double alpha_red, alpha_green, alpha_cyan, alpha_purple;// alphas for the camera
		cv::Mat G;// the perspective homography matrix
		cv::Mat H_hat;// estimated homography matrix
		cv::Mat H;// scaled homography matrix
		cv::SVD svd;// svd of the perspective homography matrix
		double svd_1, svd_2, svd_3; std::vector<double> svds; // three values for the svd 
		double gamma_h;// gamma term the estimated matrix is scaled by
		int successful_decomp;// successful decomposition
		std::vector<cv::Mat> R, T, n;// the rotation, translation, and normal output
		cv::Mat temp_scalar;// temp holder for scalar checking if a converted point is positive
		std::vector<double> scalar_value_check;// array to hold the values of the all the positive definite check values
		std::vector<double>::iterator temp_solution_start, temp_solution_end; std::vector<double> temp_solution;// temporary solution
		bool all_positive;//indicates all temp values are positive
		int current_temp_index;
		std::vector<double>::iterator first_solution_start, first_solution_end; std::vector<double> first_solution;// first solution variables
		bool first_solution_found;//indicates first solution found
		cv::Mat first_R, first_T, first_n;// first solution rotation, translation, and normal
		std::vector<double>::iterator second_solution_start, second_solution_end; std::vector<double> second_solution;// second solution variables. there may not be a second solution
		bool second_solution_found;// indicates second solution found
		cv::Mat second_R, second_T, second_n;// second solution rotation, translation, and normal
		bool fc_found;
		cv::Mat R_fc, T_fc, n_fc, n_ref;// rotation, translation, and normal of reference wrt camera
		tf::Matrix3x3 R_fc_tf, R_cf_tf;// rotation of reference wrt camera and camera wrt reference
		tf::Quaternion Q_cf_tf;// rotation of camera wrt reference as quaternion
		tf::Vector3 T_fc_tf, T_cf_tf;// position of reference wrt camera and camera wrt reference
		homog_track::DecompMsg decomposed_msg;// complete decomposed message
		homog_track::ImageProcessingMsg cam_pixels;// camera pixels for the out message
		homog_track::ImageProcessingMsg ref_cam_pixels;// reference pixels for the out message
		
		// constructor for the complete set of markers
		HomogDecomp()
		{
			pixel_sub = nh.subscribe("feature_pixels",1, &HomogDecomp::pixels_callback, this);// subscribing to the complete message
			homog_decomp_pub = nh.advertise<homog_track::DecompMsg>("decomposed_homography",1);// publisher for the decomposed stuff
			
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
			pr_ref = A_tf*((1/mr_bar_ref.getZ())*mr_bar_ref); pg_ref = A_tf*((1/mg_bar_ref.getZ())*mg_bar_ref); pc_ref = A_tf*((1/mc_bar_ref.getZ())*mc_bar_ref); pp_ref = A_tf*((1/mp_bar_ref.getZ())*mp_bar_ref);// reference points as pixels
			pr_ref_gm.x = pr_ref.getX(); pr_ref_gm.y = pr_ref.getY(); pr_ref_gm.z = pr_ref.getZ();// red
			pg_ref_gm.x = pg_ref.getX(); pg_ref_gm.y = pg_ref.getY(); pg_ref_gm.z = pg_ref.getZ();// grenn
			pc_ref_gm.x = pc_ref.getX(); pc_ref_gm.y = pc_ref.getY(); pc_ref_gm.z = pc_ref.getZ();// cyan
			pp_ref_gm.x = pp_ref.getX(); pp_ref_gm.y = pp_ref.getY(); pp_ref_gm.z = pp_ref.getZ();// purple
			ref_cam_pixels.pr = pr_ref_gm; ref_cam_pixels.pg = pg_ref_gm; ref_cam_pixels.pc = pc_ref_gm; ref_cam_pixels.pp = pp_ref_gm;//reference pixels for message
			ref_pixels.push_back(cv::Point2d(pr_ref.getX(),pr_ref.getY()));//red for find homography
			ref_pixels.push_back(cv::Point2d(pg_ref.getX(),pg_ref.getY()));//green for find homography
			ref_pixels.push_back(cv::Point2d(pc_ref.getX(),pc_ref.getY()));//cyan for find homography
			ref_pixels.push_back(cv::Point2d(pp_ref.getX(),pp_ref.getY()));//purple for find homography
			
			
			/********** decomp parameters **********/
			A = cv::Mat::eye(3,3,CV_64F);// camera matrix
			n_ref = cv::Mat::zeros(3,1,CV_64F); n_ref.at<double>(2,0) = 1; // normal for reference
			successful_decomp = 0;// initializing decomp to false
			temp_scalar = cv::Mat::zeros(1,1,CV_64F);// initializer temp scalar to zero
			
			pr_m = cv::Mat::ones(3,1,CV_64F);
			pg_m = cv::Mat::ones(3,1,CV_64F);
			pc_m = cv::Mat::ones(3,1,CV_64F);
			pp_m = cv::Mat::ones(3,1,CV_64F);
			
			mr_norm = cv::Mat::ones(3,1,CV_64F);
			mg_norm = cv::Mat::ones(3,1,CV_64F);
			mc_norm = cv::Mat::ones(3,1,CV_64F);
			mp_norm = cv::Mat::ones(3,1,CV_64F);
			
			mr_ref_norm = cv::Mat::ones(3,1,CV_64F); mr_ref_norm.at<double>(0,0) = mr_bar_ref.getX()/mr_bar_ref.getZ(); mr_ref_norm.at<double>(1,0) = mr_bar_ref.getY()/mr_bar_ref.getZ();
			mg_ref_norm = cv::Mat::ones(3,1,CV_64F); mg_ref_norm.at<double>(0,0) = mg_bar_ref.getX()/mg_bar_ref.getZ(); mg_ref_norm.at<double>(1,0) = mg_bar_ref.getY()/mg_bar_ref.getZ();
			mc_ref_norm = cv::Mat::ones(3,1,CV_64F); mc_ref_norm.at<double>(0,0) = mc_bar_ref.getX()/mc_bar_ref.getZ(); mc_ref_norm.at<double>(1,0) = mc_bar_ref.getY()/mc_bar_ref.getZ();
			mp_ref_norm = cv::Mat::ones(3,1,CV_64F); mp_ref_norm.at<double>(0,0) = mp_bar_ref.getX()/mp_bar_ref.getZ(); mp_ref_norm.at<double>(1,0) = mp_bar_ref.getY()/mp_bar_ref.getZ();
		
			ref_points_m.push_back(mr_ref_norm);
			ref_points_m.push_back(mg_ref_norm);
			ref_points_m.push_back(mc_ref_norm);
			ref_points_m.push_back(mp_ref_norm);
			
		}
		
		// callback for the complete message
		void pixels_callback(const homog_track::ImageProcessingMsg& msg)
		{
			/********** Begin splitting up the incoming message *********/
			// initializer temp scalar to zero
			temp_scalar = cv::Mat::zeros(1,1,CV_64F);
			
			pixels.push_back(cv::Point2d(msg.pr.x,msg.pr.y));//red
			pixels.push_back(cv::Point2d(msg.pg.x,msg.pg.y));//green
			pixels.push_back(cv::Point2d(msg.pc.x,msg.pc.y));//cyan
			pixels.push_back(cv::Point2d(msg.pp.x,msg.pp.y));//purple
			
			//std::cout << "reference" << std::endl;
			//for (cv::Point2d ii : ref_pixels)
			//{
				//std::cout << ii << std::endl;
			//}
			
			//std::cout << "camera" << std::endl;
			//for (cv::Point2d ii : pixels)
			//{
				//std::cout << ii << std::endl;
			//}
			
			
			pr_m.at<double>(0,0) = msg.pr.x;
			pr_m.at<double>(1,0) = msg.pr.y;
			pg_m.at<double>(0,0) = msg.pg.x;
			pg_m.at<double>(1,0) = msg.pg.y;
			pc_m.at<double>(0,0) = msg.pc.x;
			pc_m.at<double>(1,0) = msg.pc.y;
			pp_m.at<double>(0,0) = msg.pp.x;
			pp_m.at<double>(1,0) = msg.pp.y;
			
			mr_norm = A.inv(cv::DECOMP_LU)*pr_m;
			mg_norm = A.inv(cv::DECOMP_LU)*pg_m;
			mc_norm = A.inv(cv::DECOMP_LU)*pc_m;
			mp_norm = A.inv(cv::DECOMP_LU)*pp_m;
			
			curr_points_m.push_back(mr_norm);
			curr_points_m.push_back(mg_norm);
			curr_points_m.push_back(mc_norm);
			curr_points_m.push_back(mp_norm);
			
			// if any of the points have a -1 will skip over the homography
			if (msg.pr.x != -1 && msg.pg.x != -1 && msg.pc.x != -1 && msg.pp.x != -1)
			{	
				/********** following the process outlined in the reference **********/			
				G = cv::findHomography(ref_pixels,pixels,0);// finding the perspective homography
				H_hat = (A.inv(cv::DECOMP_LU)*G)*A;// finding the approximate of the euclidean homography
				// getting the svd of the approximate
				svd = cv::SVD(H_hat,cv::SVD::NO_UV);
				svd_1 = svd.w.at<double>(0,0);
				svd_2 = svd.w.at<double>(1,0);
				svd_3 = svd.w.at<double>(2,0);
				svds.push_back(svd_1);
				svds.push_back(svd_2);
				svds.push_back(svd_3);
				std::sort(svds.begin(),svds.end());
				gamma_h = *(svds.begin()+svds.size()/2);
				svds.erase(svds.begin(),svds.end());
				H = (1.0/gamma_h)*H_hat;
				successful_decomp = cv::decomposeHomographyMat(G,A,R,T,n);// decompose homography into 4 solutions
				// if the decomp is successful will find the solutions
				if (successful_decomp > 0)
				{
					// finding the alphas
					alpha_red = mr_norm.at<double>(2,0)/((H.row(2)).dot(mr_ref_norm.t()));
					alpha_green = mg_norm.at<double>(2,0)/((H.row(2)).dot(mg_ref_norm.t()));
					alpha_cyan = mc_norm.at<double>(2,0)/((H.row(2)).dot(mc_ref_norm.t()));
					alpha_purple = mp_norm.at<double>(2,0)/((H.row(2)).dot(mp_ref_norm.t()));
					
					// finding the solutions that give the positive results
					for (int ii = 0; ii < successful_decomp; ii++)
					{
						// performing the operation transpose(m)*R*n to check if greater than 0 later
						// order operating on is red green cyan purple
						for (int jj = 0; jj < 4; jj++)
						{
							temp_scalar = curr_points_m[jj].t();
							temp_scalar = temp_scalar*R[ii];
							temp_scalar = temp_scalar*n[ii];
							scalar_value_check.push_back(temp_scalar.at<double>(0,0));
						}
					}
					
					// restting first solution found and second solution found
					first_solution_found = false;
					second_solution_found = false;
					fc_found = false;
					
					// getting the two solutions or only one if there are not two
					for (int ii = 0; ii < successful_decomp; ii++)
					{
						// getting the values onto the temporary vector
						// getting the start and end of the next solution
						temp_solution_start = scalar_value_check.begin() + 4*ii;
						temp_solution_end = scalar_value_check.begin() + 4*ii+4;
						temp_solution.assign(temp_solution_start,temp_solution_end);
						
						// checking if all the values are positive
						all_positive = true;
						current_temp_index = 0;
						while (all_positive && current_temp_index < 4)
						{
							if (temp_solution[current_temp_index] >= 0)
							{
								current_temp_index++;
							}
							else
							{
								all_positive = false;
							}
						}
						// if all the values were positive and a first solution has not been found will assign 
						// to first solution. if all positive and first solution has been found will assign
						// to second solution. if all positive is false then will not do anything
						if (all_positive && first_solution_found && !second_solution_found)
						{
							// setting it to indicate a solution has been found
							second_solution_found = true;
							
							// setting the rotation, translation, and normal to be the second set
							second_R = R[ii];
							second_T = T[ii];
							second_n = n[ii];
							
							// setting the projected values
							second_solution = temp_solution;
						}
						else if (all_positive && !first_solution_found)
						{
							// setting it to indicate a solution has been found
							first_solution_found = true;
							
							// setting the rotation, translation, and normal to be the first set
							first_R = R[ii];
							first_T = T[ii];
							first_n = n[ii];
							
							// setting the projected values
							first_solution = temp_solution;
						}
						
						// erasing all the values from the temp solution
						temp_solution.erase(temp_solution.begin(),temp_solution.end());
					}
					
					// erasing all the scalar values from the check
					scalar_value_check.erase(scalar_value_check.begin(),scalar_value_check.end());
					

					/********** setting the message and sending it **********/
					// rotation matricies
					for (int ii = 0; ii < 9; ii++)
					{
						if (first_solution_found)
						{
							decomposed_msg.R1[ii] = first_R.at<double>(ii/3,ii%3);
						}
						else
						{
							decomposed_msg.R1[ii] = -1;
						}
						if (second_solution_found)
						{
							decomposed_msg.R2[ii] = second_R.at<double>(ii/3,ii%3);
						}
						else
						{
							decomposed_msg.R2[ii] = -1;
						}
					}
					// normal and normalized translation
					for (int ii = 0; ii < 3; ii++)
					{
						if (first_solution_found)
						{
							decomposed_msg.t1[ii] = first_T.at<double>(ii,0);
							decomposed_msg.n1[ii] = first_n.at<double>(ii,0);
						}
						else
						{
							decomposed_msg.t1[ii] = -1;
							decomposed_msg.n1[ii] = -1;
						}
						if (first_solution_found)
						{
							decomposed_msg.t2[ii] = second_T.at<double>(ii,0);
							decomposed_msg.n2[ii] = second_n.at<double>(ii,0);
						}
						else
						{
							decomposed_msg.t2[ii] = -1;
							decomposed_msg.n2[ii] = -1;
						}
					}
					cam_pixels.pr = msg.pr; cam_pixels.pg = msg.pg; cam_pixels.pc = msg.pc; cam_pixels.pp = msg.pp;
					
					decomposed_msg.alphar = alpha_red; decomposed_msg.alphag = alpha_green; decomposed_msg.alphac = alpha_cyan; decomposed_msg.alphap = alpha_purple;// alphas
					decomposed_msg.cam_pixels = cam_pixels;// pixels
					decomposed_msg.ref_cam_pixels = ref_cam_pixels;// reference pixels
					decomposed_msg.header.stamp = msg.header.stamp;// time
					decomposed_msg.header.frame_id = "decomp_message";
					homog_decomp_pub.publish(decomposed_msg);// sending the message
				}
			}

			// erasing all the point vectors and matrix vectors
			pixels.erase(pixels.begin(),pixels.end());
			curr_points_m.erase(curr_points_m.begin(),curr_points_m.end());
			
			std::cout << "decomp updated" << std::endl;
		}
};

/********** Controller **********/
class Controller
{
	public:
		bool display_calc_steps = true;
		double loop_rate_hz;
		double integration_window = 0.2;
		bool first_run = true;
		
		/********** Topic Declarations **********/
		ros::NodeHandle nh;
		tf::TransformBroadcaster br;
		tf::TransformListener listener;
		ros::Subscriber decomp_sub;
		ros::Publisher cmd_vel_pub;
		
		
		/********** Time Values **********/
		ros::Time last_time;// last time the camera was updated
		ros::Time current_time;// current time the camera was updated
		ros::Time start_time;// start time
		
		/********** File Declarations **********/
		bool write_to_file;
		std::fstream output_file;
		std::string output_file_name; // file name
		
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
		
		/********** Points **********/
		tf::Vector3 P_red_wrt_world, P_green_wrt_world, P_cyan_wrt_world, P_purple_wrt_world;// homography points wrt world
		tf::Transform red_wrt_world, green_wrt_world, cyan_wrt_world, purple_wrt_world;//transforms for the marker points
		
		/********** Reference **********/
		tf::Vector3 mr_bar_ref, mg_bar_ref, mc_bar_ref, mp_bar_ref;// points wrt reference
		tf::Vector3 pr_ref, pg_ref, pc_ref, pp_ref;// pixels wrt reference
		tf::Transform reference_wrt_world; // transform for reference camera
		
		/********** Camera **********/
		tf::Vector3 mr_bar, mg_bar, mc_bar, mp_bar;// points wrt camera
		tf::Vector3 mr, mg, mc, mp;// normalized points wrt camera
		tf::Vector3 pr, pg, pc, pp;// points pixels wrt camera
		geometry_msgs::Point pr_gm, pg_gm, pc_gm, pp_gm;// points pixels wrt camera for message
		homog_track::ImageProcessingMsg pixels_out;
		tf::Transform camera_wrt_world; // transform for camera
		
		/********** Decomp Declarations **********/
		tf::Transform camera_wrt_reference;// transform of camera wrt reference
		tf::Quaternion Q_cf = tf::Quaternion(0,0,0,0), Q_cf_last = tf::Quaternion(0,0,0,0), Q_cf_negated = tf::Quaternion(0,0,0,0);// camera wrt reference, last camera wrt reference, and negated camera wrt reference
		double Q_norm_current_diff, Q_norm_negated_diff;// norms to determine which rotation is closer to last
		double alpha_red, alpha_green, alpha_cyan, alpha_purple;// alphas for the camera		
		
		/********* Cam info **********/
		tf::Matrix3x3 A = tf::Matrix3x3(1,0,0,
										0,1,0,
										0,0,1);// camera matrix
										
		/********** Velocity Commands **********/
		tf::Vector3 vc, wc;//linear and angular velocity commands
		cv::Mat wc_cv;
		
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
		int number_of_samples = 20;// number of samples for stack
		int number_of_integrating_samples;// number of integrating samples
		tf::Vector3 Ev = tf::Vector3(0,0,0); tf::Vector3 Phi = tf::Vector3(0,0,0); tf::Vector3 U = tf::Vector3(0,0,0); tf::Vector3 Uvar = tf::Vector3(0,0,0);// current value for Ev, Phi, U, and Uvar
		tf::Matrix3x3 principle_point = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);// matrix with the principle point coordinates of the camera
		std::deque<tf::Vector3> Ev_samples, Phi_samples, Uvar_samples;// buffer samples for learning stack terms
		std::deque<double> trapz_timestamps;// times for buffer samples
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

		Controller(double loop_rate_des, bool write_to_file_des, std::string& output_file_name_des)
		{	
			output_file_name = output_file_name_des;
			//std::cout << "output_file_name: " << output_file_name << std::endl;
			write_to_file = write_to_file_des;
			loop_rate_hz = loop_rate_des;
			number_of_integrating_samples = loop_rate_hz*0.2;
			decomp_sub = nh.subscribe("decomposed_homography",1, &Controller::decomp_callback, this);// subscribing to the complete message
			cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);// publisher for the decomposed stuff
			
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
			pr_ref = A*((1/mr_bar_ref.getZ())*mr_bar_ref); pg_ref = A*((1/mg_bar_ref.getZ())*mg_bar_ref); pc_ref = A*((1/mc_bar_ref.getZ())*mc_bar_ref); pp_ref = A*((1/mp_bar_ref.getZ())*mp_bar_ref);// reference points as pixels
			
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
			prd = A*((1/mrd_bar.getZ())*mrd_bar); pgd = A*((1/mgd_bar.getZ())*mgd_bar); pcd = A*((1/mcd_bar.getZ())*mcd_bar); ppd = A*((1/mpd_bar.getZ())*mpd_bar);// camera points as pixels
			alpha_red_d = mr_bar_ref.getZ()/mrd_bar.getZ();
			
			/********* update time and reference*********/
			last_time = ros::Time::now();
			br.sendTransform(tf::StampedTransform(reference_wrt_world, last_time
							,"world", "reference_image"));
			
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
			
			// initializing the constant part of Lv
			for (int ii = 0; ii < 3; ii++)
			{	
				for (int jj = 0; jj<3; jj++)
				{
					Lv_term1[ii][jj] = A[ii][jj] - principle_point[ii][jj];
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
		
		/********** callback for the decomp node **********/
		void decomp_callback(const homog_track::DecompMsg& msg)
		{
			tf::Matrix3x3 R_fc_temp = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);
			current_time = msg.header.stamp;
			if (first_run)
			{
				first_run = false;
				start_time = msg.header.stamp;
				last_time = current_time;
				//pr_ref.setX() = msg.ref_cam_pixels.pr.x; pr_ref.setY() = msg.ref_cam_pixels.pr.y; pr_ref.setZ() = 1;//red pixels
				//pg_ref.setX() = msg.ref_cam_pixels.pg.x; pg_ref.setY() = msg.ref_cam_pixels.pg.y; pg_ref.setZ() = 1;//green pixels
				//pc_ref.setX() = msg.ref_cam_pixels.pc.x; pc_ref.setY() = msg.ref_cam_pixels.pc.y; pc_ref.setZ() = 1;//cyan pixels
				//pp_ref.setX() = msg.ref_cam_pixels.pp.x; pp_ref.setY() = msg.ref_cam_pixels.pp.y; pp_ref.setZ() = 1;//purple pixels
			}
			
			// because the reference is set to the exact value when when n should have only a z componenet, the correct
			// choice should be the one closest to n_ref = [0,0,1]^T which will be the one with the greatest dot product with n_ref
			
			if (msg.n1[2] != -1 && msg.n2[2] != -1)
			{
				if (msg.n1[2] >= msg.n2[2])
				{
					for (int ii = 0; ii < 9; ii++)
					{
						R_fc_temp[ii/3][ii%3] = msg.R1[ii];
					}
				}
				else
				{
					for (int ii = 0; ii < 9; ii++)
					{
						R_fc_temp[ii/3][ii%3] = msg.R2[ii];
					}
				}
			}
			else if(msg.n1[2] != -1 && msg.n2[2] == -1)
			{
				for (int ii = 0; ii < 9; ii++)
				{
					R_fc_temp[ii/3][ii%3] = msg.R1[ii];
				}
			}
			(R_fc_temp.transpose()).getRotation(Q_cf);// take transpose to get camera wrt reference
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
			pr.setX(msg.cam_pixels.pr.x); pr.setY(msg.cam_pixels.pr.y); pr.setZ(1);//red pixels
			//std::cout << "pr:\n x: " << pr.getX() << " y: " << pr.getY() << " z: " << pr.getZ() << std::endl;
			pg.setX(msg.cam_pixels.pg.x); pg.setY(msg.cam_pixels.pg.y); pg.setZ(1);//green pixels
			//std::cout << "pg:\n x: " << pg.getX() << " y: " << pg.getY() << " z: " << pg.getZ() << std::endl;
			pc.setX(msg.cam_pixels.pc.x); pc.setY(msg.cam_pixels.pc.y); pc.setZ(1);//cyan pixels
			//std::cout << "pc:\n x: " << pc.getX() << " y: " << pc.getY() << " z: " << pc.getZ() << std::endl;
			pp.setX(msg.cam_pixels.pp.x); pp.setY(msg.cam_pixels.pp.y); pp.setZ(1);//purple pixels
			//std::cout << "pp:\n x: " << pp.getX() << " y: " << pp.getY() << " z: " << pp.getZ() << std::endl;
			alpha_red = msg.alphar;
			update_desired_pixels();
			output_velocity_command();
			last_time = current_time;
		}
		
		/********** update the desired pixels **********/
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
			prd = A*((1/mrd_bar.getZ())*mrd_bar); pgd = A*((1/mgd_bar.getZ())*mgd_bar); pcd = A*((1/mcd_bar.getZ())*mcd_bar); ppd = A*((1/mpd_bar.getZ())*mpd_bar);// camera points as pixels
			alpha_red_d = mr_bar_ref.getZ()/mrd_bar.getZ();
			
			/********* desired wrt reference *********/
			//std::cout << "desired 1" << std::endl;
			
			/********* update reference*********/
			br.sendTransform(tf::StampedTransform(reference_wrt_world, current_time
							,"world", "reference_image"));
			
			//std::cout << "desired 2" << std::endl;
			
			/********* desired wrt reference *********/
			br.sendTransform(tf::StampedTransform(desired_wrt_world, current_time
							,"world","desired_image"));
			
			//std::cout << "desired 3" << std::endl;				
			
			tf::StampedTransform desired_wrt_reference;
			listener.waitForTransform("reference_image", "desired_image"
									 ,current_time, ros::Duration(5.0));
									 
			//std::cout << "desired 4" << std::endl;
			
			listener.lookupTransform("reference_image", "desired_image"
									,current_time, desired_wrt_reference);
			
			//std::cout << "desired 5" << std::endl;
			
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
		
		/********** velocity command from tracking **********/
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
			//std::cout << "alpha red: " << alpha_red << std::endl;
			//std::cout << "vc term1:\n x: " << vc_term1_temp.getX() << " y: " << vc_term1_temp.getY() << " z: " << vc_term1_temp.getZ() << std::endl;
			//std::cout << "vc term2:\n x: " << vc_term2_temp.getX() << " y: " << vc_term2_temp.getY() << " z: " << vc_term2_temp.getZ() << std::endl;
			
			tf::Vector3 vc_temp = vc_term1_temp + vc_term2_temp;// sum them together for vc
			vc = vc_temp;
			
		}
		/********** update the pixels and everything with them **********/
		void update_pixels()
		{
			tf::Vector3 mr_temp = A.inverse()*pr; 
			mr = mr_temp;//normalized pixel coordinates
			tf::Matrix3x3 ss_mr_temp(           0, -1*mr.getZ(),    mr.getY(),
										    mr.getZ(),            0, -1*mr.getX(),
										 -1*mr.getY(),    mr.getX(),           0);
			ss_mr = ss_mr_temp;// skew symmetrix mr
			tf::Vector3 mrd_temp = A.inverse()*prd; 
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
			//std::cout << "zr star hat dot: " << zr_star_hat_dot << std::endl;
			//std::cout << "time from last: " << time_from_last << std::endl;
			//std::cout << "zr star hat before: " << zr_star_hat << std::endl;
			zr_star_hat += zr_star_hat_dot*time_from_last;
			//std::cout << "zr star hat after: " << zr_star_hat << std::endl;
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
			
			//std::cout << "zr star hat dot sum: " << zr_star_hat_dot_sum << std::endl;
			//std::cout << "ev.dot(phi): " << ev.dot(phi) << std::endl;
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
			geometry_msgs::Twist velocity_command;
			velocity_command.linear.x = vc.getX();
			velocity_command.linear.y = vc.getY();
			velocity_command.linear.z = vc.getZ();
			velocity_command.angular.z = wc.getZ();
			cmd_vel_pub.publish(velocity_command);
			std::cout << "command updated" << std::endl;
		}

};

// main
int main(int argc, char** argv)
{   
	ros::init(argc,argv,"image_processing_node");
	ros::init(argc,argv,"homog_decomp_node");
	ros::init(argc,argv,"controller_node");
	
	double loop_rate_hz = 30;
	bool write_to_file = true;
	std::string filename = "/home/zack/v1_ws/src/homog_track/testing_files/test_with_nodes.txt";
	if( (std::remove( filename.c_str() ) != 0) && write_to_file)
	{
		std::cout << "file does not exist" << std::endl;
	}
	else
	{
		std::cout << "file deleted or not saving" << std::endl;
	}
	ImageProcessing image_processing(loop_rate_hz);// image processing 
	HomogDecomp homog_decomp;// homography decomp
	Controller controller(loop_rate_hz, write_to_file, filename);// controller
	
	ros::Rate loop_rate(loop_rate_hz);
	
	while (ros::ok())
	{
		/********* Update the transforms wrt world and current time **********/
		if (image_processing.camera_updated)
		{
			image_processing.pixel_pub.publish(image_processing.pixels_out);
			image_processing.camera_updated = false;
		}
		else
		{
			ros::spinOnce();
			continue;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
