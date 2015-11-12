#include <iostream>
#include <string>
#include <vector>
#include <cmath>
// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <homog_track/HomogMarker.h>
#include <homog_track/HomogComplete.h>
#include <homog_track/HomogDecomposed.h>
#include <homog_track/HomogDesired.h>
#include <homog_track/CameraDesiredStart.h>



// class to generate desired camera
class DesiredCamera
{
	public:
		bool display_calc_steps = true;
		ros::NodeHandle nh;// node for the image converter
		
		/********** Begin Topic Declarations **********/
		ros::Publisher desired_camera_pub;// publisher
		tf::TransformBroadcaster br;// tf broadcaster
		tf::TransformListener listener;
		/********** End Topic Declarations **********/
		
		/********** Begin Point Declarations **********/
		bool first_run = true;// boolean to tell if this is the first run
		tf::Vector3 P_red_wrt_world, P_green_wrt_world, P_cyan_wrt_world, P_purple_wrt_world;// homography points wrt world
		tf::Vector3 mrd_bar, mgd_bar, mcd_bar, mpd_bar;// points wrt desired
		tf::Vector3 prd, pgd, pcd, ppd;// pixels wrt desired
		tf::Vector3 mr_bar_ref, mg_bar_ref, mc_bar_ref, mp_bar_ref;// points wrt reference
		tf::Vector3 pr_ref, pg_ref, pc_ref, pp_ref;// pixels wrt reference
		tf::Transform reference_wrt_world,// reference image wrt world
					  desired_wrt_world;// transform for desired wrt world
		tf::Quaternion Q_dw, Q_df, Q_df_negated, Q_df_last;
		ros::Time last_virtual_update_time;// last time the virtual reference was updated
		homog_track::HomogDesired desired_msg;// message out
		tf::Matrix3x3 K;// camera matrix
		cv::Mat wcd;
		tf::Vector3 vcd;
		double desired_radius;// desired radius in meters
		double desired_period;// desired period
		/********** End Point Declarations **********/
		
		/********** constructor **********/
		DesiredCamera()
		{	
			desired_camera_pub = nh.advertise<homog_track::HomogDesired>("desired_homogrphy",1);// publisher for the desired reference
			K.setIdentity();//camera matrix
			wcd = cv::Mat::zeros(3,1,CV_64F);
			/********** markers in the world frame **********/
			// red:   (x,y,z) = (-0.05, -0.05, 0) m
			// green: (x,y,z) = ( -0.05, 0.05, 0) m
			// cyan:  (x,y,z) = ( 0.05,  0.05, 0) m
			// purple:  (x,y,z) = (0.05,  -0.05, 0) m
			P_red_wrt_world = tf::Vector3(-0.05,-0.05,0); P_green_wrt_world = tf::Vector3(-0.05,0.05,0); P_cyan_wrt_world = tf::Vector3(0.05,0.05,0); P_purple_wrt_world = tf::Vector3(0.05,-0.05,0);// homography points wrt world

			/********** initialize the transform of the reference wrt world  **********/
			double z_ref = 2; //reference height
			reference_wrt_world.setOrigin(tf::Vector3(0,0,z_ref));//origin
			tf::Matrix3x3 R_fw_tf(0,1,0,
								  1,0,0,
								  0,0,-1);// rotation of reference wrt world
			tf::Quaternion Q_fw_tf;// as a quaternion
			R_fw_tf.getRotation(Q_fw_tf);// initialize quaternion
			reference_wrt_world.setRotation(Q_fw_tf);// set the rotation
			
			/********** initialize the pixels in reference image  **********/
			tf::Vector3 temp_v;
			tf::Quaternion temp_Q;
			temp_v = P_red_wrt_world-reference_wrt_world.getOrigin(); temp_Q = ((reference_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*reference_wrt_world.getRotation(); mr_bar_ref = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-reference_wrt_world.getOrigin(); temp_Q = ((reference_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*reference_wrt_world.getRotation(); mg_bar_ref = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-reference_wrt_world.getOrigin(); temp_Q = ((reference_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*reference_wrt_world.getRotation(); mc_bar_ref = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-reference_wrt_world.getOrigin(); temp_Q = ((reference_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*reference_wrt_world.getRotation(); mp_bar_ref = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			pr_ref = K*((1/mr_bar_ref.getZ())*mr_bar_ref); pg_ref = K*((1/mg_bar_ref.getZ())*mg_bar_ref); pc_ref = K*((1/mc_bar_ref.getZ())*mc_bar_ref); pp_ref = K*((1/mp_bar_ref.getZ())*mp_bar_ref);// reference points as pixels
			
			/********** initialize the transform of the desired wrt world  **********/
			double z_init = 2; //starting desired height
			double cam_a0 = 0;// starting desired angle
			desired_radius = 1;// desired radius in meters
			desired_period = 30;
			desired_wrt_world.setOrigin(tf::Vector3(desired_radius,0,z_init));//origin
			tf::Matrix3x3 R_df_tf(std::cos(cam_a0),-std::sin(cam_a0),0,
								  std::sin(cam_a0),std::cos(cam_a0),0,
								  0,0,1);// rotation of desired wrt reference
			tf::Matrix3x3 R_dw_tf = R_fw_tf*R_df_tf;// rotation of camera wrt world
			tf::Quaternion Q_dw_tf;// as a quaternion
			R_dw_tf.getRotation(Q_dw_tf);// initialize quaternion
			desired_wrt_world.setRotation(Q_dw_tf);// set the rotation
			wcd.at<double>(2,0) = -2*M_PIl/desired_period;
			//wcd.at<double>(2,0) = 0;
			vcd = tf::Vector3(2*M_PIl*desired_radius/desired_period,0,0);
			//vcd = tf::Vector3(0,0,0);
			
			/********** initialize the pixels in camera image  **********/
			temp_v = P_red_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mrd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mgd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mcd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mpd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			prd = K*((1/mrd_bar.getZ())*mrd_bar); pgd = K*((1/mgd_bar.getZ())*mgd_bar); pcd = K*((1/mcd_bar.getZ())*mcd_bar); ppd = K*((1/mpd_bar.getZ())*mpd_bar);// camera points as pixels

			if (display_calc_steps)
			{
				std::cout << "desired wrt reference:"
						  << "\n row 0:"
						  << " x: " << R_df_tf.getRow(0).getX()
						  << " y: " << R_df_tf.getRow(0).getY()
						  << " z: " << R_df_tf.getRow(0).getZ()
						  << "\n row 1:"
						  << " x: " << R_df_tf.getRow(1).getX()
						  << " y: " << R_df_tf.getRow(1).getY()
						  << " z: " << R_df_tf.getRow(1).getZ()
						  << "\n row 2:"
						  << " x: " << R_df_tf.getRow(2).getX()
						  << " y: " << R_df_tf.getRow(2).getY()
						  << " z: " << R_df_tf.getRow(2).getZ()
						  <<std::endl;
				std::cout << "desired wrt world:"
						  << "\n row 0:"
						  << " x: " << R_dw_tf.getRow(0).getX()
						  << " y: " << R_dw_tf.getRow(0).getY()
						  << " z: " << R_dw_tf.getRow(0).getZ()
						  << "\n row 1:"
						  << " x: " << R_dw_tf.getRow(1).getX()
						  << " y: " << R_dw_tf.getRow(1).getY()
						  << " z: " << R_dw_tf.getRow(1).getZ()
						  << "\n row 2:"
						  << " x: " << R_dw_tf.getRow(2).getX()
						  << " y: " << R_dw_tf.getRow(2).getY()
						  << " z: " << R_dw_tf.getRow(2).getZ()
						  <<std::endl;
				std::cout << "Rotation, Fstar wrt World:" 
						  << "\n x: " << reference_wrt_world.getRotation().getX() 
						  << " y: " << reference_wrt_world.getRotation().getY()
						  << " z: " << reference_wrt_world.getRotation().getZ()
						  << " w: " << reference_wrt_world.getRotation().getW()
						  << std::endl;
				std::cout << "Origin, Fstar wrt World:" 
						  << "\n x: " << reference_wrt_world.getOrigin().getX()
						  << " y: " << reference_wrt_world.getOrigin().getY()
						  << " z: " << reference_wrt_world.getOrigin().getZ()
						  << std::endl;
				std::cout << "Rotation, Fd wrt World:" 
						  << "\n x: " << desired_wrt_world.getRotation().getX() 
						  << " y: " << desired_wrt_world.getRotation().getY()
						  << " z: " << desired_wrt_world.getRotation().getZ()
						  << " w: " << desired_wrt_world.getRotation().getW()
						  << std::endl;
				std::cout << "Origin, Fd wrt World:" 
						  << "\n x: " << desired_wrt_world.getOrigin().getX()
						  << " y: " << desired_wrt_world.getOrigin().getY()
						  << " z: " << desired_wrt_world.getOrigin().getZ()
						  << std::endl;
				std::cout << "Origin, Red wrt Fd:" 
						  << "\n x: " << mrd_bar.getX()
						  << " y: " << mrd_bar.getY()
						  << " z: " << mrd_bar.getZ()
						  << std::endl;
				std::cout << "Origin, Green wrt Fd:" 
						  << "\n x: " << mgd_bar.getX()
						  << " y: " << mgd_bar.getY()
						  << " z: " << mgd_bar.getZ()
						  << std::endl;
				std::cout << "Origin, Cyan wrt Fd:" 
						  << "\n x: " << mcd_bar.getX()
						  << " y: " << mcd_bar.getY()
						  << " z: " << mcd_bar.getZ()
						  << std::endl;
				std::cout << "Origin, Purple wrt Fd:" 
						  << "\n x: " << mpd_bar.getX()
						  << " y: " << mpd_bar.getY()
						  << " z: " << mpd_bar.getZ()
						  << std::endl;
				std::cout << "Pixels, Red in Fd:" 
						  << "\n x: " << prd.getX()
						  << " y: " << prd.getY()
						  << " z: " << prd.getZ()
						  << std::endl;
				std::cout << "Pixels, Green in Fd:" 
						  << "\n x: " << pgd.getX()
						  << " y: " << pgd.getY()
						  << " z: " << pgd.getZ()
						  << std::endl;
				std::cout << "Pixels, Cyan in Fd:" 
						  << "\n x: " << pcd.getX()
						  << " y: " << pcd.getY()
						  << " z: " << pcd.getZ()
						  << std::endl;
				std::cout << "Pixels, Purple in Fd:" 
						  << "\n x: " << ppd.getX()
						  << " y: " << ppd.getY()
						  << " z: " << ppd.getZ()
						  << std::endl;
				
			}
		}
				
		// update the virtual camera
		void update_pixels()
		{
			if (first_run)
			{
				first_run = false;
				last_virtual_update_time = ros::Time::now();//update time
				return;
			}
			
			/********** update the camera wrt world transform given the linear and angular velocities **********/
			double current_time = ros::Time::now().toSec();// update current time
			double time_diff = current_time - last_virtual_update_time.toSec(); // get the time difference
			Q_dw = desired_wrt_world.getRotation();// rotation of camera wrt world
			cv::Mat Q_dw_old = cv::Mat::zeros(4,1,CV_64F);
			Q_dw_old.at<double>(0,0) = Q_dw.getW(); Q_dw_old.at<double>(1,0) = Q_dw.getX(); Q_dw_old.at<double>(2,0) = Q_dw.getY(); Q_dw_old.at<double>(3,0) = Q_dw.getZ();
			cv::Mat Bd = cv::Mat::zeros(4,3,CV_64F);// differential matrix
			Bd.at<double>(0,0) = -Q_dw.getX(); Bd.at<double>(0,1) = -Q_dw.getY(); Bd.at<double>(0,2) = -Q_dw.getZ();
			Bd.at<double>(1,0) = Q_dw.getW(); Bd.at<double>(1,1) = -Q_dw.getZ(); Bd.at<double>(1,2) = Q_dw.getY();
			Bd.at<double>(2,0) = Q_dw.getZ(); Bd.at<double>(2,1) = Q_dw.getW(); Bd.at<double>(2,2) = -Q_dw.getX();
			Bd.at<double>(3,0) = -Q_dw.getY(); Bd.at<double>(3,1) = Q_dw.getX(); Bd.at<double>(3,2) = Q_dw.getW();
			cv::Mat Q_dw_dot = 0.5*(Bd*wcd);// rate of change of rotation of camera wrt world
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
			
			/********** update the pixels in camera image  **********/
			tf::Vector3 temp_v;
			tf::Quaternion temp_Q;
			temp_v = P_red_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mrd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mgd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mcd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mpd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			prd = K*((1/mrd_bar.getZ())*mrd_bar); pgd = K*((1/mgd_bar.getZ())*mgd_bar); pcd = K*((1/mcd_bar.getZ())*mcd_bar); ppd = K*((1/mpd_bar.getZ())*mpd_bar);// camera points as pixels
			last_virtual_update_time = ros::Time::now();//update time
			br.sendTransform(tf::StampedTransform(reference_wrt_world, last_virtual_update_time
										 ,"world","reference_image"));
			br.sendTransform(tf::StampedTransform(desired_wrt_world, last_virtual_update_time
										 ,"world","desired_image"));
										 
			tf::StampedTransform desired_wrt_reference;
			listener.waitForTransform("reference_image","desired_image",last_virtual_update_time,ros::Duration(5.0));
			listener.lookupTransform("reference_image","desired_image",last_virtual_update_time, desired_wrt_reference);
			
			Q_df = desired_wrt_reference.getRotation();
			Q_df_negated = tf::Quaternion(-Q_df.getX(),-Q_df.getY(),-Q_df.getZ(),-Q_df.getW()); // getting the negated version of the quaternion for the check
			// checking if the quaternion has flipped
			double Q_norm_current_diff = std::sqrt(std::pow(Q_df.getX() - Q_df_last.getX(),2.0)
										  + std::pow(Q_df.getY() - Q_df_last.getY(),2.0) 
										  + std::pow(Q_df.getZ() - Q_df_last.getZ(),2.0) 
										  + std::pow(Q_df.getW() - Q_df_last.getW(),2.0));
			double Q_norm_negated_diff = std::sqrt(std::pow(Q_df_negated.getX() - Q_df_last.getX(),2.0)
										  + std::pow(Q_df_negated.getY() - Q_df_last.getY(),2.0) 
										  + std::pow(Q_df_negated.getZ() - Q_df_last.getZ(),2.0) 
										  + std::pow(Q_df_negated.getW() - Q_df_last.getW(),2.0));
			
			if (display_calc_steps)
			{
				//std::cout << "time diff: " << time_diff << std::endl;
				//std::cout << "vcd:"
						  //<< "\n x: " << vcd.getX()
						  //<< " y: " << vcd.getY()
						  //<< " z: " << vcd.getZ()
						  //<< std::endl;
				//std::cout << "wcd:"
						  //<< "\n x: " << wcd.at<double>(0,0)
						  //<< " y: " << wcd.at<double>(1,0)
						  //<< " z: " << wcd.at<double>(2,0)
						  //<< std::endl;
				//std::cout << "Derivative, Rotation, Fd wrt World:\n"
						  ////<< Q_dw_dot << std::endl;
				//std::cout << "Norm of Q_dw: " << Q_dw_new_norm << std::endl;
				
				//std::cout << "Derivative, Origin, Fd wrt World:"
						  //<< "\n x: " << P_dw_dot.getX()
						  //<< " y: " << P_dw_dot.getY()
						  //<< " z: " << P_dw_dot.getZ()
						  //<< std::endl;
				//std::cout << "Derivative, Origin, Fd wrt World time time diff:"
						  //<< "\n x: " << (P_dw_dot*time_diff).getX()
						  //<< " y: " << (P_dw_dot*time_diff).getY()
						  //<< " z: " << (P_dw_dot*time_diff).getZ()
						  //<< std::endl;
				//std::cout << "Old Origin, Fd wrt World:"
						  //<< "\n x: " << P_dw.getX()
						  //<< " y: " << P_dw.getY()
						  //<< " z: " << P_dw.getZ()
						  //<< std::endl;
				//std::cout << "New Origin, Fd wrt World:"
						  //<< "\n x: " << P_dw_new.getX()
						  //<< " y: " << P_dw_new.getY()
						  //<< " z: " << P_dw_new.getZ()
						  //<< std::endl;
						  
				std::cout << "Rotation, Fd wrt Fstar:" 
						  << "\n x: " << Q_df.getX() 
						  << " y: " << Q_df.getY()
						  << " z: " << Q_df.getZ()
						  << " w: " << Q_df.getW()
						  << std::endl;
				std::cout << "Rotation, Fd wrt Fstar negated:" 
						  << "\n x: " << Q_df_negated.getX() 
						  << " y: " << Q_df_negated.getY()
						  << " z: " << Q_df_negated.getZ()
						  << " w: " << Q_df_negated.getW()
						  << std::endl;
				std::cout << "Rotation, Fd wrt Fstar last:" 
						  << "\n x: " << Q_df_last.getX() 
						  << " y: " << Q_df_last.getY()
						  << " z: " << Q_df_last.getZ()
						  << " w: " << Q_df_last.getW()
						  << std::endl;
				std::cout << "Current - Last norm: " <<  Q_norm_current_diff << std::endl;
				std::cout << "Negated - Last norm: " <<  Q_norm_negated_diff << std::endl;
				//std::cout << "Origin, Fstar wrt World:" 
						  //<< "\n x: " << reference_wrt_world.getOrigin().getX()
						  //<< " y: " << reference_wrt_world.getOrigin().getY()
						  //<< " z: " << reference_wrt_world.getOrigin().getZ()
						  //<< std::endl;
				//std::cout << "Rotation, Fd wrt World:" 
						  //<< "\n x: " << desired_wrt_world.getRotation().getX() 
						  //<< " y: " << desired_wrt_world.getRotation().getY()
						  //<< " z: " << desired_wrt_world.getRotation().getZ()
						  //<< " w: " << desired_wrt_world.getRotation().getW()
						  //<< std::endl;
				//std::cout << "Origin, Fd wrt World:" 
						  //<< "\n x: " << desired_wrt_world.getOrigin().getX()
						  //<< " y: " << desired_wrt_world.getOrigin().getY()
						  //<< " z: " << desired_wrt_world.getOrigin().getZ()
						  //<< std::endl;
				//std::cout << "Origin, Red wrt Fd:" 
						  //<< "\n x: " << mrd_bar.getX()
						  //<< " y: " << mrd_bar.getY()
						  //<< " z: " << mrd_bar.getZ()
						  //<< std::endl;
				//std::cout << "Origin, Green wrt Fd:" 
						  //<< "\n x: " << mgd_bar.getX()
						  //<< " y: " << mgd_bar.getY()
						  //<< " z: " << mgd_bar.getZ()
						  //<< std::endl;
				//std::cout << "Origin, Cyan wrt Fd:" 
						  //<< "\n x: " << mcd_bar.getX()
						  //<< " y: " << mcd_bar.getY()
						  //<< " z: " << mcd_bar.getZ()
						  //<< std::endl;
				//std::cout << "Origin, Purple wrt Fd:" 
						  //<< "\n x: " << mpd_bar.getX()
						  //<< " y: " << mpd_bar.getY()
						  //<< " z: " << mpd_bar.getZ()
						  //<< std::endl;
				//std::cout << "Pixels, Red in Fd:" 
						  //<< "\n x: " << prd.getX()
						  //<< " y: " << prd.getY()
						  //<< " z: " << prd.getZ()
						  //<< std::endl;
				//std::cout << "Pixels, Green in Fd:" 
						  //<< "\n x: " << pgd.getX()
						  //<< " y: " << pgd.getY()
						  //<< " z: " << pgd.getZ()
						  //<< std::endl;
				//std::cout << "Pixels, Cyan in Fd:" 
						  //<< "\n x: " << pcd.getX()
						  //<< " y: " << pcd.getY()
						  //<< " z: " << pcd.getZ()
						  //<< std::endl;
				//std::cout << "Pixels, Purple in Fd:" 
						  //<< "\n x: " << ppd.getX()
						  //<< " y: " << ppd.getY()
						  //<< " z: " << ppd.getZ()
						  //<< std::endl;
			}
			
			if (Q_norm_current_diff > Q_norm_negated_diff)
			{
				Q_df = Q_df_negated;
			}
			
			Q_df_last = Q_df;// updating the last
			desired_wrt_reference.setRotation(Q_df);
			std::cout << "Rotation, Fd wrt Fstar:" 
						  << "\n x: " << Q_df.getX() 
						  << " y: " << Q_df.getY()
						  << " z: " << Q_df.getZ()
						  << " w: " << Q_df.getW()
						  << std::endl;
			std::cout << "Rotation, Fd wrt Fstar from transform:" 
						  << "\n x: " << desired_wrt_reference.getRotation().getX() 
						  << " y: " << desired_wrt_reference.getRotation().getY()
						  << " z: " << desired_wrt_reference.getRotation().getZ()
						  << " w: " << desired_wrt_reference.getRotation().getW()
						  << std::endl;
			
			// output transform and message
			desired_msg.header.stamp = last_virtual_update_time;
			desired_msg.pose.position.x = desired_wrt_reference.getOrigin().getX(); desired_msg.pose.position.y = desired_wrt_reference.getOrigin().getY(); desired_msg.pose.position.z = desired_wrt_world.getOrigin().getZ(); // update position
			desired_msg.pose.orientation.x = Q_df.getX(); desired_msg.pose.orientation.y = Q_df.getY(); desired_msg.pose.orientation.z = Q_df.getZ(); desired_msg.pose.orientation.w = Q_df.getW();// update orientation
			desired_msg.header.frame_id = "desired wrt_reference";
			desired_msg.height.data = desired_wrt_world.getOrigin().getZ();
			desired_msg.updating_desired = true;
			desired_msg.red_circle.x = prd.getX(); desired_msg.red_circle.y = prd.getY(); desired_msg.red_circle.z = prd.getZ();
			desired_msg.green_circle.x = pgd.getX(); desired_msg.green_circle.y = pgd.getY(); desired_msg.green_circle.z = pgd.getZ();
			desired_msg.cyan_circle.x = pcd.getX(); desired_msg.cyan_circle.y = pcd.getY(); desired_msg.cyan_circle.z = pcd.getZ();
			desired_msg.purple_circle.x = ppd.getX(); desired_msg.purple_circle.y = ppd.getY(); desired_msg.purple_circle.z = ppd.getZ();
			desired_msg.omega_cd.x = wcd.at<double>(0,0); desired_msg.omega_cd.y = wcd.at<double>(1,0); desired_msg.omega_cd.z = wcd.at<double>(2,0);
			desired_msg.v_cd.x = vcd.getX(); desired_msg.v_cd.y = vcd.getY(); desired_msg.v_cd.z = vcd.getZ();
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

	ros::Rate loop_rate(1000);
	while (ros::ok())
	{
		desired_camera.update_pixels();
		ros::spinOnce();
		loop_rate.sleep();
	}
    
    return 0;
}

