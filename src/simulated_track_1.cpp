#include <iostream>
#include <string>
#include <vector>
#include <cmath>

// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <homog_track/HomogMarker.h>
#include <homog_track/HomogDecomposed.h>
#include <homog_track/HomogComplete.h>
#include <homog_track/HomogReference.h>


// class to convert images
class ImageConverter
{
	public:
		bool display_calc_steps = true;
		ros::NodeHandle nh;// node for the image converter
		
		/********** Begin Topic Declarations **********/
		ros::Publisher circle_pub;// circles publishers
		ros::Subscriber virtual_camera_sub;// virtual camera subscriber
		ros::Subscriber homog_decomp_sub;// decomp subscriber
		ros::ServiceServer ref_service;// reference service
		tf::TransformBroadcaster br;// tf broadcaster
		/********** End Topic Declarations **********/
		
		/********** Begin Point Declarations **********/
		bool first_run = true;// boolean to tell if this is the first run
		tf::Vector3 P_red_wrt_world, P_green_wrt_world, P_cyan_wrt_world, P_purple_wrt_world;// homography points wrt world
		tf::Vector3 mr_bar, mg_bar, mc_bar, mp_bar;// points wrt camera
		tf::Vector3 pr, pg, pc, pp;// pixels wrt camera
		tf::Vector3 mr_bar_ref, mg_bar_ref, mc_bar_ref, mp_bar_ref;// points wrt reference
		tf::Vector3 pr_ref, pg_ref, pc_ref, pp_ref;// pixels wrt reference
		tf::Transform reference_wrt_world,// reference image wrt world
					  camera_wrt_world,// transform for camera wrt world
					  red_wrt_world,
					  green_wrt_world,
					  cyan_wrt_world,
					  purple_wrt_world;//transforms for the marker points
		ros::Time last_virtual_update_time;// last time the virtual reference was updated
		homog_track::HomogMarker ref_marker,// reference marker points
								 marker;// virtual
		homog_track::HomogComplete complete_msg;// message out
		tf::Matrix3x3 K;// camera matrix
		/********** End Point Declarations **********/
		
		/********** constructor **********/
		ImageConverter()
		{	
			circle_pub = nh.advertise<homog_track::HomogComplete>("complete_homog_set",1);//Initialize the publisher for the 4 circles
			virtual_camera_sub = nh.subscribe("/cmd_vel",1, &ImageConverter::virtual_camera_callback, this);
			ref_marker.red_circle.x = -1;ref_marker.red_circle.y = -1;ref_marker.red_circle.z = -1;//red
			ref_marker.green_circle.x = -1;ref_marker.green_circle.y = -1;ref_marker.green_circle.z = -1;//green
			ref_marker.cyan_circle.x = -1;ref_marker.cyan_circle.y = -1;ref_marker.cyan_circle.z = -1;//cyan
			ref_marker.purple_circle.x = -1;ref_marker.purple_circle.y = -1;ref_marker.purple_circle.z = -1;//purple
			K.setIdentity();//camera matrix
			last_virtual_update_time = ros::Time::now();//update time
			/********** markers in the world frame **********/
			// red:   (x,y,z) = (-0.05, -0.05, 0) m
			// green: (x,y,z) = ( -0.05, 0.05, 0) m
			// cyan:  (x,y,z) = ( 0.05,  0.05, 0) m
			// purple:  (x,y,z) = (0.05,  -0.05, 0) m
			P_red_wrt_world = tf::Vector3(-0.05,-0.05,0); P_green_wrt_world = tf::Vector3(-0.05,0.05,0); P_cyan_wrt_world = tf::Vector3(0.05,0.05,0); P_purple_wrt_world = tf::Vector3(0.05,-0.05,0);// homography points wrt world
			red_wrt_world.setIdentity(); red_wrt_world.setOrigin(P_red_wrt_world);//red
			green_wrt_world.setIdentity(); green_wrt_world.setOrigin(P_green_wrt_world);//green
			cyan_wrt_world.setIdentity(); cyan_wrt_world.setOrigin(P_cyan_wrt_world);//cyan
			purple_wrt_world.setIdentity(); purple_wrt_world.setOrigin(P_purple_wrt_world);//purple

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
			ref_marker.red_circle.x = pr_ref.getX(); ref_marker.red_circle.y = pr_ref.getY(); ref_marker.red_circle.z = pr_ref.getZ();// red
			ref_marker.green_circle.x = pg_ref.getX(); ref_marker.green_circle.y = pg_ref.getY(); ref_marker.green_circle.z = pg_ref.getZ();// green
			ref_marker.cyan_circle.x = pc_ref.getX(); ref_marker.cyan_circle.y = pc_ref.getY(); ref_marker.cyan_circle.z = pc_ref.getZ();// cyan
			ref_marker.purple_circle.x = pp_ref.getX(); ref_marker.purple_circle.y = pp_ref.getY(); ref_marker.purple_circle.z = pp_ref.getZ();// purple
			
			/********** initialize the transform of the camera wrt world  **********/
			double z_init = 3; //starting camera height
			double cam_a0 = M_PIl/2;
			camera_wrt_world.setOrigin(tf::Vector3(1,1,z_init));//origin
			tf::Matrix3x3 R_cf_tf(std::cos(cam_a0),-std::sin(cam_a0),0,
								  std::sin(cam_a0),std::cos(cam_a0),0,
								  0,0,1);// rotation of camera wrt reference
			tf::Matrix3x3 R_cw_tf = R_fw_tf*R_cf_tf;// rotation of camera wrt world
			tf::Quaternion Q_cw_tf;// as a quaternion
			R_cw_tf.getRotation(Q_cw_tf);// initialize quaternion
			camera_wrt_world.setRotation(Q_cw_tf);// set the rotation
			
			/********** initialize the pixels in camera image  **********/
			temp_v = P_red_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mr_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mg_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mc_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mp_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			pr = K*((1/mr_bar.getZ())*mr_bar); pg = K*((1/mg_bar.getZ())*mg_bar); pc = K*((1/mc_bar.getZ())*mc_bar); pp = K*((1/mp_bar.getZ())*mp_bar);// camera points as pixels
			marker.red_circle.x = pr.getX(); marker.red_circle.y = pr.getY(); marker.red_circle.z = pr.getZ();// red
			marker.green_circle.x = pg.getX(); marker.green_circle.y = pg.getY(); marker.green_circle.z = pg.getZ();// green
			marker.cyan_circle.x = pc.getX(); marker.cyan_circle.y = pc.getY(); marker.cyan_circle.z = pc.getZ();// cyan
			marker.purple_circle.x = pp.getX(); marker.purple_circle.y = pp.getY(); marker.purple_circle.z = pp.getZ();// purple
			
			if (display_calc_steps)
			{
				std::cout << "camera wrt reference:"
						  << "\n row 0:"
						  << " x: " << R_cf_tf.getRow(0).getX()
						  << " y: " << R_cf_tf.getRow(0).getY()
						  << " z: " << R_cf_tf.getRow(0).getZ()
						  << "\n row 1:"
						  << " x: " << R_cf_tf.getRow(1).getX()
						  << " y: " << R_cf_tf.getRow(1).getY()
						  << " z: " << R_cf_tf.getRow(1).getZ()
						  << "\n row 2:"
						  << " x: " << R_cf_tf.getRow(2).getX()
						  << " y: " << R_cf_tf.getRow(2).getY()
						  << " z: " << R_cf_tf.getRow(2).getZ()
						  <<std::endl;
				std::cout << "camera wrt world:"
						  << "\n row 0:"
						  << " x: " << R_cw_tf.getRow(0).getX()
						  << " y: " << R_cw_tf.getRow(0).getY()
						  << " z: " << R_cw_tf.getRow(0).getZ()
						  << "\n row 1:"
						  << " x: " << R_cw_tf.getRow(1).getX()
						  << " y: " << R_cw_tf.getRow(1).getY()
						  << " z: " << R_cw_tf.getRow(1).getZ()
						  << "\n row 2:"
						  << " x: " << R_cw_tf.getRow(2).getX()
						  << " y: " << R_cw_tf.getRow(2).getY()
						  << " z: " << R_cw_tf.getRow(2).getZ()
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
				std::cout << "Rotation, F wrt World:" 
						  << "\n x: " << camera_wrt_world.getRotation().getX() 
						  << " y: " << camera_wrt_world.getRotation().getY()
						  << " z: " << camera_wrt_world.getRotation().getZ()
						  << " w: " << camera_wrt_world.getRotation().getW()
						  << std::endl;
				std::cout << "Origin, F wrt World:" 
						  << "\n x: " << camera_wrt_world.getOrigin().getX()
						  << " y: " << camera_wrt_world.getOrigin().getY()
						  << " z: " << camera_wrt_world.getOrigin().getZ()
						  << std::endl;
				std::cout << "Origin, Red wrt F:" 
						  << "\n x: " << mr_bar.getX()
						  << " y: " << mr_bar.getY()
						  << " z: " << mr_bar.getZ()
						  << std::endl;
				std::cout << "Origin, Green wrt F:" 
						  << "\n x: " << mg_bar.getX()
						  << " y: " << mg_bar.getY()
						  << " z: " << mg_bar.getZ()
						  << std::endl;
				std::cout << "Origin, Cyan wrt F:" 
						  << "\n x: " << mc_bar.getX()
						  << " y: " << mc_bar.getY()
						  << " z: " << mc_bar.getZ()
						  << std::endl;
				std::cout << "Origin, Purple wrt F:" 
						  << "\n x: " << mp_bar.getX()
						  << " y: " << mp_bar.getY()
						  << " z: " << mp_bar.getZ()
						  << std::endl;
				std::cout << "Pixels, Red in F:" 
						  << "\n x: " << pr.getX()
						  << " y: " << pr.getY()
						  << " z: " << pr.getZ()
						  << std::endl;
				std::cout << "Pixels, Green in F:" 
						  << "\n x: " << pg.getX()
						  << " y: " << pg.getY()
						  << " z: " << pg.getZ()
						  << std::endl;
				std::cout << "Pixels, Cyan in F:" 
						  << "\n x: " << pc.getX()
						  << " y: " << pc.getY()
						  << " z: " << pc.getZ()
						  << std::endl;
				std::cout << "Pixels, Purple in F:" 
						  << "\n x: " << pp.getX()
						  << " y: " << pp.getY()
						  << " z: " << pp.getZ()
						  << std::endl;
				
			}
		}
				
		// update the virtual camera
		void virtual_camera_callback(const geometry_msgs::Twist& msg)
		{
			//if (first_run)
			//{
				//first_run = false;
				//last_virtual_update_time = ros::Time::now();//update time
				
			//}
			/********** update the camera wrt world transform given the linear and angular velocities **********/
			double current_time = ros::Time::now().toSec();// update current time
			
			double time_diff = current_time - last_virtual_update_time.toSec(); // get the time difference
			tf::Vector3 vc = tf::Vector3(msg.linear.x, msg.linear.y, msg.linear.z);// linear velocity of the camera wrt reference expressed in camera
			cv::Mat wc = cv::Mat::zeros(3,1,CV_64F); wc.at<double>(2,0) = msg.angular.z;// angular velocity of camera wrt reference expressed in camera
			tf::Quaternion Q_cw = camera_wrt_world.getRotation();// rotation of camera wrt world
			cv::Mat Q_cw_old = cv::Mat::zeros(4,1,CV_64F);
			Q_cw_old.at<double>(0,0) = Q_cw.getW(); Q_cw_old.at<double>(1,0) = Q_cw.getX(); Q_cw_old.at<double>(2,0) = Q_cw.getY(); Q_cw_old.at<double>(3,0) = Q_cw.getZ();
			cv::Mat B = cv::Mat::zeros(4,3,CV_64F);// differential matrix
			B.at<double>(0,0) = -Q_cw.getX(); B.at<double>(0,1) = -Q_cw.getY(); B.at<double>(0,2) = -Q_cw.getZ();
			B.at<double>(1,0) = Q_cw.getW(); B.at<double>(1,1) = -Q_cw.getZ(); B.at<double>(1,2) = Q_cw.getY();
			B.at<double>(2,0) = Q_cw.getZ(); B.at<double>(2,1) = Q_cw.getW(); B.at<double>(2,2) = -Q_cw.getX();
			B.at<double>(3,0) = -Q_cw.getY(); B.at<double>(3,1) = Q_cw.getX(); B.at<double>(3,2) = Q_cw.getW();
			cv::Mat Q_cw_dot = 0.5*(B*wc);// rate of change of rotation of camera wrt world
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
			
			/********** initialize the pixels in camera image  **********/
			tf::Vector3 temp_v;
			tf::Quaternion temp_Q;
			temp_v = P_red_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mr_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mg_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mc_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-camera_wrt_world.getOrigin(); temp_Q = ((camera_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*camera_wrt_world.getRotation(); mp_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			pr = K*((1/mr_bar.getZ())*mr_bar); pg = K*((1/mg_bar.getZ())*mg_bar); pc = K*((1/mc_bar.getZ())*mc_bar); pp = K*((1/mp_bar.getZ())*mp_bar);// camera points as pixels
			marker.red_circle.x = pr.getX(); marker.red_circle.y = pr.getY(); marker.red_circle.z = pr.getZ();// red
			marker.green_circle.x = pg.getX(); marker.green_circle.y = pg.getY(); marker.green_circle.z = pg.getZ();// green
			marker.cyan_circle.x = pc.getX(); marker.cyan_circle.y = pc.getY(); marker.cyan_circle.z = pc.getZ();// cyan
			marker.purple_circle.x = pp.getX(); marker.purple_circle.y = pp.getY(); marker.purple_circle.z = pp.getZ();// purple
			
			last_virtual_update_time = ros::Time::now();//update time
			
			if (display_calc_steps)
			{
				std::cout << "time diff: " << time_diff << std::endl;
				std::cout << "vc:"
						  << "\n x: " << vc.getX()
						  << " y: " << vc.getY()
						  << " z: " << vc.getZ()
						  << std::endl;
				std::cout << "wc:"
						  << "\n x: " << wc.at<double>(0,0)
						  << " y: " << wc.at<double>(1,0)
						  << " z: " << wc.at<double>(2,0)
						  << std::endl;
				std::cout << "Derivative, Rotation, F wrt World:\n"
						  << Q_cw_dot << std::endl;
				std::cout << "Norm of Q_cw: " << Q_cw_new_norm << std::endl;
				
				std::cout << "Derivative, Origin, F wrt World:"
						  << "\n x: " << P_cw_dot.getX()
						  << " y: " << P_cw_dot.getY()
						  << " z: " << P_cw_dot.getZ()
						  << std::endl;
				std::cout << "Derivative, Origin, F wrt World time time diff:"
						  << "\n x: " << (P_cw_dot*time_diff).getX()
						  << " y: " << (P_cw_dot*time_diff).getY()
						  << " z: " << (P_cw_dot*time_diff).getZ()
						  << std::endl;
				std::cout << "Old Origin, F wrt World:"
						  << "\n x: " << P_cw.getX()
						  << " y: " << P_cw.getY()
						  << " z: " << P_cw.getZ()
						  << std::endl;
				std::cout << "New Origin, F wrt World:"
						  << "\n x: " << P_cw_new.getX()
						  << " y: " << P_cw_new.getY()
						  << " z: " << P_cw_new.getZ()
						  << std::endl;
						  
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
				std::cout << "Rotation, F wrt World:" 
						  << "\n x: " << camera_wrt_world.getRotation().getX() 
						  << " y: " << camera_wrt_world.getRotation().getY()
						  << " z: " << camera_wrt_world.getRotation().getZ()
						  << " w: " << camera_wrt_world.getRotation().getW()
						  << std::endl;
				std::cout << "Origin, F wrt World:" 
						  << "\n x: " << camera_wrt_world.getOrigin().getX()
						  << " y: " << camera_wrt_world.getOrigin().getY()
						  << " z: " << camera_wrt_world.getOrigin().getZ()
						  << std::endl;
				std::cout << "Origin, Red wrt F:" 
						  << "\n x: " << mr_bar.getX()
						  << " y: " << mr_bar.getY()
						  << " z: " << mr_bar.getZ()
						  << std::endl;
				std::cout << "Origin, Green wrt F:" 
						  << "\n x: " << mg_bar.getX()
						  << " y: " << mg_bar.getY()
						  << " z: " << mg_bar.getZ()
						  << std::endl;
				std::cout << "Origin, Cyan wrt F:" 
						  << "\n x: " << mc_bar.getX()
						  << " y: " << mc_bar.getY()
						  << " z: " << mc_bar.getZ()
						  << std::endl;
				std::cout << "Origin, Purple wrt F:" 
						  << "\n x: " << mp_bar.getX()
						  << " y: " << mp_bar.getY()
						  << " z: " << mp_bar.getZ()
						  << std::endl;
				std::cout << "Pixels, Red in F:" 
						  << "\n x: " << pr.getX()
						  << " y: " << pr.getY()
						  << " z: " << pr.getZ()
						  << std::endl;
				std::cout << "Pixels, Green in F:" 
						  << "\n x: " << pg.getX()
						  << " y: " << pg.getY()
						  << " z: " << pg.getZ()
						  << std::endl;
				std::cout << "Pixels, Cyan in F:" 
						  << "\n x: " << pc.getX()
						  << " y: " << pc.getY()
						  << " z: " << pc.getZ()
						  << std::endl;
				std::cout << "Pixels, Purple in F:" 
						  << "\n x: " << pp.getX()
						  << " y: " << pp.getY()
						  << " z: " << pp.getZ()
						  << std::endl;
			}
		}
		
};

// main
int main(int argc, char** argv)
{   
	// initialize node
	ros::init(argc,argv,"tracking_node");
	
	// initialize image converter    
	ImageConverter image_converter;

	ros::Rate loop_rate(30);
	while (ros::ok())
	{
		// giving the virtual circles to the output message
		//std::cout << "last virtual update time:\n" << image_converter.last_virtual_update_time << std::endl;
		image_converter.marker.header.stamp = image_converter.last_virtual_update_time;
		image_converter.ref_marker.header.stamp = image_converter.last_virtual_update_time;
		image_converter.complete_msg.header.stamp = image_converter.last_virtual_update_time;
		image_converter.complete_msg.reference_set = true;
		image_converter.complete_msg.current_points = image_converter.marker;
		image_converter.complete_msg.reference_points = image_converter.ref_marker;
		
		// publish the circle points
		image_converter.circle_pub.publish(image_converter.complete_msg);
		
		//image_converter.br.sendTransform(tf::StampedTransform(image_converter.reference_wrt_world, image_converter.last_virtual_update_time
										 //,"world", "reference_image"));
		image_converter.br.sendTransform(tf::StampedTransform(image_converter.camera_wrt_world, image_converter.last_virtual_update_time
										 ,"world","current_image"));
		image_converter.br.sendTransform(tf::StampedTransform(image_converter.red_wrt_world, image_converter.last_virtual_update_time
										 ,"world", "red_feature"));
		image_converter.br.sendTransform(tf::StampedTransform(image_converter.green_wrt_world, image_converter.last_virtual_update_time
										 ,"world","green_feature"));
		image_converter.br.sendTransform(tf::StampedTransform(image_converter.cyan_wrt_world, image_converter.last_virtual_update_time
										 ,"world", "cyan_feature"));
		image_converter.br.sendTransform(tf::StampedTransform(image_converter.purple_wrt_world, image_converter.last_virtual_update_time
										 ,"world","purple_feature"));
		
		//current_wrt_reference;
		//std::cout << "complete message\n" << image_converter.complete_msg << std::endl << std::endl;
		ros::spinOnce();
		loop_rate.sleep();
	}
    
    return 0;
}

