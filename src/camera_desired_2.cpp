
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

////////////addddd the package and stuff

//#define PI 3.14159265

class DesiredCamera
{
	public:	
		// node handle
		ros::NodeHandle nh;
		
		// publisher
		ros::Publisher desired_camera_pub;
		
		// desired start service
		ros::ServiceServer desired_start_service;
		
		/********** Desired Declarations **********/
		// start desired publishing
		bool start_publishing = false;
		
		// radius of 0.5 and height of 2 should work
		
		// desired radius in meters
		double radius = 0.5;
		
		// distance off the ground in meters
		std_msgs::Float64 height;
		
		// desired period
		double period = 30;
		
		// angular rate of change of the circle is then
		//double omega_cd = -2.0*M_PIl/period;
		double omega_cd = 0;
		
		// magnitude of the linear velocity
		double v_cd = -radius*omega_cd;
		
		// vecotrs for the angular desired velocity and the lineard desired velocity
		geometry_msgs::Vector3 omega_cd_v;
		geometry_msgs::Vector3 vcd_v;
		 
		// start time
		double start_time;
		
		// current time
		double current_time;
		
		// time difference
		double time_diff = 0;
		
		// desired angle
		double theta = omega_cd*time_diff;
		
		// transform between the desired and the reference
		cv::Mat T_fd_fstar;
		
		// the camera matrix
		cv::Mat K;
		
		// location of the red marker to track 4 element, last element is 1
		cv::Mat m_r_4;
		
		// location of the green marker to track 4 element, last element is 1
		cv::Mat m_g_4;
		
		// location of the cyan marker to track 4 element, last element is 1
		cv::Mat m_c_4;
		
		// location of the purple marker to track 4 element, last element is 1
		cv::Mat m_p_4;
		
		// vector of the four matrices
		std::vector<cv::Mat> marker_locations_4;
		
		// temp 4 element for the conversion
		cv::Mat marker_temp_4;
		
		// temp 3 element for the conversion
		cv::Mat marker_temp_3;
		
		// pixel locations
		std::vector<cv::Mat> pixel_locations;
		
		// geometry message for the pixel location
		geometry_msgs::Point p_r_gm;
		geometry_msgs::Point p_g_gm;
		geometry_msgs::Point p_c_gm;
		geometry_msgs::Point p_p_gm;
		
		// pose message for the desired pose
		tf::Matrix3x3 R_df_tf;
		tf::Quaternion Q_df_tf = tf::Quaternion(0,0,0,0);;
		tf::Quaternion  Q_df_tf_last = tf::Quaternion(0,0,0,0);
		tf::Quaternion  Q_df_tf_negated = tf::Quaternion(0,0,0,0);
		double Q_norm_current_diff = 0;
		double Q_norm_negated_diff = 0;
		tf::Vector3 T_df_tf;
		geometry_msgs::Quaternion Q_df_gm;
		geometry_msgs::Point P_df_gm;
		geometry_msgs::Pose pose_df_gm;
		
		// message
		homog_track::HomogDesired desired_msg;
		
		DesiredCamera()
		{
			// publisher for the desired reference
			desired_camera_pub = nh.advertise<homog_track::HomogDesired>("desired_homogrphy",1);
			
			// initialize the service
			desired_start_service = nh.advertiseService("start_desired", &DesiredCamera::set_desired_service_handler,this);
			
			// desired height from desired to reference
			height.data = -6;
			
			// camera matrix for the ardrone
			K = cv::Mat::zeros(3,3,CV_64F);
			//first row
			K.at<double>(0,0) = 567.79;
			K.at<double>(0,2) = 337.35;
			// second row
			K.at<double>(1,1) = 564.52;
			K.at<double>(1,2) = 169.54;
			// third row
			K.at<double>(2,2) = 1;
			
			// red marker location in refernece frame
			m_r_4 = cv::Mat::zeros(4,1,CV_64F);
			// x is at -0.05 m
			m_r_4.at<double>(0,0) = -0.05;
			// y is at -0.05 m
			m_r_4.at<double>(1,0) = -0.05;
			// z is at 2 m
			m_r_4.at<double>(2,0) = height.data;
			// 1 for multiplying with the transform
			m_r_4.at<double>(3,0) = 1;
			
			// green marker location in refernece frame
			m_g_4 = cv::Mat::zeros(4,1,CV_64F);
			// x is at -0.05 m
			m_g_4.at<double>(0,0) = 0.05;
			// y is at -0.05 m
			m_g_4.at<double>(1,0) = -0.05;
			// z is at 2 m
			m_g_4.at<double>(2,0) = height.data;
			// 1 for multiplying with the transform
			m_g_4.at<double>(3,0) = 1;
			
			// cyan marker location in refernece frame
			m_c_4 = cv::Mat::zeros(4,1,CV_64F);
			// x is at -0.05 m
			m_c_4.at<double>(0,0) = 0.05;
			// y is at -0.05 m
			m_c_4.at<double>(1,0) = 0.05;
			// z is at 2 m
			m_c_4.at<double>(2,0) = height.data;
			// 1 for multiplying with the transform
			m_c_4.at<double>(3,0) = 1;
			
			// purple marker location in refernece frame
			m_p_4 = cv::Mat::zeros(4,1,CV_64F);
			// x is at -0.05 m
			m_p_4.at<double>(0,0) = -0.05;
			// y is at -0.05 m
			m_p_4.at<double>(1,0) = 0.05;
			// z is at 2 m
			m_p_4.at<double>(2,0) = height.data;
			// 1 for multiplying with the transform
			m_p_4.at<double>(3,0) = 1;
			
			// putting the marker locations onto the vector
			marker_locations_4.push_back(m_r_4);
			marker_locations_4.push_back(m_g_4);
			marker_locations_4.push_back(m_c_4);
			marker_locations_4.push_back(m_p_4);
			
			// initializing the transformation
			T_fd_fstar = cv::Mat::zeros(4,4,CV_64F);
			
			// initializing the temp markers
			marker_temp_4 = cv::Mat::zeros(4,1,CV_64F);
			marker_temp_3 = cv::Mat::zeros(3,1,CV_64F);
			
		}
		
		// service callback/handler
		bool set_desired_service_handler(homog_track::CameraDesiredStart::Request &req, homog_track::CameraDesiredStart::Response &res)
		{
			// if the boolean is false setting the boolean to true so it will start tracking and setting
			// the start time to now otherwise stopping the desired reference
			if (!start_publishing)
			{
				start_publishing = true;
				start_time = ros::Time::now().toSec();
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
				// getting the current time
				current_time = ros::Time::now().toSec();
				
				// getting the time difference
				time_diff = current_time - start_time;
				std::cout << "\ntime difference:\t" << time_diff << std::endl;
				
				//  getting the current theta
				theta = omega_cd*time_diff;
				//theta = M_PIl/6;
				//theta = -2*M_PIl;
				
				// wrapping the angle, if its spinning positively will subtract and add if spining negatively, tolerance at low value
				while (std::abs(theta) >= 2.0*M_PIl)
				{
					std::cout << theta << std::endl;
					if (omega_cd > 0)
					{
						//if ()
						theta -= 2.0*M_PIl;
					}
					else
					{
						theta += 2.0*M_PIl;
					}
				}
				
				std::cout << "theta current:\t" << theta*180.0/M_PIl << std::endl;
				
				// reinitializing the transformation
				T_fd_fstar = cv::Mat::zeros(4,4,CV_64F);
				
				// first row
				T_fd_fstar.at<double>(0,0) = cos(theta);
				T_fd_fstar.at<double>(0,1) = sin(theta);
				T_fd_fstar.at<double>(0,3) = -1*radius;
				// second row
				T_fd_fstar.at<double>(1,0) = -1*sin(theta);
				T_fd_fstar.at<double>(1,1) = cos(theta);
				//// first row
				//T_fd_fstar.at<double>(0,0) = 1;
				//T_fd_fstar.at<double>(0,1) = 0;
				//T_fd_fstar.at<double>(0,3) = -1*radius;
				//// second row
				//T_fd_fstar.at<double>(1,0) = 0;
				//T_fd_fstar.at<double>(1,1) = 1;
				// third row
				T_fd_fstar.at<double>(2,2) = 1;
				T_fd_fstar.at<double>(2,3) = height.data;
				//fourth row
				T_fd_fstar.at<double>(3,3) = 1;
				
				
				//std::cout << "current transform\n" << T_fd_fstar << std::endl;
				
				// erasing all the pixel locations from the vector
				pixel_locations.erase(pixel_locations.begin(),pixel_locations.end());
				//std::cout << "pixel size\t" << pixel_locations.size() << std::endl;
				
				// updating the temp values
				for (int ii = 0; ii < 4; ii++)
				{
					// get the nect marker new location
					marker_temp_4 = T_fd_fstar*marker_locations_4[ii];
					
					// normalize the marker
					marker_temp_3.at<double>(0,0) = marker_temp_4.at<double>(0,0)/marker_temp_4.at<double>(2,0);
					marker_temp_3.at<double>(1,0) = marker_temp_4.at<double>(1,0)/marker_temp_4.at<double>(2,0);
					marker_temp_3.at<double>(2,0) = marker_temp_4.at<double>(2,0)/marker_temp_4.at<double>(2,0);
					
					// convert to pixels and add to the vector
					pixel_locations.push_back(K*marker_temp_3);
					//std::cout << "marker " << ii << "\n" << pixel_locations[ii] << std::endl;
				}
				// initializing the markers
				p_r_gm.x = pixel_locations[0].at<double>(0,0);
				p_r_gm.y = pixel_locations[0].at<double>(1,0);
				p_r_gm.z = pixel_locations[0].at<double>(2,0);
				
				p_g_gm.x = pixel_locations[1].at<double>(0,0);
				p_g_gm.y = pixel_locations[1].at<double>(1,0);
				p_g_gm.z = pixel_locations[1].at<double>(2,0);
				
				p_c_gm.x = pixel_locations[2].at<double>(0,0);
				p_c_gm.y = pixel_locations[2].at<double>(1,0);
				p_c_gm.z = pixel_locations[2].at<double>(2,0);
				
				p_p_gm.x = pixel_locations[3].at<double>(0,0);
				p_p_gm.y = pixel_locations[3].at<double>(1,0);
				p_p_gm.z = pixel_locations[3].at<double>(2,0);
				
				// getting the desired pose message
				// converting the rotation from a cv matrix to quaternion, first need it as a matrix3x3
				R_df_tf[0][0] = T_fd_fstar.at<double>(0,0);
				R_df_tf[0][1] = T_fd_fstar.at<double>(0,1);
				R_df_tf[0][2] = T_fd_fstar.at<double>(0,2);
				R_df_tf[1][0] = T_fd_fstar.at<double>(1,0);
				R_df_tf[1][1] = T_fd_fstar.at<double>(1,1);
				R_df_tf[1][2] = T_fd_fstar.at<double>(1,2);
				R_df_tf[2][0] = T_fd_fstar.at<double>(2,0);
				R_df_tf[2][1] = T_fd_fstar.at<double>(2,1);
				R_df_tf[2][2] = T_fd_fstar.at<double>(2,2);
				
				// converting the translation to a vector 3
				T_df_tf.setX(T_fd_fstar.at<double>(0,3));
				T_df_tf.setY(T_fd_fstar.at<double>(1,3));
				T_df_tf.setZ(T_fd_fstar.at<double>(2,3));
				
				std::cout << "Final transform:\n" << T_fd_fstar << std::endl;
				
				//std::cout << "rotatation matrix:"
						  //<< "\n\txx:\t" << R_df_tf.getColumn(0).getX()
						  //<< "\n\txy:\t" << R_df_tf.getColumn(0).getY()
						  //<< "\n\txz:\t" << R_df_tf.getColumn(0).getZ()
						  //<< "\n\tyx:\t" << R_df_tf.getColumn(1).getX()
						  //<< "\n\tyy:\t" << R_df_tf.getColumn(1).getY()
						  //<< "\n\tyz:\t" << R_df_tf.getColumn(1).getZ()
						  //<< "\n\tzx:\t" << R_df_tf.getColumn(2).getX()
						  //<< "\n\tzy:\t" << R_df_tf.getColumn(2).getY()
						  //<< "\n\tzz:\t" << R_df_tf.getColumn(2).getZ()
						  //<< std::endl;
						  
				//std::cout << "norms of the rotation:" << "\n\tcx:\t" << R_df_tf.getColumn(0).length() 
													  //<< "\n\tcy:\t" << R_df_tf.getColumn(1).length()
													  //<< "\n\tcz:\t" << R_df_tf.getColumn(2).length()
													  //<< "\n\trx:\t" << R_df_tf.getRow(0).length()
													  //<< "\n\try:\t" << R_df_tf.getRow(1).length()
													  //<< "\n\trz:\t" << R_df_tf.getRow(2).length()
													  //<< std::endl;
				
				//std::cout << "dot products of the rotation:" << "\n\tc1c2\t" << R_df_tf.getColumn(0).dot(R_df_tf.getColumn(1))
															 //<< "\n\tc1c3\t" << R_df_tf.getColumn(0).dot(R_df_tf.getColumn(2))
															 //<< "\n\tc2c3\t" << R_df_tf.getColumn(1).dot(R_df_tf.getColumn(2))
															 //<< "\n\tr1r2\t" << R_df_tf.getRow(0).dot(R_df_tf.getRow(1))
															 //<< "\n\tr1r3\t" << R_df_tf.getRow(0).dot(R_df_tf.getRow(2))
															 //<< "\n\tr2r3\t" << R_df_tf.getRow(1).dot(R_df_tf.getRow(2))
															 //<< std::endl;
				
				// getting the rotation as a quaternion
				R_df_tf.getRotation(Q_df_tf);
				
				//std::cout << "current orientation:" << "\n\tx:\t" << Q_df_tf.getX() 
													//<< "\n\ty:\t" << Q_df_tf.getY() 
													//<< "\n\tz:\t" << Q_df_tf.getZ() 
													//<< "\n\tw:\t" << Q_df_tf.getW() 
													//<< std::endl;
				
				//std::cout << "norm of quaternion:\t" << Q_df_tf.length() << std::endl;
				
				// getting the negated version of the quaternion for the check
				Q_df_tf_negated = tf::Quaternion(-Q_df_tf.getX(),-Q_df_tf.getY(),-Q_df_tf.getZ(),-Q_df_tf.getW());
				
				//std::cout << "negated orientation:" << "\n\tx:\t" << Q_df_tf_negated.getX() 
													//<< "\n\ty:\t" << Q_df_tf_negated.getY() 
													//<< "\n\tz:\t" << Q_df_tf_negated.getZ() 
													//<< "\n\tw:\t" << Q_df_tf_negated.getW() 
													//<< std::endl;
													
				//std::cout << "norm of negated quaternion:\t" << Q_df_tf_negated.length() << std::endl;
				
				//// showing the last orientation
				//std::cout << "last orientation:" << "\n\tx:\t" << Q_df_tf_last.getX() 
												 //<< "\n\ty:\t" << Q_df_tf_last.getY() 
												 //<< "\n\tz:\t" << Q_df_tf_last.getZ() 
												 //<< "\n\tw:\t" << Q_df_tf_last.getW() 
												 //<< std::endl;
													
				//std::cout << "norm of last quaternion:\t" << Q_df_tf_last.length() << std::endl;
				
				
				// checking if the quaternion has flipped
				Q_norm_current_diff = std::sqrt(std::pow(Q_df_tf.getX() - Q_df_tf_last.getX(),2.0)
											  + std::pow(Q_df_tf.getY() - Q_df_tf_last.getY(),2.0) 
											  + std::pow(Q_df_tf.getZ() - Q_df_tf_last.getZ(),2.0) 
											  + std::pow(Q_df_tf.getW() - Q_df_tf_last.getW(),2.0));
				
				//std::cout << "current difference:\t" << Q_norm_current_diff << std::endl;
				
				Q_norm_negated_diff = std::sqrt(std::pow(Q_df_tf_negated.getX() - Q_df_tf_last.getX(),2.0)
											  + std::pow(Q_df_tf_negated.getY() - Q_df_tf_last.getY(),2.0) 
											  + std::pow(Q_df_tf_negated.getZ() - Q_df_tf_last.getZ(),2.0) 
											  + std::pow(Q_df_tf_negated.getW() - Q_df_tf_last.getW(),2.0));
				
				//std::cout << "negated difference:\t" << Q_norm_negated_diff << std::endl;
				
				if (Q_norm_current_diff > Q_norm_negated_diff)
				{
					Q_df_tf = Q_df_tf_negated;
				}
				
				// updating the last
				Q_df_tf_last = Q_df_tf;
				
				// converting the tf quaternion to a geometry message quaternion
				Q_df_gm.x = Q_df_tf.getX();
				Q_df_gm.y = Q_df_tf.getY();
				Q_df_gm.z = Q_df_tf.getZ();
				Q_df_gm.w = Q_df_tf.getW();
				//std::cout << "current output quaternion" << "\n\tx:\t" << Q_df_gm.x 
														 //<< "\n\ty:\t" << Q_df_gm.y
														 //<< "\n\tz:\t" << Q_df_gm.z
														 //<< "\n\tw:\t" << Q_df_gm.w
														 //<< std::endl;
				
				// converting the tf vector3 to a point
				P_df_gm.x = T_df_tf.getX()/T_df_tf.getZ();
				P_df_gm.y = T_df_tf.getY()/T_df_tf.getZ();
				P_df_gm.z = T_df_tf.getZ()/T_df_tf.getZ();
				//std::cout << "current output position" << "\n\tx:\t" << P_df_gm.x 
													   //<< "\n\ty:\t" << P_df_gm.y
													   //<< "\n\tz:\t" << P_df_gm.z
													   //<< std::endl;
				
				// setting the desired message
				pose_df_gm.position = P_df_gm;
				pose_df_gm.orientation = Q_df_gm;
				desired_msg.pose = pose_df_gm;
				desired_msg.header.stamp = ros::Time::now();
				desired_msg.header.frame_id = "desired_frame_normalized";
				desired_msg.height = height;
				desired_msg.updating_desired = start_publishing;
				desired_msg.red_circle = p_r_gm;
				desired_msg.green_circle = p_g_gm;
				desired_msg.cyan_circle = p_c_gm;
				desired_msg.purple_circle = p_p_gm;
				omega_cd_v.x = 0;
				omega_cd_v.y = 0;
				omega_cd_v.z = omega_cd;
				desired_msg.omega_cd = omega_cd_v;
				vcd_v.x = 0;
				vcd_v.y = v_cd;
				vcd_v.z = 0;
				desired_msg.v_cd = vcd_v;
				
				std::cout << "complete message\n" << desired_msg << std::endl << std::endl;
				
			}
			else
			{
				// erasing all the pixel locations from the vector
				pixel_locations.erase(pixel_locations.begin(),pixel_locations.end());
				//std::cout << "pixel size\t" << pixel_locations.size() << std::endl;
				
				// updating the temp values
				for (int ii = 0; ii < 4; ii++)
				{
					// set to -1
					marker_temp_3.at<double>(0,0) = -1;
					marker_temp_3.at<double>(1,0) = -1;
					marker_temp_3.at<double>(2,0) = -1;
					
					// convert to pixels and add to the vector
					pixel_locations.push_back(marker_temp_3);
					//std::cout << "marker " << ii << "\n" << pixel_locations[ii] << std::endl;
				}
				desired_msg.updating_desired = start_publishing;
				
			}
			
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

	ros::Rate loop_rate(30);
    
    while(ros::ok())
    {
		// updating the desired camera pixels
		desired_camera.update_pixels();
		//for (int ii = 0; ii < desired_camera.pixel_locations.size(); ii++)
		//{
			//std::cout << "marker " << ii << "\n" << desired_camera.pixel_locations[ii] << std::endl;
		//}
		
		// publish the current desired reference
		//desired_camera_pub.publish
		
		// release 
		ros::spinOnce();
		loop_rate.sleep();
	}

    
    return 0;
}
