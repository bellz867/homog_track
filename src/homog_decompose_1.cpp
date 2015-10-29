#include <iostream>
#include <string>
#include <vector>

// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <homog_track/HomogMarker.h>
#include <homog_track/HomogComplete.h>
#include <homog_track/HomogDecomposed.h>
#include <visualization_msgs/Marker.h>

// class for subscribing to complete homography message, finds the homography, then finds the decomposition
// returns the two positive solutions
class FindTransformation
{
	public:
		// node handle
		ros::NodeHandle nh;
		
		/********** Begin Topic Declarations **********/		
		// subscriber to the complete homography message containing both matrices
		ros::Subscriber complete_message_sub;
		
		// publisher for the decomposed homography
		ros::Publisher homog_decomp_pub;
		
		// broadcaster for the transform
		tf::TransformBroadcaster tf_broad;
		
		// marker publisher
		ros::Publisher marker_pub;
		/********** End Topic Declarations **********/
		
		/********** Begin Point Declarations **********/		
		// holds the current complete message
		homog_track::HomogComplete complete_msg;
		
		// tells the reference has been set
		bool reference_set;
		
		// current marker
		homog_track::HomogMarker circles_curr;
		
		// reference marker
		homog_track::HomogMarker circles_ref;
		
		// takes the current points and converts them for the homography
		std::vector<cv::Point2f> curr_points_p;
		
		// takes the reference points and converts them for the homography
		std::vector<cv::Point2f> ref_points_p;
		
		// the current points
		cv::Point2f curr_red_p;
		cv::Point2f curr_green_p;
		cv::Point2f curr_cyan_p;
		cv::Point2f curr_purple_p;
		
		// holds the reference points
		cv::Point2f ref_red_p;
		cv::Point2f ref_green_p;
		cv::Point2f ref_cyan_p;
		cv::Point2f ref_purple_p;
		
		// matrix for the current points 3x1
		cv::Mat curr_red_m;
		cv::Mat curr_green_m;
		cv::Mat curr_cyan_m;
		cv::Mat curr_purple_m;
		
		// matrix for the reference points 3x1
		cv::Mat ref_red_m;
		cv::Mat ref_green_m;
		cv::Mat ref_cyan_m;
		cv::Mat ref_purple_m;
		
		// vector for the matrix of current points
		std::vector<cv::Mat> curr_points_m;
		
		// vector for the matrix of reference points
		std::vector<cv::Mat> ref_points_m;
		
		/********** End Point Declarations **********/
		
		/********** Begin decomposition Declarations **********/
		// the perspective homography matrix
		cv::Mat G;
		
		// the camera matrix
		cv::Mat K;
		
		// successful decomposition
		int successful_decomp;
		
		// the rotation matrices
		std::vector<cv::Mat> R;
		
		// the translation vectors
		std::vector<cv::Mat> T;
		
		// the normal vectors
		std::vector<cv::Mat> n;
		
		// temp holder for scalar checking if a converted point is positive
		cv::Mat temp_scalar;
		
		// array to hold the values of the all the positive definite check values
		std::vector<double> scalar_value_check;
		
		// temporary solution iterators can read and write
		std::vector<double>::iterator temp_solution_start;
		std::vector<double>::iterator temp_solution_end;
		std::vector<double> temp_solution;
		bool all_positive;
		int current_temp_index;
		
		// first solution variables
		std::vector<double>::iterator first_solution_start;
		std::vector<double>::iterator first_solution_end;
		std::vector<double> first_solution;
		bool first_solution_found;
		cv::Mat first_R;
		cv::Mat first_T;
		cv::Mat first_n;
		
		// second solution variables. there may not be a second solution
		std::vector<double>::iterator second_solution_start;
		std::vector<double>::iterator second_solution_end;
		std::vector<double> second_solution;
		bool second_solution_found;
		cv::Mat second_R;
		cv::Mat second_T;
		cv::Mat second_n;
		
		// conversion variables for the most normal solution to publish as a pose message for current to fixed
		bool fc_found;
		cv::Mat n_ref;
		cv::Mat R_fc;
		cv::Mat T_fc;
		cv::Mat n_fc;
		tf::Matrix3x3 R_fc_tf;
		tf::Quaternion Q_fc_tf;
		tf::Vector3 T_fc_tf;
		tf::Transform fc_tf;
		geometry_msgs::Quaternion Q_fc_gm;
		tf::Quaternion Q_fc_tf_last = tf::Quaternion(0,0,0,0);
		tf::Quaternion Q_fc_tf_negated = tf::Quaternion(0,0,0,0);
		double Q_norm_current_diff = 0;
		double Q_norm_negated_diff = 0;
		geometry_msgs::Point P_fc_gm;
		geometry_msgs::Pose pose_fc_gm;
		
		// alpha values
		std_msgs::Float64 alpha_red;
		std_msgs::Float64 alpha_green;
		std_msgs::Float64 alpha_cyan;
		std_msgs::Float64 alpha_purple;
		
		// complete decomposed message of 
		homog_track::HomogDecomposed decomposed_msg;
		
		/********** End decomposition Declarations **********/
		
		/********** Marker Stuff **********/
		// shape of the marker
		uint32_t shape;
		
		// marker itself
		visualization_msgs::Marker marker;
		
		
		// constructor for the complete set of markers
		FindTransformation()
		{
			// subscribing to the complete message
			complete_message_sub = nh.subscribe("complete_homog_set",1, &FindTransformation::complete_message_callback, this);
			
			// publisher for the decomposed stuff
			homog_decomp_pub = nh.advertise<homog_track::HomogDecomposed>("decomposed_homography",1);
			
			// publisher for the pose marker
			marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
			
			// marker
			shape = visualization_msgs::Marker::CUBE;
			marker.header.frame_id = "homog_decomp_marker_cube";
			marker.id = 0;
			marker.type = shape;
			marker.action = visualization_msgs::Marker::ADD;
			marker.scale.x = 0.5;
			marker.scale.y = 0.5;
			marker.scale.z = 0.1;
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0;
			marker.lifetime = ros::Duration();
			marker.ns = "marker_ns";
			
			// camera matrix for the ardrone
			K = cv::Mat::zeros(3,3,CV_64F);
			K.at<double>(0,0) = 567.79;
			K.at<double>(0,2) = 337.35;
			K.at<double>(1,1) = 564.52;
			K.at<double>(1,2) = 169.54;
			K.at<double>(2,2) = 1;
			
			std::cout << " K value " << K << std::endl;
			
			// normal direction vector for the fixed reference, should be [0, 0, 1]^T
			n_ref = cv::Mat::zeros(3,1,CV_64F);
			n_ref.at<double>(2,0) = 1;
			
			// initializing the current and ref matricies
			curr_red_m = cv::Mat::ones(3,1,CV_64F);
			curr_green_m = cv::Mat::ones(3,1,CV_64F);
			curr_cyan_m = cv::Mat::ones(3,1,CV_64F);
			curr_purple_m = cv::Mat::ones(3,1,CV_64F);
			
			ref_red_m = cv::Mat::ones(3,1,CV_64F);
			ref_green_m = cv::Mat::ones(3,1,CV_64F);
			ref_cyan_m = cv::Mat::ones(3,1,CV_64F);
			ref_purple_m = cv::Mat::ones(3,1,CV_64F);
			
			// initializing decomp to false
			successful_decomp = 0;
			
			// initializer temp scalar to zero
			temp_scalar = cv::Mat::zeros(1,1,CV_64F);
		
		}
		
		// callback for the complete message
		void complete_message_callback(const homog_track::HomogComplete& msg)
		{
			/********** Begin splitting up the incoming message *********/
			// getting boolean indicating the reference has been set
			reference_set = msg.reference_set;

			// if the reference is set then will break out the points
			if (reference_set)
			{
				// initializer temp scalar to zero
				temp_scalar = cv::Mat::zeros(1,1,CV_64F);
			
				// getting the current marker points
				circles_curr = msg.current_points;
				
				// getting the refernce marker points
				circles_ref = msg.reference_points;
				
				// setting the current points to the point vector
				curr_red_p.x = circles_curr.red_circle.x;
				curr_green_p.x = circles_curr.green_circle.x;
				curr_cyan_p.x = circles_curr.cyan_circle.x;
				curr_purple_p.x = circles_curr.purple_circle.x;
				curr_red_p.y = circles_curr.red_circle.y;
				curr_green_p.y = circles_curr.green_circle.y;
				curr_cyan_p.y = circles_curr.cyan_circle.y;
				curr_purple_p.y = circles_curr.purple_circle.y;
				curr_points_p.push_back(curr_red_p);
				curr_points_p.push_back(curr_green_p);
				curr_points_p.push_back(curr_cyan_p);
				curr_points_p.push_back(curr_purple_p);
				
				
				
				// converting the points to be the projective coordinates
				for (int ii = 0; ii < curr_points_m.size(); ii++)
				{
					curr_points_m[ii] = K.inv(cv::DECOMP_LU)*curr_points_m[ii];
					std::cout << "currpoints at " << ii << " is: " << curr_points_m[ii] << std::endl;
					
				}
				
				// setting the reference points to the point vector
				ref_red_p.x = circles_ref.red_circle.x;
				ref_green_p.x = circles_ref.green_circle.x;
				ref_cyan_p.x = circles_ref.cyan_circle.x;
				ref_purple_p.x = circles_ref.purple_circle.x;
				ref_red_p.y = circles_ref.red_circle.y;
				ref_green_p.y = circles_ref.green_circle.y;
				ref_cyan_p.y = circles_ref.cyan_circle.y;
				ref_purple_p.y = circles_ref.purple_circle.y;
				ref_points_p.push_back(ref_red_p);
				ref_points_p.push_back(ref_green_p);
				ref_points_p.push_back(ref_cyan_p);
				ref_points_p.push_back(ref_purple_p);
				
				
				// setting the reference points to the matrix vector, dont need to do the last one because its already 1
				ref_red_m.at<double>(0,0) = ref_red_p.x;
				ref_red_m.at<double>(1,0) = ref_red_p.y;
				ref_green_m.at<double>(0,0) = ref_green_p.x;
				ref_green_m.at<double>(1,0) = ref_green_p.y;
				ref_cyan_m.at<double>(0,0) = ref_cyan_p.x;
				ref_cyan_m.at<double>(1,0) = ref_cyan_p.y;
				ref_purple_m.at<double>(0,0) = ref_purple_p.x;
				ref_purple_m.at<double>(1,0) = ref_purple_p.y;
				ref_points_m.push_back(ref_red_m);
				ref_points_m.push_back(ref_green_m);
				ref_points_m.push_back(ref_cyan_m);
				ref_points_m.push_back(ref_purple_m);
				
				
				// converting the points to be the projective coordinates
				for (int ii = 0; ii < ref_points_m.size(); ii++)
				{
					ref_points_m[ii] = K.inv(cv::DECOMP_LU)*ref_points_m[ii];
					//std::cout << "refpoints at " << ii << " is: " << ref_points_m[ii] << std::endl;
				}
				
				// if any of the points have a -1 will skip over the homography
				if (curr_red_p.x != -1 && curr_green_p.x != -1 && curr_cyan_p.x != -1 && curr_purple_p.x != -1)
				{
					//std::cout << "hi" << std::endl;
					
					// finding the perspective homography
					G = cv::findHomography(curr_points_p,ref_points_p,0);
					//G = cv::findHomography(ref_points_p,ref_points_p,0);
					
					std::cout << "G: " << G << std::endl;
					
					// decomposing the homography into the four solutions
					// G and K are 3x3
					// R is 3x3
					// 3x1
					// 3x1
					// successful_decomp is the number of solutions found
					successful_decomp = cv::decomposeHomographyMat(G,K,R,T,n);
					
					std::cout << "successful_decomp: " << successful_decomp << std::endl;
					
					
					// if the decomp is successful will find the best matching
					if (successful_decomp > 0)
					{
						
						std::cout << std::endl << std::endl << " begin check for visibility" << std::endl;
						
						// finding the alphas
						alpha_red.data = 1/(G.at<double>(2,0)*ref_red_p.x + G.at<double>(2,1)*ref_red_p.y + 1);
						alpha_green.data = 1/(G.at<double>(2,0)*ref_green_p.x + G.at<double>(2,1)*ref_green_p.y + 1);
						alpha_cyan.data = 1/(G.at<double>(2,0)*ref_cyan_p.x + G.at<double>(2,1)*ref_cyan_p.y + 1);
						alpha_purple.data = 1/(G.at<double>(2,0)*ref_purple_p.x + G.at<double>(2,1)*ref_purple_p.y + 1);
						
						// finding the solutions that give the positive results
						for (int ii = 0; ii < successful_decomp; ii++)
						{
							
							std::cout << "solution set number " << ii << std::endl;
							
							// performing the operation transpose(m)*R*n to check if greater than 0 later
							// order operating on is red green cyan purple
							for (int jj = 0; jj < 4; jj++)
							{
								
								//std::cout << " T size: " << T[ii].size() << std::endl;
								//std::cout << " T type: " << T[ii].type() << std::endl;
								std::cout << " T value: " << T[ii] << std::endl;
								
								//std::cout << " temp scalar 1 " << std::endl;
								//std::cout << " temp scalar size: " << temp_scalar.size() << std::endl;
								//std::cout << " temp scalar type: " << temp_scalar.type() << std::endl;
								//std::cout << " temp scalar value " << temp_scalar <<std::endl;
								temp_scalar = curr_points_m[jj].t();
								
								//std::cout << " temp scalar 2 " << std::endl;
								//std::cout << " temp scalar size: " << temp_scalar.size() << std::endl;
								//std::cout << " temp scalar type: " << temp_scalar.type() << std::endl;
								//std::cout << " temp scalar value " << temp_scalar <<std::endl;
								
								//std::cout << " R size: " << R[ii].size() << std::endl;
								//std::cout << " R type: " << R[ii].type() << std::endl;
								//std::cout << " R value: " << R[ii] << std::endl;
								temp_scalar = temp_scalar*R[ii];
								
								//std::cout << " temp scalar 3 " << std::endl;
								//std::cout << " temp scalar size: " << temp_scalar.size() << std::endl;
								//std::cout << " temp scalar type: " << temp_scalar.type() << std::endl;
								//std::cout << " temp scalar value " << temp_scalar <<std::endl;
								
								//std::cout << " n size: " << n[ii].size() << std::endl;
								//std::cout << " n type: " << n[ii].type() << std::endl;
								std::cout << " n value: " << n[ii] << std::endl;
								temp_scalar = temp_scalar*n[ii];
								
								//std::cout << " temp scalar size: " << temp_scalar.size() << std::endl;
								//std::cout << " temp scalar type: " << temp_scalar.type() << std::endl;
								//std::cout << " temp scalar value " << temp_scalar <<std::endl;
								//std::cout << " temp scalar value at 0,0" << temp_scalar.at<double>(0,0) << std::endl;
								
								scalar_value_check.push_back(temp_scalar.at<double>(0,0));
								
								////std::cout << " scalar value check size: " << scalar_value_check.size() << std::endl;
								//std::cout << " \tthe value for the " << jj << " visibility check is: " << scalar_value_check[4*ii+jj] << std::endl;
								
							}
						}
						
						std::cout << " end check for visibility" << std::endl << std::endl;
						
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
					
					
						// displaying the first solution if it was found
						if (first_solution_found)
						{
							std::cout << std::endl << "first R: " << first_R << std::endl;
							std::cout << "first T: " << first_T << std::endl;
							std::cout << "first n: " << first_n << std::endl;
							for (double ii : first_solution)
							{
								std::cout << ii << " ";
							}
							std::cout << std::endl;
							
						}
						
						// displaying the second solution if it was found
						if (second_solution_found)
						{
							std::cout << std::endl << "second R: " << second_R << std::endl;
							std::cout << "second T: " << second_T << std::endl;
							std::cout << "second n: " << second_n << std::endl;
							for (double ii : second_solution)
							{
								std::cout << ii << " ";
							}
							std::cout << std::endl;
						}
						
						// because the reference is set to the exact value when when n should have only a z componenet, the correct
						// choice should be the one closest to n_ref = [0,0,1]^T which will be the one with the greatest dot product with n_ref
						if (first_solution_found && second_solution_found)
						{
							if (first_n.dot(n_ref) >= second_n.dot(n_ref))
							{
								R_fc = first_R;
								T_fc = first_T;
							}
							else
							{
								R_fc = second_R;
								T_fc = second_T;
							}
							fc_found = true;
						}
						else if(first_solution_found)
						{
							R_fc = first_R;
							T_fc = first_T;
							fc_found = true;
						}
						
						//if a solution was found will publish
						// need to convert to pose message so use
						if (fc_found)
						{
							// converting the rotation from a cv matrix to quaternion, first need it as a matrix3x3
							R_fc_tf[0][0] = R_fc.at<double>(0,0);
							R_fc_tf[0][1] = R_fc.at<double>(0,1);
							R_fc_tf[0][2] = R_fc.at<double>(0,2);
							R_fc_tf[1][0] = R_fc.at<double>(1,0);
							R_fc_tf[1][1] = R_fc.at<double>(1,1);
							R_fc_tf[1][2] = R_fc.at<double>(1,2);
							R_fc_tf[2][0] = R_fc.at<double>(2,0);
							R_fc_tf[2][1] = R_fc.at<double>(2,1);
							R_fc_tf[2][2] = R_fc.at<double>(2,2);
							std::cout << "Final R:\n" << R_fc << std::endl;
							
							// converting the translation to a vector 3
							T_fc_tf.setX(T_fc.at<double>(0,0));
							T_fc_tf.setY(T_fc.at<double>(0,1));
							T_fc_tf.setZ(T_fc.at<double>(0,2));
							std::cout << "Final T :\n" << T_fc << std::endl;
							
							// getting the rotation as a quaternion
							R_fc_tf.getRotation(Q_fc_tf);
							
							std::cout << "current orientation:" << "\n\tx:\t" << Q_fc_tf.getX() 
																<< "\n\ty:\t" << Q_fc_tf.getY() 
																<< "\n\tz:\t" << Q_fc_tf.getZ() 
																<< "\n\tw:\t" << Q_fc_tf.getW() 
																<< std::endl;
				
							std::cout << "norm of quaternion:\t" << Q_fc_tf.length() << std::endl;
							
							// getting the negated version of the quaternion for the check
							Q_fc_tf_negated = tf::Quaternion(-Q_fc_tf.getX(),-Q_fc_tf.getY(),-Q_fc_tf.getZ(),-Q_fc_tf.getW());
							
							std::cout << "negated orientation:" << "\n\tx:\t" << Q_fc_tf_negated.getX() 
																<< "\n\ty:\t" << Q_fc_tf_negated.getY() 
																<< "\n\tz:\t" << Q_fc_tf_negated.getZ() 
																<< "\n\tw:\t" << Q_fc_tf_negated.getW() 
																<< std::endl;
																
							std::cout << "norm of negated quaternion:\t" << Q_fc_tf_negated.length() << std::endl;
							
							// showing the last orientation
							std::cout << "last orientation:" << "\n\tx:\t" << Q_fc_tf_last.getX() 
															 << "\n\ty:\t" << Q_fc_tf_last.getY() 
															 << "\n\tz:\t" << Q_fc_tf_last.getZ() 
															 << "\n\tw:\t" << Q_fc_tf_last.getW() 
															 << std::endl;
																
							std::cout << "norm of last quaternion:\t" << Q_fc_tf_last.length() << std::endl;
							
							// checking if the quaternion has flipped
							Q_norm_current_diff = std::sqrt(std::pow(Q_fc_tf.getX() - Q_fc_tf_last.getX(),2.0)
														  + std::pow(Q_fc_tf.getY() - Q_fc_tf_last.getY(),2.0) 
														  + std::pow(Q_fc_tf.getZ() - Q_fc_tf_last.getZ(),2.0) 
														  + std::pow(Q_fc_tf.getW() - Q_fc_tf_last.getW(),2.0));
							
							std::cout << "current difference:\t" << Q_norm_current_diff << std::endl;
							
							Q_norm_negated_diff = std::sqrt(std::pow(Q_fc_tf_negated.getX() - Q_fc_tf_last.getX(),2.0)
														  + std::pow(Q_fc_tf_negated.getY() - Q_fc_tf_last.getY(),2.0) 
														  + std::pow(Q_fc_tf_negated.getZ() - Q_fc_tf_last.getZ(),2.0) 
														  + std::pow(Q_fc_tf_negated.getW() - Q_fc_tf_last.getW(),2.0));
							
							std::cout << "negated difference:\t" << Q_norm_negated_diff << std::endl;
							
							if (Q_norm_current_diff > Q_norm_negated_diff)
							{
								Q_fc_tf = Q_fc_tf_negated;
							}
							
							// updating the last
							Q_fc_tf_last = Q_fc_tf;
							
							// converting the tf quaternion to a geometry message quaternion
							Q_fc_gm.x = Q_fc_tf.getX();
							Q_fc_gm.y = Q_fc_tf.getY();
							Q_fc_gm.z = Q_fc_tf.getZ();
							Q_fc_gm.w = Q_fc_tf.getW();
							
							// converting the tf vector3 to a point
							P_fc_gm.x = T_fc_tf.getX();
							P_fc_gm.y = T_fc_tf.getY();
							P_fc_gm.z = T_fc_tf.getZ();
							
							// setting the transform with the values
							fc_tf.setOrigin(T_fc_tf);
							fc_tf.setRotation(Q_fc_tf);
							tf_broad.sendTransform(tf::StampedTransform(fc_tf, msg.header.stamp,"f_star","f_current"));
							
							// setting the decomposed message
							pose_fc_gm.position = P_fc_gm;
							pose_fc_gm.orientation = Q_fc_gm;
							decomposed_msg.pose = pose_fc_gm;
							decomposed_msg.header.stamp = msg.header.stamp;
							decomposed_msg.header.frame_id = "current_frame_normalized";
							decomposed_msg.alpha_red = alpha_red;
							decomposed_msg.alpha_green = alpha_green;
							decomposed_msg.alpha_cyan = alpha_cyan;
							decomposed_msg.alpha_purple = alpha_purple;
							homog_decomp_pub.publish(decomposed_msg);
							
							std::cout << "complete message\n" << decomposed_msg << std::endl << std::endl;
							
							// publish the marker
							marker.pose = pose_fc_gm;
							marker_pub.publish(marker);
							
						}
					}
				}

				// erasing all the temporary points
				if (first_solution_found || second_solution_found)
				{
					// erasing all the point vectors and matrix vectors
					curr_points_p.erase(curr_points_p.begin(),curr_points_p.end());
					ref_points_p.erase(ref_points_p.begin(),ref_points_p.end());
					curr_points_m.erase(curr_points_m.begin(),curr_points_m.end());
					ref_points_m.erase(ref_points_m.begin(),ref_points_m.end());
				}
			}
			/********** End splitting up the incoming message *********/
			
		}
		
		
};

// main
int main(int argc, char** argv)
{   
	// initialize node
	ros::init(argc,argv,"decomp_node");
	
	// initialize the transformation    
	FindTransformation find_transformation;

    // release and sleep until next time image is available
    ros::spin();
    
    return 0;
}
