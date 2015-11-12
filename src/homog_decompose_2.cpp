/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 // This decompose program was implemented by Zack Bell. It implements the homography decomposition algorithm
 // descriped in the research report:
 // 
 // Malis, E and Vargas, M, "Deeper understanding of the homography decomposition
 // for vision-based control", Research Report 6303, INRIA (2007)
 //
 * The research report was again followed in converting the perspective homography to euclidean homography
 * perspective homography:
 * G = findHomography(reference points, current points)
 * finds the 4 solutions for R, T, and n 
 * decomposeHomographyMat(G,K,R,T,n);
 * relationship between output perspective homography and H
 * G = gamma*K*H*inv(K)
 * dont know gamma so need to find
 * relationship between G and H_bar
 * H_bar = inv(K)*G*K
 * getting gamma from the svd of H_bar
 * gamma = med(svd(H_bar))
 * relationship between H and H_bar
 * H = H_bar/gamma = R+(t/d_star)*transpose(n_star)
 * the alphas can then be found as:
 * alpha_ix = m_ix/((H.row(1)).dot(m_istar))
 * alpha_iy = m_iy/((H.row(2)).dot(m_istar))
 * alpha_iz = m_iz/((H.row(3)).dot(m_istar))
 //
 //M*/
#include <iostream>
#include <string>
#include <vector>

// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
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
		ros::NodeHandle nh;// node handle
		/********** Begin Topic Declarations **********/		
		ros::Subscriber complete_message_sub;// subscriber to the complete homography message containing both matrices
		ros::Publisher homog_decomp_pub;// publisher for the decomposed homography
		/********** End Topic Declarations **********/
		
		/********** Begin Point Declarations **********/		
		homog_track::HomogComplete complete_msg;// holds the current complete message
		bool reference_set;// tells the reference has been set
		homog_track::HomogMarker circles_curr, circles_ref;// current and reference marker
		std::vector<cv::Point2d> curr_points_p, ref_points_p;// current and ref points and converts them for the homography
		cv::Point2d curr_red_p, curr_green_p, curr_cyan_p, curr_purple_p;// current points
		cv::Mat pr_m, pg_m, pc_m, pp_m;// current points as pixels
		cv::Mat mr_norm, mg_norm, mc_norm, mp_norm;// current points normalized
		cv::Point2d ref_red_p, ref_green_p, ref_cyan_p, ref_purple_p;// reference points
		cv::Mat pr_ref_m, pg_ref_m, pc_ref_m, pp_ref_m; // reference points as pixels
		cv::Mat mr_ref_norm, mg_ref_norm, mc_ref_norm, mp_ref_norm; // reference points normalized
		std::vector<cv::Mat> curr_points_m, ref_points_m;// vector for the matrix of current points
		/********** End Point Declarations **********/
		
		/********** Begin decomposition Declarations **********/
		cv::Mat G;// the perspective homography matrix
		cv::Mat H_hat;// estimated homography matrix
		cv::Mat H;// scaled homography matrix
		cv::SVD svd;// svd of the perspective homography matrix
		double svd_1, svd_2, svd_3; std::vector<double> svds; // three values for the svd 
		double gamma_h;// gamma term the estimated matrix is scaled by
		cv::Mat K;// the camera matrix
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
		tf::Transform camera_wrt_reference;// transform of camera wrt reference
		geometry_msgs::Quaternion Q_cf_gm;// camera wrt reference as quaternion geometry message
		tf::Quaternion Q_cf_tf_last = tf::Quaternion(0,0,0,0), Q_cf_tf_negated = tf::Quaternion(0,0,0,0);// last camera wrt reference quaternion and negated current camera wrt reference quaternion
		double Q_norm_current_diff = 0, Q_norm_negated_diff = 0;// norms to determine which is closer to last
		geometry_msgs::Point P_cf_gm;// position of camera wrt reference as geometry message
		geometry_msgs::Pose pose_cf_gm;// pose of camera wrt reference
		std_msgs::Float64 alpha_red, alpha_green, alpha_cyan, alpha_purple;// alpha values
		homog_track::HomogDecomposed decomposed_msg;// complete decomposed message
		/********** End decomposition Declarations **********/
		
		
		// constructor for the complete set of markers
		FindTransformation()
		{
			complete_message_sub = nh.subscribe("complete_homog_set",1, &FindTransformation::complete_message_callback, this);// subscribing to the complete message
			homog_decomp_pub = nh.advertise<homog_track::HomogDecomposed>("decomposed_homography",1);// publisher for the decomposed stuff
			//// camera matrix for the ardrone
			//K = cv::Mat::zeros(3,3,CV_64F);
			//K.at<double>(0,0) = 567.79;
			//K.at<double>(0,2) = 337.35;
			//K.at<double>(1,1) = 564.52;
			//K.at<double>(1,2) = 169.54;
			//K.at<double>(2,2) = 1;
			K = cv::Mat::eye(3,3,CV_64F);
			
			std::cout << " K value:\n" << K << std::endl;
			
			// normal direction vector for the fixed reference, should be [0, 0, 1]^T
			n_ref = cv::Mat::zeros(3,1,CV_64F);
			n_ref.at<double>(2,0) = 1;
			
			// initializing decomp to false
			successful_decomp = 0;
			
			// initializer temp scalar to zero
			temp_scalar = cv::Mat::zeros(1,1,CV_64F);
			
			pr_m = cv::Mat::ones(3,1,CV_64F);
			pg_m = cv::Mat::ones(3,1,CV_64F);
			pc_m = cv::Mat::ones(3,1,CV_64F);
			pp_m = cv::Mat::ones(3,1,CV_64F);
			
			pr_ref_m = cv::Mat::ones(3,1,CV_64F);
			pg_ref_m = cv::Mat::ones(3,1,CV_64F);
			pc_ref_m = cv::Mat::ones(3,1,CV_64F);
			pp_ref_m = cv::Mat::ones(3,1,CV_64F);
			
			mr_norm = cv::Mat::ones(3,1,CV_64F);
			mg_norm = cv::Mat::ones(3,1,CV_64F);
			mc_norm = cv::Mat::ones(3,1,CV_64F);
			mp_norm = cv::Mat::ones(3,1,CV_64F);
			
			mr_ref_norm = cv::Mat::ones(3,1,CV_64F);
			mg_ref_norm = cv::Mat::ones(3,1,CV_64F);
			mc_ref_norm = cv::Mat::ones(3,1,CV_64F);
			mp_ref_norm = cv::Mat::ones(3,1,CV_64F);
		
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
				
				pr_m.at<double>(0,0) = curr_red_p.x;
				pr_m.at<double>(1,0) = curr_red_p.y;
				pg_m.at<double>(0,0) = curr_green_p.x;
				pg_m.at<double>(1,0) = curr_green_p.y;
				pc_m.at<double>(0,0) = curr_cyan_p.x;
				pc_m.at<double>(1,0) = curr_cyan_p.y;
				pp_m.at<double>(0,0) = curr_purple_p.x;
				pp_m.at<double>(1,0) = curr_purple_p.y;
				
				mr_norm = K.inv(cv::DECOMP_LU)*pr_m;
				mg_norm = K.inv(cv::DECOMP_LU)*pg_m;
				mc_norm = K.inv(cv::DECOMP_LU)*pc_m;
				mp_norm = K.inv(cv::DECOMP_LU)*pp_m;
				
				curr_points_m.push_back(mr_norm);
				curr_points_m.push_back(mg_norm);
				curr_points_m.push_back(mc_norm);
				curr_points_m.push_back(mp_norm);
				
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
				
				pr_ref_m.at<double>(0,0) = ref_red_p.x;
				pr_ref_m.at<double>(1,0) = ref_red_p.y;
				pg_ref_m.at<double>(0,0) = ref_green_p.x;
				pg_ref_m.at<double>(1,0) = ref_green_p.y;
				pc_ref_m.at<double>(0,0) = ref_cyan_p.x;
				pc_ref_m.at<double>(1,0) = ref_cyan_p.y;
				pp_ref_m.at<double>(0,0) = ref_purple_p.x;
				pp_ref_m.at<double>(1,0) = ref_purple_p.y;
				
				mr_ref_norm = K.inv(cv::DECOMP_LU)*pr_ref_m;
				mg_ref_norm = K.inv(cv::DECOMP_LU)*pg_ref_m;
				mc_ref_norm = K.inv(cv::DECOMP_LU)*pc_ref_m;
				mp_ref_norm = K.inv(cv::DECOMP_LU)*pp_ref_m;

				ref_points_m.push_back(mr_ref_norm);
				ref_points_m.push_back(mg_ref_norm);
				ref_points_m.push_back(mc_ref_norm);
				ref_points_m.push_back(mp_ref_norm);
				
				// if any of the points have a -1 will skip over the homography
				if (curr_red_p.x != -1 && curr_green_p.x != -1 && curr_cyan_p.x != -1 && curr_purple_p.x != -1)
				{	
					
					/********** following the process outlined in the reference **********/			
					// finding the perspective homography
					//G = cv::findHomography(curr_points_p,ref_points_p,0);
					G = cv::findHomography(ref_points_p,curr_points_p,0);
					
					// finding the approximate of the euclidean homography
					H_hat = (K.inv(cv::DECOMP_LU)*G)*K;
					
					
					std::cout << "testing" << std::endl; 
					
					// getting the svd of the approximate
					svd = cv::SVD(H_hat,cv::SVD::NO_UV);
					svd_1 = svd.w.at<double>(0,0);
					svd_2 = svd.w.at<double>(1,0);
					svd_3 = svd.w.at<double>(2,0);
					
					std::cout << "svd w:\n" << svd.w << std::endl;
					std::cout << "svd1:\t" << svd_1 << std::endl;
					std::cout << "svd2:\t" << svd_2 << std::endl;
					std::cout << "svd3:\t" << svd_3 << std::endl;
					
					svds.push_back(svd_1);
					svds.push_back(svd_2);
					svds.push_back(svd_3);
					std::sort(svds.begin(),svds.end());
					for (double ii : svds)
					{
						std::cout << ii << "\t";
					}
					std::cout << std::endl;
					
					gamma_h = *(svds.begin()+svds.size()/2);
					
					std::cout << "gamma_h:\t" << gamma_h << std::endl;
					
					svds.erase(svds.begin(),svds.end());
					
					std::cout << "G:\n" << G << std::endl;
					
					H = (1.0/gamma_h)*H_hat;
					
					std::cout << "H:\n" << H << std::endl;
					std::cout << "H_hat:\n" << H_hat << std::endl;
					
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
						
						//// finding the alphas
						//std::cout << "red alpha x:\t" << pr_m.at<double>(0,0)/((G.row(0)).dot(pr_ref_m.t())) << std::endl;
						//std::cout << "red alpha y:\t" << pr_m.at<double>(1,0)/((G.row(1)).dot(pr_ref_m.t())) << std::endl;
						//std::cout << "red alpha z:\t" << pr_m.at<double>(2,0)/((G.row(2)).dot(pr_ref_m.t())) << std::endl;
						
						//std::cout << "green alpha x:\t" << pg_m.at<double>(0,0)/((G.row(0)).dot(pg_ref_m.t())) << std::endl;
						//std::cout << "green alpha y:\t" << pg_m.at<double>(1,0)/((G.row(1)).dot(pg_ref_m.t())) << std::endl;
						//std::cout << "green alpha z:\t" << pg_m.at<double>(2,0)/((G.row(2)).dot(pg_ref_m.t())) << std::endl;
						
						//std::cout << "cyan alpha x:\t" << pc_m.at<double>(0,0)/((G.row(0)).dot(pc_ref_m.t())) << std::endl;
						//std::cout << "cyan alpha y:\t" << pc_m.at<double>(1,0)/((G.row(1)).dot(pc_ref_m.t())) << std::endl;
						//std::cout << "cyan alpha z:\t" << pc_m.at<double>(2,0)/((G.row(2)).dot(pc_ref_m.t())) << std::endl;
						
						//std::cout << "purple alpha x:\t" << pp_m.at<double>(0,0)/((G.row(0)).dot(pp_ref_m.t())) << std::endl;
						//std::cout << "purple alpha y:\t" << pp_m.at<double>(1,0)/((G.row(1)).dot(pp_ref_m.t())) << std::endl;
						//std::cout << "purple alpha z:\t" << pp_m.at<double>(2,0)/((G.row(2)).dot(pp_ref_m.t())) << std::endl;
						
						// finding the alphas
						std::cout << "red alpha x:\t" << mr_norm.at<double>(0,0)/((H.row(0)).dot(mr_ref_norm.t())) << std::endl;
						std::cout << "red alpha y:\t" << mr_norm.at<double>(1,0)/((H.row(1)).dot(mr_ref_norm.t())) << std::endl;
						std::cout << "red alpha z:\t" << mr_norm.at<double>(2,0)/((H.row(2)).dot(mr_ref_norm.t())) << std::endl;
						
						std::cout << "green alpha x:\t" << mg_norm.at<double>(0,0)/((H.row(0)).dot(mg_ref_norm.t())) << std::endl;
						std::cout << "green alpha y:\t" << mg_norm.at<double>(1,0)/((H.row(1)).dot(mg_ref_norm.t())) << std::endl;
						std::cout << "green alpha z:\t" << mg_norm.at<double>(2,0)/((H.row(2)).dot(mg_ref_norm.t())) << std::endl;
						
						std::cout << "cyan alpha x:\t" << mc_norm.at<double>(0,0)/((H.row(0)).dot(mc_ref_norm.t())) << std::endl;
						std::cout << "cyan alpha y:\t" << mc_norm.at<double>(1,0)/((H.row(1)).dot(mc_ref_norm.t())) << std::endl;
						std::cout << "cyan alpha z:\t" << mc_norm.at<double>(2,0)/((H.row(2)).dot(mc_ref_norm.t())) << std::endl;
						
						std::cout << "purple alpha x:\t" << mp_norm.at<double>(0,0)/((H.row(0)).dot(mp_ref_norm.t())) << std::endl;
						std::cout << "purple alpha y:\t" << mp_norm.at<double>(1,0)/((H.row(1)).dot(mp_ref_norm.t())) << std::endl;
						std::cout << "purple alpha z:\t" << mp_norm.at<double>(2,0)/((H.row(2)).dot(mp_ref_norm.t())) << std::endl;
						
						alpha_red.data = mr_norm.at<double>(2,0)/((H.row(2)).dot(mr_ref_norm.t()));
						alpha_green.data = mg_norm.at<double>(2,0)/((H.row(2)).dot(mg_ref_norm.t()));
						alpha_cyan.data = mc_norm.at<double>(2,0)/((H.row(2)).dot(mc_ref_norm.t()));
						alpha_purple.data = mp_norm.at<double>(2,0)/((H.row(2)).dot(mp_ref_norm.t()));
						
						// finding the solutions that give the positive results
						for (int ii = 0; ii < successful_decomp; ii++)
						{
							
							//std::cout << "solution set number " << ii << std::endl;
							
							// performing the operation transpose(m)*R*n to check if greater than 0 later
							// order operating on is red green cyan purple
							for (int jj = 0; jj < 4; jj++)
							{
								
								//std::cout << " T size: " << T[ii].size() << std::endl;
								//std::cout << " T type: " << T[ii].type() << std::endl;
								//std::cout << " T value:\n" << T[ii] << std::endl;
								
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
								//std::cout << " n value: " << n[ii] << std::endl;
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
						
						//std::cout << " end check for visibility" << std::endl << std::endl;
						
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
							
							// take transpose for the quaternion
							R_cf_tf = R_fc_tf.transpose();
							
							std::cout << "Rotation of F wrt Fstar:\n" << R_fc.t() << std::endl;
							
							// converting the translation to a vector 3
							T_fc_tf.setX(T_fc.at<double>(0,0));
							T_fc_tf.setY(T_fc.at<double>(0,1));
							T_fc_tf.setZ(T_fc.at<double>(0,2));
							
							// changeing to showing camera wrt to reference
							T_cf_tf = -1*(R_cf_tf*T_fc_tf);
							
							std::cout << "Position of F wrt Fstar:\n" << -1*(R_fc.t()*T_fc) << std::endl;
							
							// getting the rotation as a quaternion
							R_cf_tf.getRotation(Q_cf_tf);

							// getting the negated version of the quaternion for the check
							Q_cf_tf_negated = tf::Quaternion(-Q_cf_tf.getX(),-Q_cf_tf.getY(),-Q_cf_tf.getZ(),-Q_cf_tf.getW());
							
							// checking if the quaternion has flipped
							Q_norm_current_diff = std::sqrt(std::pow(Q_cf_tf.getX() - Q_cf_tf_last.getX(),2.0)
														  + std::pow(Q_cf_tf.getY() - Q_cf_tf_last.getY(),2.0) 
														  + std::pow(Q_cf_tf.getZ() - Q_cf_tf_last.getZ(),2.0) 
														  + std::pow(Q_cf_tf.getW() - Q_cf_tf_last.getW(),2.0));
							
							//std::cout << "current difference:\t" << Q_norm_current_diff << std::endl;
							
							Q_norm_negated_diff = std::sqrt(std::pow(Q_cf_tf_negated.getX() - Q_cf_tf_last.getX(),2.0)
														  + std::pow(Q_cf_tf_negated.getY() - Q_cf_tf_last.getY(),2.0) 
														  + std::pow(Q_cf_tf_negated.getZ() - Q_cf_tf_last.getZ(),2.0) 
														  + std::pow(Q_cf_tf_negated.getW() - Q_cf_tf_last.getW(),2.0));
							
							//std::cout << "negated difference:\t" << Q_norm_negated_diff << std::endl;
							
							if (Q_norm_current_diff > Q_norm_negated_diff)
							{
								Q_cf_tf = Q_cf_tf_negated;
							}
							
							// updating the last
							Q_cf_tf_last = Q_cf_tf;
							
							// converting the tf quaternion to a geometry message quaternion
							Q_cf_gm.x = Q_cf_tf.getX();
							Q_cf_gm.y = Q_cf_tf.getY();
							Q_cf_gm.z = Q_cf_tf.getZ();
							Q_cf_gm.w = Q_cf_tf.getW();
							
							// converting the tf vector3 to a point
							P_cf_gm.x = T_cf_tf.getX();
							P_cf_gm.y = T_cf_tf.getY();
							P_cf_gm.z = T_cf_tf.getZ();
							
							// setting the transform with the values
							camera_wrt_reference.setOrigin(T_cf_tf);
							camera_wrt_reference.setRotation(Q_cf_tf);
							
							// setting the decomposed message
							pose_cf_gm.position = P_cf_gm;
							pose_cf_gm.orientation = Q_cf_gm;
							decomposed_msg.pose = pose_cf_gm;
							decomposed_msg.header.stamp = msg.current_points.header.stamp;
							decomposed_msg.header.frame_id = "current_frame_normalized";
							decomposed_msg.alpha_red = alpha_red;
							decomposed_msg.alpha_green = alpha_green;
							decomposed_msg.alpha_cyan = alpha_cyan;
							decomposed_msg.alpha_purple = alpha_purple;
							homog_decomp_pub.publish(decomposed_msg);
							
							std::cout << "complete message\n" << decomposed_msg << std::endl << std::endl;
							
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
