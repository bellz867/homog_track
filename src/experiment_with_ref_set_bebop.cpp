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
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>

// function to push values onto a deque and if it is over some length will remove the first and then push it
template <typename sample_set_type> void shift_sample_set(std::deque<sample_set_type> &sample_set, sample_set_type &new_sample)
{
	sample_set.pop_front();
	sample_set.push_back(new_sample);
}

// prototype for the filtering function
void filter_image_for_color(cv::Mat&, cv::Mat&, int*, int*,cv::Mat, cv::Mat);

class Circle
{       
    public:
        float radius, x, y;// values for the circle
        
        /********** initialize values contstructor **********/
		Circle(float self_radius, float self_x, float self_y)
		{
			radius = self_radius; x = self_x; y = self_y;
		}

		/********** default constructor **********/
		Circle()
		{
			radius = 0; x = 0; y = 0;
		}

		/********** set callback **********/
		void setCircle(float self_radius, float self_x, float self_y)
		{
			radius = self_radius; x = self_x; y = self_y;
		}
		
		/********** print a circle **********/
		void printCircle()
		{
			std::cout << "x: " << x << " y: " << y << " r: " << radius << std::endl;
		}
};

/********** Image processing class **********/
class ImageProcessing
{
	public:
		// node for the image converter
		ros::NodeHandle nh;
		
		/********** Topic Declarations **********/
		// image transport publisher and subscriber
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;
		ros::Publisher pixel_pub;// circles publishers
		
		/********** Point Declarations **********/
		geometry_msgs::Point red_circle_p, green_circle_p, cyan_circle_p, purple_circle_p;// the four points for the circles
		geometry_msgs::Point red_circle_p_curr, green_circle_p_curr, cyan_circle_p_curr, purple_circle_p_curr;// the four points for the circles currently
		geometry_msgs::Point red_circle_p_last, green_circle_p_last, cyan_circle_p_last, purple_circle_p_last;// the four points for the circles last time
		float low_pass_gain = 1;// tuning gain for the low pass
		bool first_run = false;// boolean to tell if this is the first run
		homog_track::ImageProcessingMsg pixels_out;//message
		bool camera_updated = false;
		
		/********** threshold declarations and initializations **********/
		// order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
		int r_thresh[6] = {155, 180, 25, 255, 80, 255};// values for the red color threshold values: hue, staturation, and value
		int g_thresh[6] = {65, 90, 40, 255, 40, 255};// values for the green color threshold values: hue, staturation, and value
		int c_thresh[6] = {95, 105, 60, 255, 80, 255};// values for the cyan color threshold values: hue, staturation, and value
		int p_thresh[6] = {110, 135, 30, 255, 50, 255};// values for the violet color threshold values: hue, staturation, and value
		std::vector<int*> thresh_current{ r_thresh, g_thresh, c_thresh, p_thresh };// putting the start arrays into a vector
		int thresh_max[6] = {180, 180, 255, 255, 255, 255};// max values for the primary colorthreshold value: hue, saturation, and value
		
		/********** Image Matrix Declarations **********/    
		cv_bridge::CvImagePtr cv_ptr;// image pointer for the converted images
		cv::Mat distorted_frame, frame, blur_frame, hsv_frame, r_binary_frame, g_binary_frame, c_binary_frame, p_binary_frame;//frames 
		std::vector<cv::Mat> binary_frames{r_binary_frame, g_binary_frame, c_binary_frame, p_binary_frame};// putting the binary frames into a vector
		cv::Mat A;// camera matrix
		std::vector<double> dC;// distortion coefficients
		
		/********** Kernel Declarations **********/
		int blur_kernel_size = 11;// kernel size for blurring
		int erode_kernel_size = 5;// kernel size for erode
		int dilate_kernel_size = 3;// kernel size for dilate
		cv::Mat erode_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
														   cv::Size(  erode_kernel_size, erode_kernel_size ),
														   cv::Point( -1, -1) );// getting the structuring element for the erode
		cv::Mat dilate_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
															cv::Size( dilate_kernel_size, dilate_kernel_size ),
															cv::Point( -1, -1) );// getting the structuring element for the dilate 
															
		/********** Decarations for the contours and the shapes **********/
		std::vector< std::vector<cv::Point>> r_contours, g_contours, c_contours, p_contours; // contours
		std::vector< std::vector< std::vector<cv::Point> > > contours{r_contours, g_contours, c_contours, p_contours};// putting the contours into a vector
		std::vector<std::string> contour_names{ "Red Contour", "Green Contour", "Cyan Contour", "Purple Contour" };
		Circle *temp_circle = new Circle; // order is (radius, center x, center y)
		std::vector<Circle> r_circles1, g_circles1, c_circles1, p_circles1, r_circles2, g_circles2, c_circles2, p_circles2; //circles
		std::vector< std::vector<Circle> > temp_circles{r_circles1, g_circles1, c_circles1, p_circles1};// putting them in a vector
		std::vector< std::vector<Circle> > final_circles{r_circles2, g_circles2, c_circles2, p_circles2};// final circles
		int current_index;// tells which index is currently the greatest
		int current_compare;// tells which index is the compare index
		int current_number;// current number of circles found
		int desired_number;// desired number of circles to find
		bool end_reached;// tells when the current compare has exceeded the length of the vector
		
		
		/********* Constructor for the image processing *********/
		ImageProcessing() : it_(nh)
		{
			image_sub_ = it_.subscribe( "/bebop/image_raw", 1, &ImageProcessing::image_callback, this);// subscribe to ardrone front camera
			image_pub_ = it_.advertise("processed_image", 1);// publish processed image
			pixel_pub = nh.advertise<homog_track::ImageProcessingMsg>("feature_pixels", 1);//pixel publisher
			A = cv::Mat::zeros(3,3,CV_64F); A.at<double>(0,0) = 396.17782; A.at<double>(0,2) = 322.453185; A.at<double>(1,1) = 399.798333; A.at<double>(1,2) = 174.243174; A.at<double>(2,2) = 1; // camera matrix for the ardrone
			dC.push_back(-0.001983); dC.push_back(0.015844); dC.push_back(-0.003171); dC.push_back(0.001506); dC.push_back(0.0); // distortion matrix for the ardrone
		}

		/********* Callback for the image processing *********/
		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{
			// trying to initialize the ptr to the image passed in and
			// converting it to a bgr8 image so opencv can use it
			// if it fails will through the error
			try
			{
				/********** begin processing the image **********/
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);// converting the image
				distorted_frame = cv_ptr->image;// getting the frame from the converted image can use clone on image
				cv::undistort(distorted_frame,frame,A,dC);// undistorting the frame
				cv::GaussianBlur( frame, blur_frame, cv::Size(blur_kernel_size,blur_kernel_size),0,0 );//low pass filter
				cv::cvtColor(blur_frame,hsv_frame,CV_BGR2HSV);// converting the frame to hsv
				
				// filtering the image for the  different colors, need to do it four times, once for each circle color
				for (int ii = 0; ii < 4; ii++)
				{
					filter_image_for_color( hsv_frame,
											binary_frames[ii],
											thresh_current[ii],
											thresh_max,
											erode_element,
											dilate_element 
										  );
				}
				/********** finding all the contours then the max radius circles for each color **********/
				for (int ii = 0; ii < 4; ii++)
				{
					cv::findContours(binary_frames[ii].clone(), contours[ii], CV_RETR_LIST, CV_CHAIN_APPROX_NONE);    
					std::vector<cv::Point2d> center( contours[ii].size() );// declaring centers to be all zeros the size of contours to hold all the contours circles centers
					std::vector<float> radius( contours[ii].size() );// declaring radius to be all zeros the size of contours to hold all the contours circles radius
					cv::Moments mu_temp;
					cv::Point2f center_temp;
					// getting the minimum enclcosing circle for the contours
					for (int jj = 0; jj < contours[ii].size(); jj++)
					{
						mu_temp = cv::moments(contours[ii][jj],false);
						center[jj] = cv::Point2d(mu_temp.m10/mu_temp.m00,mu_temp.m01/mu_temp.m00);
						minEnclosingCircle( contours[ii][jj], center_temp, radius[jj] );// getting the circle for the current contour
						temp_circle = new Circle;// create the new var for storing
						temp_circle->setCircle(radius[jj], center[jj].x, center[jj].y);// giving it to the circle
						temp_circles[ii].push_back (*temp_circle);// adding the circle to the temp array
					}
					/********** getting the max circle for each color **********/
					// resetting the search values
					current_number = 0;
					current_index = 0;
					current_compare = 0;
					end_reached = false;
					desired_number = 0;
					// determing if the circle vector is not empty and if not saying to find the max circle
					if (!temp_circles[ii].empty())
					{
						desired_number = 1;
					}
					// getting the number of maxes needed and adding them to the final vector as long as it is not empty
					while ( (desired_number > current_number) && !temp_circles[ii].empty() )
					{
						// first checking if the current index and current comapre are the same will increment
						// to the next compare. if it exceeds loop will take the current index for the max
						if (current_index == current_compare)
						{
							current_compare++;// incrementing to the next compare
							// if the current index sends current compare past the end will take the current index to be the max
							if ( current_compare > (temp_circles[ii].size() - 1) )
							{
								end_reached = true;
							}
						}
						// If the radius at the current compare is greater than the radius at the current index
						// will reset the current index to be the curent compare, then reset the curent compare
						// to restart the search
						else if ( temp_circles[ii][current_compare].radius > temp_circles[ii][current_index].radius)
						{
							current_index = current_compare;// resetting the index to be the compare
							current_compare = 0;// restting the index
						}
						// else is if the radius at the current index is greater than the current compare will increment
						// to the next compare. if it exceeds the vector will take the current index for the max.
						else
						{
							current_compare++;// incrementing to the next compare
							// if the current index sends current compare past the end will take the current index to be the max
							if ( current_compare > (temp_circles[ii].size() - 1) )
							{
								end_reached = true;
							}
						}
						// checking if the end has been reached. if it has then will take the current index to be
						// the larget index and add it to the final vector and increment the current number. then
						// will remove the circle from the temp vector.
						if (end_reached)
						{
							final_circles[ii].push_back ( temp_circles[ii][current_index] );// adding the circle to the final circle vector
							temp_circles[ii].erase( temp_circles[ii].begin() + current_index );// removing the value form the temporary circles
							current_number++;// incrementing the current number
							current_index = 0;// reset current index
							current_compare = 0;// reset current compare
							end_reached = false;// resetting end reached
						}
					}
					temp_circles[ii].erase(temp_circles[ii].begin(),temp_circles[ii].end());// erasing all the temp circles
				}		 
				//std::cout << "number of red circles: " << final_circles[0].size() << std::endl;
				//std::cout << "number of green circles: " << final_circles[1].size() << std::endl;
				//std::cout << "number of cyan circles: " << final_circles[2].size() << std::endl;
				//std::cout << "number of purple circles: " << final_circles[3].size() << std::endl;
				
				/********** drawing the circles and the contours **********/
				// do this four times for all the different color circles
				// selecting the color of the circle
				for (int ii = 0; ii < 4; ii++)
				{
					// drawing the circles if there is one
					if (!final_circles[ii].empty())
					{
						cv::Scalar* color_con = new cv::Scalar();// color for the drawing
						// choosing which circle based on ii value
						switch (ii)
						{
							// red
							case 0:
								//std::cout << "red found" << std::endl;
								color_con = new cv::Scalar(0, 0, 255);
								red_circle_p_curr.x = final_circles[ii][0].x; red_circle_p_curr.y = final_circles[ii][0].y; red_circle_p_curr.z = 1;// assigning the values of the circle to the circle
								break;
								
							// green
							case 1:
								//std::cout << "green found" << std::endl;
								color_con = new cv::Scalar(0, 255, 0);
								green_circle_p_curr.x = final_circles[ii][0].x; green_circle_p_curr.y = final_circles[ii][0].y; green_circle_p_curr.z = 1;// assigning the values of the circle to the circle
								break;
								
							// cyan
							case 2:
								//std::cout << "cyan found" << std::endl;
								color_con = new cv::Scalar(255, 255, 0);
								cyan_circle_p_curr.x = final_circles[ii][0].x; cyan_circle_p_curr.y = final_circles[ii][0].y; cyan_circle_p_curr.z = 1;// assigning the values of the circle to the circle
								break;
								
							// purple
							case 3:
								//std::cout << "purple found" << std::endl;
								color_con = new cv::Scalar(255, 0, 255);
								purple_circle_p_curr.x = final_circles[ii][0].x; purple_circle_p_curr.y = final_circles[ii][0].y; purple_circle_p_curr.z = 1;// assigning the values of the circle to the circle
								break;
						}
						// if it is the first run will just use the value brought in and set up the last value for next time
						if (first_run)
						{
							first_run = false;
							red_circle_p = red_circle_p_curr; green_circle_p = green_circle_p_curr; cyan_circle_p = cyan_circle_p_curr; purple_circle_p = purple_circle_p_curr; // assigning the values of the circle to the circle
							red_circle_p_last = red_circle_p; green_circle_p_last = green_circle_p; cyan_circle_p_last = cyan_circle_p; purple_circle_p_last = purple_circle_p;// updating the last
						}
						else
						{
							// assiging the values as a function of the current and the last
							red_circle_p.x = low_pass_gain*red_circle_p_curr.x + (1 - low_pass_gain)*red_circle_p_last.x;
							red_circle_p.y = low_pass_gain*red_circle_p_curr.y + (1 - low_pass_gain)*red_circle_p_last.y;
							red_circle_p.z = 1;
							green_circle_p.x = low_pass_gain*green_circle_p_curr.x + (1 - low_pass_gain)*green_circle_p_last.x;
							green_circle_p.y = low_pass_gain*green_circle_p_curr.y + (1 - low_pass_gain)*green_circle_p_last.y;
							green_circle_p.z = 1;
							cyan_circle_p.x = low_pass_gain*cyan_circle_p_curr.x + (1 - low_pass_gain)*cyan_circle_p_last.x;
							cyan_circle_p.y = low_pass_gain*cyan_circle_p_curr.y + (1 - low_pass_gain)*cyan_circle_p_last.y;
							cyan_circle_p.z = 1;
							purple_circle_p.x = low_pass_gain*purple_circle_p_curr.x + (1 - low_pass_gain)*purple_circle_p_last.x;
							purple_circle_p.y = low_pass_gain*purple_circle_p_curr.y + (1 - low_pass_gain)*purple_circle_p_last.y;
							purple_circle_p.z = 1;

						}
						cv::drawContours(blur_frame, contours[ii], -1, *color_con, 1);// draw all the contours
						cv::circle( blur_frame, cv::Point(final_circles[ii][0].x, final_circles[ii][0].y ),
									final_circles[ii][0].radius,
									*color_con, 2 );
						// printing out the circles values
						//final_circles[ii][0].printCircle();
					}
					// if the circle is missing will make its location -1 to indicate
					else
					{
						// choosing which color to print based on the circle
						switch (ii)
						{
							// red
							case 0:
								//std::cout << "red missing" << std::endl;
								red_circle_p.x = -1; red_circle_p.y = -1; red_circle_p.z = -1;// assigning the values of the circle to the circle
								break;
								
							// green
							case 1:
								//std::cout << "green missing" << std::endl;
								green_circle_p.x = -1; green_circle_p.y = -1; green_circle_p.z = -1;// assigning the values of the circle to the circle
								break;
								
							// cyan
							case 2:
								//std::cout << "cyan missing" << std::endl;
								cyan_circle_p.x = -1; cyan_circle_p.y = -1; cyan_circle_p.z = -1;// assigning the values of the circle to the circle
								break;
								
							// purple
							case 3:
								//std::cout << "purple missing" << std::endl;
								purple_circle_p.x = -1;	purple_circle_p.y = -1;	purple_circle_p.z = -1;// assigning the values of the circle to the circle
								break;
						}
					}
						
				}
				
				// publish the processed image
				//cv_ptr->image = frame;
				cv_ptr->image = blur_frame;
				image_pub_.publish(cv_ptr->toImageMsg());
				
				// giving the circles to the output message
				pixels_out.header.stamp = msg->header.stamp;
				pixels_out.pr = red_circle_p; pixels_out.pg = green_circle_p; pixels_out.pc = cyan_circle_p; pixels_out.pp = purple_circle_p;//circles

				// publish the circle points
				pixel_pub.publish(pixels_out);
				
				camera_updated = true;
				
				// erasing all the circles and restting the circles_xy to 0
				for (int ii = 0; ii < 4; ii++)
				{
					final_circles[ii].erase(final_circles[ii].begin(),final_circles[ii].end());
				}
			}
			// if the conversion fails 
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}
};

/********** filter images for a color **********/
void filter_image_for_color(cv::Mat& hsv_src, cv::Mat& bin_src, int* thresh_src, int* thesh_max_src, cv::Mat erode_src, cv::Mat dilate_src)
{
        // get the threshold image (src lower bound upper bound output
        // if the value of the hue upper is less than the value of the hue lower will do two thresholds
        // to get the value for the hue. this helps with red because it is on both sides
        if (thresh_src[0] > thresh_src[1])
        {
            // getting the lower bound first then the upper bound
            // after both are retrieved will or mask the two and give the output to the final
            cv::Mat temp_bin_lower, temp_bin_upper;
            
            //getting the upper region of the hue
            cv::inRange( hsv_src, 
                     cv::Scalar( thresh_src[0],
                                 thresh_src[2], 
                                 thresh_src[4] 
                               ),
                     cv::Scalar( thesh_max_src[0],
                                 thresh_src[3],
                                 thresh_src[5]
                               ),
                     temp_bin_upper
                   );
            
            // getting the lower region of the hue
            cv::inRange( hsv_src, 
                     cv::Scalar( 0,
                                 thresh_src[2], 
                                 thresh_src[4] 
                               ),
                     cv::Scalar( thresh_src[1],
                                 thresh_src[3],
                                 thresh_src[5]
                               ),
                     temp_bin_lower
                   );
            
            // combining the two into the final mask
            cv::bitwise_or(temp_bin_lower,temp_bin_upper,bin_src);
        }
        else
        {
            cv::inRange( hsv_src, 
                     cv::Scalar( thresh_src[0],
                                 thresh_src[2], 
                                 thresh_src[4] 
                               ),
                     cv::Scalar( thresh_src[1],
                                 thresh_src[3],
                                 thresh_src[5]
                               ),
                     bin_src 
                   );
        }

        // eroding then dilating the binary image
        // eroding
        cv::erode(bin_src, bin_src, erode_src);

        //dilating
        cv::dilate(bin_src, bin_src, dilate_src);

}

/********** Homography Decomposition **********/
class HomogDecomp
{
	public:
		/********** node and topics **********/
		ros::NodeHandle nh;// handle for the image processing
		ros::Subscriber pixel_sub, joy_sub;// pixel and joy subscriber
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
		tf::Matrix3x3 A_tf = tf::Matrix3x3(396.17782, 0, 322.453185,
										0, 399.798333, 174.243174,
										0, 0, 1);// camera matrix// camera matrix
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
		bool set_reference = false;
		int rb_button_teleop_b5 = 0;
		// constructor for the complete set of markers
		HomogDecomp()
		{
			pixel_sub = nh.subscribe("feature_pixels",1, &HomogDecomp::pixels_callback, this);// subscribing to the complete message
			joy_sub = nh.subscribe("joy",1,&HomogDecomp::joy_callback,this);
			homog_decomp_pub = nh.advertise<homog_track::DecompMsg>("decomposed_homography",1);// publisher for the decomposed stuff
			
			/********** decomp parameters **********/
			A = cv::Mat::zeros(3,3,CV_64F); 
			for (int ii = 0; ii < 3; ii++)
			{
				for (int jj = 0; jj < 3; jj++)
				{
					A.at<double>(ii,jj) = A_tf[ii][jj];
				}
			}
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
			cam_pixels.pr.x = -1; cam_pixels.pr.y = -1; cam_pixels.pr.z = -1;
			cam_pixels.pg.x = -1; cam_pixels.pg.y = -1; cam_pixels.pg.z = -1;
			cam_pixels.pc.x = -1; cam_pixels.pc.y = -1; cam_pixels.pc.z = -1;
			cam_pixels.pp.x = -1; cam_pixels.pp.y = -1; cam_pixels.pp.z = -1;
			ref_cam_pixels.pr.x = -1; ref_cam_pixels.pr.y = -1; ref_cam_pixels.pr.z = -1;
			ref_cam_pixels.pg.x = -1; ref_cam_pixels.pg.y = -1; ref_cam_pixels.pg.z = -1;
			ref_cam_pixels.pc.x = -1; ref_cam_pixels.pc.y = -1; ref_cam_pixels.pc.z = -1;
			ref_cam_pixels.pp.x = -1; ref_cam_pixels.pp.y = -1; ref_cam_pixels.pp.z = -1;
		}
		
		/********** callback for the controller **********/
		void joy_callback(const sensor_msgs::Joy& msg)
		{
			rb_button_teleop_b5 = msg.buttons[5];// right bumper says to set reference image
			if (rb_button_teleop_b5 > 0 && cam_pixels.pr.x != -1 && cam_pixels.pg.x != -1 && cam_pixels.pc.x != -1 && cam_pixels.pp.x != -1)
			{
				ref_pixels.erase(ref_pixels.begin(),ref_pixels.end());
				ref_points_m.erase(ref_points_m.begin(),ref_points_m.end());
				pr_ref.setValue(cam_pixels.pr.x,cam_pixels.pr.y,cam_pixels.pr.z);
				pg_ref.setValue(cam_pixels.pg.x,cam_pixels.pg.y,cam_pixels.pg.z);
				pc_ref.setValue(cam_pixels.pc.x,cam_pixels.pc.y,cam_pixels.pc.z);
				pp_ref.setValue(cam_pixels.pp.x,cam_pixels.pp.y,cam_pixels.pp.z);
				pr_ref_gm.x = pr_ref.getX(); pr_ref_gm.y = pr_ref.getY(); pr_ref_gm.z = pr_ref.getZ();// red
				pg_ref_gm.x = pg_ref.getX(); pg_ref_gm.y = pg_ref.getY(); pg_ref_gm.z = pg_ref.getZ();// grenn
				pc_ref_gm.x = pc_ref.getX(); pc_ref_gm.y = pc_ref.getY(); pc_ref_gm.z = pc_ref.getZ();// cyan
				pp_ref_gm.x = pp_ref.getX(); pp_ref_gm.y = pp_ref.getY(); pp_ref_gm.z = pp_ref.getZ();// purple
				ref_cam_pixels.pr = pr_ref_gm; ref_cam_pixels.pg = pg_ref_gm; ref_cam_pixels.pc = pc_ref_gm; ref_cam_pixels.pp = pp_ref_gm;//reference pixels for message
				ref_pixels.push_back(cv::Point2d(pr_ref.getX(),pr_ref.getY()));//red for find homography
				ref_pixels.push_back(cv::Point2d(pg_ref.getX(),pg_ref.getY()));//green for find homography
				ref_pixels.push_back(cv::Point2d(pc_ref.getX(),pc_ref.getY()));//cyan for find homography
				ref_pixels.push_back(cv::Point2d(pp_ref.getX(),pp_ref.getY()));//purple for find homography
							
				pr_ref_m = cv::Mat::ones(3,1,CV_64F);
				pg_ref_m = cv::Mat::ones(3,1,CV_64F);
				pc_ref_m = cv::Mat::ones(3,1,CV_64F);
				pp_ref_m = cv::Mat::ones(3,1,CV_64F);
				
				pr_ref_m.at<double>(0,0) = pr_ref.getX(); pr_ref_m.at<double>(1,0) = pr_ref.getY(); pr_ref_m.at<double>(2,0) = pr_ref.getZ(); 
				pg_ref_m.at<double>(0,0) = pg_ref.getX(); pg_ref_m.at<double>(1,0) = pg_ref.getY(); pg_ref_m.at<double>(2,0) = pg_ref.getZ(); 
				pc_ref_m.at<double>(0,0) = pc_ref.getX(); pc_ref_m.at<double>(1,0) = pc_ref.getY(); pc_ref_m.at<double>(2,0) = pc_ref.getZ(); 
				pp_ref_m.at<double>(0,0) = pp_ref.getX(); pp_ref_m.at<double>(1,0) = pp_ref.getY(); pp_ref_m.at<double>(2,0) = pp_ref.getZ(); 
				
				mr_ref_norm = A.inv(cv::DECOMP_LU)*pr_ref_m;
				mg_ref_norm = A.inv(cv::DECOMP_LU)*pg_ref_m;
				mc_ref_norm = A.inv(cv::DECOMP_LU)*pc_ref_m;
				mp_ref_norm = A.inv(cv::DECOMP_LU)*pp_ref_m;
				
				ref_points_m.push_back(mr_ref_norm);
				ref_points_m.push_back(mg_ref_norm);
				ref_points_m.push_back(mc_ref_norm);
				ref_points_m.push_back(mp_ref_norm);
				
				decomposed_msg.ref_cam_pixels = ref_cam_pixels;// reference pixels
				std::cout << "set reference pixels" << std::endl;
				set_reference = true;
			}
		}
		
		// callback for the complete message
		void pixels_callback(const homog_track::ImageProcessingMsg& msg)
		{
			// erasing all the point vectors and matrix vectors
			pixels.erase(pixels.begin(),pixels.end());
			curr_points_m.erase(curr_points_m.begin(),curr_points_m.end());
			
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
			
			alpha_red = -1;
			alpha_green = -1;
			alpha_cyan = -1;
			alpha_purple = -1;
			
			// if any of the points have a -1 will skip over the homography
			if (msg.pr.x != -1 && msg.pg.x != -1 && msg.pc.x != -1 && msg.pp.x != -1 && set_reference)
			{	
				//std::cout << "before find homography" << std::endl;
				try
				{
					/********** following the process outlined in the reference **********/			
					G = cv::findHomography(ref_pixels,pixels,0);// finding the perspective homography
					//std::cout << "after find homography" << std::endl;
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
					//std::cout << "after decompose homography matrix" << std::endl;
					// if the decomp is successful will find the solutions
					if (successful_decomp > 0)
					{
						// finding the alphas
						alpha_red = mr_norm.at<double>(2,0)/((H.row(2)).dot(mr_ref_norm.t()));
						alpha_green = mg_norm.at<double>(2,0)/((H.row(2)).dot(mg_ref_norm.t()));
						alpha_cyan = mc_norm.at<double>(2,0)/((H.row(2)).dot(mc_ref_norm.t()));
						alpha_purple = mp_norm.at<double>(2,0)/((H.row(2)).dot(mp_ref_norm.t()));
						
						//std::cout << "after alpha calcs" << std::endl;
						
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
						
						//std::cout << "after getting all 4 solutions of homog decomp" << std::endl;
						
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
						
						//std::cout << "after getting the 2 solutions solutions of homog decomp" << std::endl;
						
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
						
						//std::cout << "after setting the R solutions" << std::endl;
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
							if (second_solution_found)
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
						
						//std::cout << "after setting the R,t, and n solutions into the message" << std::endl;
					}
				}
				catch (cv::Exception)
				{
					std::cout << "homg decomp failed" << std::endl;
				}
			}
			else
			{
				/********** setting the message and sending it **********/
				// rotation matricies
				for (int ii = 0; ii < 9; ii++)
				{
					decomposed_msg.R1[ii] = -1;
					decomposed_msg.R2[ii] = -1;
				}
				// normal and normalized translation
				for (int ii = 0; ii < 3; ii++)
				{
					decomposed_msg.t1[ii] = -1;
					decomposed_msg.n1[ii] = -1;
					decomposed_msg.t2[ii] = -1;
					decomposed_msg.n2[ii] = -1;
				}
			}
			
			cam_pixels.pr = msg.pr; cam_pixels.pg = msg.pg; cam_pixels.pc = msg.pc; cam_pixels.pp = msg.pp;
			decomposed_msg.alphar = alpha_red; decomposed_msg.alphag = alpha_green; decomposed_msg.alphac = alpha_cyan; decomposed_msg.alphap = alpha_purple;// alphas
			decomposed_msg.cam_pixels = cam_pixels;// pixels
			decomposed_msg.header.stamp = msg.header.stamp;// time
			decomposed_msg.header.frame_id = "decomp_message";
			homog_decomp_pub.publish(decomposed_msg);// sending the message
			//std::cout << "decomp updated" << std::endl;
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
		ros::Subscriber decomp_sub, joy_sub, body_imu_sub, body_mocap_sub, camera_state_sub, image_imu_sub;
		ros::Publisher takeoff_pub, land_pub, reset_pub, cmd_vel_pub, desired_pose_pub, current_pose_pub, desired_pixels_pub, current_error_pub, vc_marker_pub, wc_marker_pub;
		
		/********** Time Values **********/
		ros::Time last_time;// last time the camera was updated
		ros::Time current_time;// current time the camera was updated
		ros::Time start_time;// start time
		ros::Time last_time_d;//desired last time
		ros::Time current_time_d;//desired current time
		ros::Time start_time_d;//starting time
		
		/********** File Declarations **********/
		bool write_to_file;
		std::fstream output_file;
		std::string output_file_name; // file name
		
		/********** Gains **********/
		double Kws = 0.1;// K_w scalar
		tf::Matrix3x3 Kw = tf::Matrix3x3(Kws,0,0,
										 0,Kws,0,
										 0,0,Kws);// rotational gain matrix initialize to identity
		double Kvs = 0.3;
		tf::Matrix3x3 Kv = tf::Matrix3x3(Kvs,0,0,
										 0,Kvs,0,
										 0,0,Kvs);// linear velocity gain matrix
		double gamma_1 = 0.001;// control gains for the z_star_hat_dot calculation
		double gamma_2 = 0.0001;
		double zr_star_hat = 2;// current value for zr_star_hat
		
		/********** Points **********/
		tf::Vector3 P_red_wrt_world, P_green_wrt_world, P_cyan_wrt_world, P_purple_wrt_world;// homography points wrt world
		tf::Transform red_wrt_world, green_wrt_world, cyan_wrt_world, purple_wrt_world;//transforms for the marker points
		tf::Transform red_wrt_world_calc, green_wrt_world_calc, cyan_wrt_world_calc, purple_wrt_world_calc;//transforms for the marker points calcs
		tf::Transform red_wrt_camera_calc, green_wrt_camera_calc, cyan_wrt_camera_calc, purple_wrt_camera_calc;//transforms for the marker points calcs
		tf::Transform red_wrt_des_camera_calc, green_wrt_des_camera_calc, cyan_wrt_des_camera_calc, purple_wrt_des_camera_calc;//transforms for the marker points desired calcs
		
		/********** UAV body **********/
		tf::Transform camera_init_wrt_body;//transform of the camera init wrt the body
		tf::Transform camera_wrt_body_init;// camera wrt body init
		tf::Transform camera_wrt_camera_init;//transform of the current
		tf::Transform camera_wrt_body;
		tf::Vector3 w_body = tf::Vector3(0,0,0);// angular velocity of the body wrt world expressed in the body frame
		tf::Transform body_wrt_world_mocap;//pose of body from the mocap
		tf::Transform body_wrt_world_calc;//pose of body from calculations
		double gimbal_angle = -55*M_PIl/180.0;// rotation of gimbal about tilt axis
		tf::Vector3 w_image = tf::Vector3(0,0,0);// image angular velocity wrt world expressed in image frame
		
		/********** Reference **********/
		tf::Vector3 mr_bar_ref, mg_bar_ref, mc_bar_ref, mp_bar_ref;// points wrt reference
		tf::Vector3 pr_ref, pg_ref, pc_ref, pp_ref;// pixels wrt reference
		tf::Transform reference_wrt_world, red_wrt_reference; // transform for reference camera and the red marker wrt the reference
		tf::Vector3 n_ref;
		tf::Vector3 n_world;
		ros::Time last_body_pose_time;
		
		/********** Camera **********/
		tf::Vector3 mr_bar, mg_bar, mc_bar, mp_bar;// points wrt camera
		tf::Vector3 mr, mg, mc, mp;// normalized points wrt camera
		tf::Vector3 pr, pg, pc, pp;// points pixels wrt camera
		geometry_msgs::Point pr_gm, pg_gm, pc_gm, pp_gm;// points pixels wrt camera for message
		homog_track::ImageProcessingMsg pixels_out;
		tf::Transform camera_wrt_world; // transform for camera
		tf::Transform camera_wrt_world_calc;
		
		/********** Decomp Declarations **********/
		tf::Transform camera_wrt_reference;// transform of camera wrt reference
		tf::Quaternion Q_cf = tf::Quaternion(0,0,0,0), Q_cf_last = tf::Quaternion(0,0,0,0), Q_cf_negated = tf::Quaternion(0,0,0,0);// camera wrt reference, last camera wrt reference, and negated camera wrt reference
		double Q_norm_current_diff, Q_norm_negated_diff;// norms to determine which rotation is closer to last
		double alpha_red, alpha_green, alpha_cyan, alpha_purple;// alphas for the camera		
		
		/********* Cam info **********/
		tf::Matrix3x3 A = tf::Matrix3x3(396.17782, 0, 322.453185,
										0, 399.798333, 174.243174,
										0, 0, 1);// camera matrix// camera matrix
		tf::Matrix3x3 principle_point = tf::Matrix3x3(0,0,322.453185, // u0
													  0,0,174.243174, // v0
													  0,0,0);// matrix with the principle point coordinates of the camera								
		/********** Velocity Commands **********/
		tf::Vector3 vc, wc;//linear and angular velocity commands
		cv::Mat wc_cv;
		
		/********** Desired Declarations **********/
		tf::Vector3 mrd_bar, mgd_bar, mcd_bar, mpd_bar;// points wrt desired
		tf::Vector3 mrd;
		tf::Vector3 prd, pgd, pcd, ppd;// pixels wrt desired
		tf::Transform desired_wrt_world;// transform for desired wrt world
		tf::Transform desired_body_wrt_world;// transform for desired body wrt world
		tf::Quaternion Q_dw = tf::Quaternion(0,0,0,0), Q_df = tf::Quaternion(0,0,0,0), Q_df_negated = tf::Quaternion(0,0,0,0), Q_df_last = tf::Quaternion(0,0,0,0);// desired wrt reference, last desired wrt reference, and negated desired wrt reference
		cv::Mat wcd_cv;// desired angular velocity
		tf::Vector3 vcd;// desired linear velocity
		double desired_radius;// desired radius
		double desired_period;// desired period
		double alpha_red_d;
		homog_track::ImageProcessingMsg desired_pixels;
		
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
		std::deque<tf::Vector3> Ev_samples, Phi_samples, Uvar_samples;// buffer samples for learning stack terms
		std::deque<double> trapz_timestamps;// times for buffer samples
		std::deque<tf::Vector3> Us, Evs_minus_Phis;// learning stack terms
		std::deque<double> Evs_minus_Phis_svds;// norms for the Ev - Phi stack
		tf::Vector3 Ev_minus_Phi;// current Ev - Phi
		double Ev_minus_Phi_svd;// current Ev - Phi svd
		double curr_Ev_minus_Phi_svd_min;// min svd on the Ev - Phi svd stack
		std::deque<double>::iterator curr_Ev_minus_Phi_svd_min_iter;// iterator to the min norm on the stack
		int curr_Ev_minus_Phi_svd_min_index;
		
		/********** zr hat estimate Controller **********/
		double zr_star_hat_dot = 0;// current value for zr_star_hat_dot = gamma1*transpose(ev)*phi + gamma1*gamma2*sum_from_k=1_to_n( transpose(Ev(k)-Phi(k))*(-(Ev(k)-Phi(k))*zr_star_hat - U(k)) )
		double zr_star_hat_dot_sum = 0;// temp for the summation in zr_star_hat_dot
		bool first_time_update = true; // first time update occurance
		bool first_desired_recieved = false, first_marker_recieved = false, first_decomp_recieved = false; // first messages recieved
		bool desired_recieved = false, marker_recieved = false, decomp_recieved = false; // messages recieved
		bool start_controller = false;// start the controller when the right bumper is pressed
		bool start_autonomous = false;
		bool new_decomp_recieved = false;
		bool new_joy_recieved = false;
		
		/********** output command choice and xbox controller **********/
		geometry_msgs::Twist velocity_command;
		geometry_msgs::Pose velocity_as_pose;
		int a_button_land_b0 = 0, b_button_reset_b1 = 0, y_button_takeoff_b3 = 0, lb_button_teleop_b4 = 0, rb_button_teleop_b5 = 0;
		double rt_stick_ud_x_a3 = 0, rt_stick_lr_y_a2 = 0, lt_stick_ud_z_a1 = 0, lt_stick_lr_th_a0 = 0;
		
		/********** marker **********/
		visualization_msgs::Marker vc_marker, wc_marker;
		geometry_msgs::Point vc_marker_start, vc_marker_end, wc_marker_start, wc_marker_end;
		
		Controller(double loop_rate_des, bool write_to_file_des, std::string& output_file_name_des)
		{	
			output_file_name = output_file_name_des;
			//std::cout << "output_file_name: " << output_file_name << std::endl;
			write_to_file = write_to_file_des;
			loop_rate_hz = loop_rate_des;
			number_of_integrating_samples = loop_rate_hz*0.2;
			decomp_sub = nh.subscribe("decomposed_homography",1, &Controller::decomp_callback, this);// subscribing to the decomp message
			body_imu_sub = nh.subscribe("/bebop/body_vel", 1, &Controller::body_imu_callback, this);// subscribing to the body velocity publisher
			body_mocap_sub = nh.subscribe("/bebop/pose", 1, &Controller::body_pose_callback, this);// subscribing to the p
			image_imu_sub = nh.subscribe("/bebop_image/body_vel",1,&Controller::image_imu_callback,this);// subscribing to the image velocity publisher
			joy_sub = nh.subscribe("joy",1,&Controller::joy_callback,this);//subscribing to the joy node
			cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_from_control",1);// publisher for the decomposed stuff
			desired_pose_pub = nh.advertise<geometry_msgs::Pose>("desired_pose",1);//desire pose message
			desired_pixels_pub = nh.advertise<homog_track::ImageProcessingMsg>("desired_pixels",1);//desire pixels message
			current_pose_pub = nh.advertise<geometry_msgs::Pose>("current_pose",1);//current pose message
			current_error_pub = nh.advertise<geometry_msgs::Pose>("ev_and_qtilde",1);//current error
			takeoff_pub = nh.advertise<std_msgs::Empty>("bebop/takeoff",1);
			land_pub = nh.advertise<std_msgs::Empty>("bebop/land",1);
			reset_pub = nh.advertise<std_msgs::Empty>("bebop/reset",1);
			vc_marker_pub = nh.advertise<visualization_msgs::Marker>( "vc_marker", 0 );
			wc_marker_pub = nh.advertise<visualization_msgs::Marker>( "wc_marker", 0 );
			//camera_state_sub = nh.subscribe("/bebop/camera_control",1,&Controller::camera_state_callback,this);//get the current gimbal desired center
			
			/********** Set transform for body wrt world from mocap and calculated **********/
			body_wrt_world_mocap.setOrigin(tf::Vector3(0,0,0));
			body_wrt_world_mocap.setRotation(tf::Quaternion(0,0,0,1));
			body_wrt_world_calc.setOrigin(tf::Vector3(0,0,0));
			body_wrt_world_calc.setRotation(tf::Quaternion(0,0,0,1));
			
			/********** Camera with respect to body initial **********/
			camera_init_wrt_body.setOrigin(tf::Vector3(0.053144708904881, -0.007325857858683, -0.063096991249463));
			camera_init_wrt_body.setRotation(tf::Quaternion(-0.509759529319467,  0.498136685988136, -0.496486743347509,  0.495424003823942));
			camera_wrt_camera_init.setOrigin(tf::Vector3(0,0,0));
			tf::Matrix3x3 camera_wrt_camera_init_rot_temp = tf::Matrix3x3(1,0,0,
																		  0,std::cos(gimbal_angle),-1*std::sin(gimbal_angle),
																		  0,std::sin(gimbal_angle),std::cos(gimbal_angle));
			tf::Quaternion camera_wrt_camera_init_Q_temp;
			camera_wrt_camera_init_rot_temp.getRotation(camera_wrt_camera_init_Q_temp);
			camera_wrt_camera_init.setRotation(camera_wrt_camera_init_Q_temp);
			camera_wrt_body_init = camera_init_wrt_body*camera_wrt_camera_init;
			
			/******************** Writing headers to file ********************/
			if (write_to_file)
			{
				output_file.open(output_file_name, std::fstream::out | std::fstream::app);
			
				if (output_file.is_open())
				{
					output_file << "time,"
								<< "left_bumper,"
								<< "prx," << "pry," << "prz,"
								<< "prdx," << "prdy," << "prdz,"
								<< "mrx," << "mry," << "mrz,"
								<< "mrdx," << "mrdy," << "mrdz,"
								<< "red_calc_x," << "red_calc_y," << "red_calc_z,"
								<< "green_calc_x," << "green_calc_y," << "green_calc_z,"
								<< "cyan_calc_x," << "cyan_calc_y," << "cyan_calc_z,"
								<< "purple_calc_x," << "purple_calc_y," << "purple_calc_z,"
								<< "red_x," << "red_y," << "red_z,"
								<< "green_x," << "green_y," << "green_z,"
								<< "cyan_x," << "cyan_y," << "cyan_z,"
								<< "purple_x," << "purple_y," << "purple_z,"
								<< "desired_orig_x," << "desired_orig_y," << "desired_orig_z,"
								<< "camera_calc_orig_x," << "camera_calc_orig_y," << "camera_calc_orig_z,"
								<< "camera_mocap_orig_x," << "camera_mocap_orig_y," << "camera_mocap_orig_z,"
								<< "desired_rot_w," << "desired_rot_x," << "desired_rot_y," << "desired_rot_z,"
								<< "camera_calc_rot_w," << "camera_calc_rot_x," << "camera_calc_rot_y," << "camera_calc_rot_z,"
								<< "camera_mocap_rot_w," << "camera_mocap_rot_x," << "camera_mocap_rot_y," << "camera_mocap_rot_z,"
								
								<< "body_mocap_orig_x," << "body_mocap_orig_y," << "body_mocap_orig_z,"
								
								<< "body_mocap_rot_w," << "body_mocap_rot_x," << "body_mocap_rot_y," << "body_mocap_rot_z,"
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
								<< "w_bodyimux," << "w_bodyimuy," << "w_bodyimuz,"
								<< "v_bodyx," << "v_bodyy," << "v_bodyz,"
								<< "w_bodyx," << "w_bodyy," << "w_bodyz,"
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
			
			/********** camera wrt world **********/
			camera_wrt_world.setIdentity();
			camera_wrt_world_calc.setIdentity();
			
			/********** markers wrt world **********/
			P_red_wrt_world = tf::Vector3(-0.05,-0.05,0); P_green_wrt_world = tf::Vector3(-0.05,0.05,0); P_cyan_wrt_world = tf::Vector3(0.05,0.05,0); P_purple_wrt_world = tf::Vector3(0.05,-0.05,0);// homography points wrt world
			red_wrt_world.setIdentity(); red_wrt_world.setOrigin(P_red_wrt_world);//red
			green_wrt_world.setIdentity(); green_wrt_world.setOrigin(P_green_wrt_world);//green
			cyan_wrt_world.setIdentity(); cyan_wrt_world.setOrigin(P_cyan_wrt_world);//cyan
			purple_wrt_world.setIdentity(); purple_wrt_world.setOrigin(P_purple_wrt_world);//purple
			red_wrt_world_calc.setIdentity(); red_wrt_world_calc.setOrigin(P_red_wrt_world);//red
			green_wrt_world_calc.setIdentity(); green_wrt_world_calc.setOrigin(P_green_wrt_world);//green
			cyan_wrt_world_calc.setIdentity(); cyan_wrt_world_calc.setOrigin(P_cyan_wrt_world);//cyan
			purple_wrt_world_calc.setIdentity(); purple_wrt_world_calc.setOrigin(P_purple_wrt_world);//purple
			red_wrt_camera_calc.setIdentity();//red
			green_wrt_camera_calc.setIdentity();//green
			cyan_wrt_camera_calc.setIdentity();//cyan
			purple_wrt_camera_calc.setIdentity();//purple
			red_wrt_des_camera_calc.setIdentity();//red
			green_wrt_des_camera_calc.setIdentity();//green
			cyan_wrt_des_camera_calc.setIdentity();//cyan
			purple_wrt_des_camera_calc.setIdentity();//purple

			/********** reference wrt world  **********/
			n_ref = tf::Vector3(0,0,0);
			n_world = tf::Vector3(0,0,-1);
			reference_wrt_world.setIdentity();
			red_wrt_reference.setIdentity();
			
			/********** desired wrt world **********/
			double zd_init = 1.5; //starting desired height
			desired_radius = 1;// desired radius in meters
			desired_period = 60;
			desired_body_wrt_world.setOrigin(tf::Vector3(-1*desired_radius,0,zd_init));//start body back 1 m along x and up 2 m
			tf::Matrix3x3 R_desired_body_wrt_world(1,0,0,
												   0,1,0,
												   0,0,1);//rotation for the body, initially aligned with world
			tf::Quaternion Q_desired_body_wrt_world; R_desired_body_wrt_world.getRotation(Q_desired_body_wrt_world);// get rotation as a quaternion
			desired_body_wrt_world.setRotation(Q_desired_body_wrt_world);// set the initial body transform rotation
			desired_wrt_world.setOrigin(desired_body_wrt_world.getOrigin());// desired camera has same origin as the body, no actual length for the virtual body
			desired_wrt_world.setRotation(desired_body_wrt_world.getRotation()*camera_wrt_body_init.getRotation());;// set the initial rotation of the camera, its the body_wrt_world*camera_wrt_body_init
			//tf::Vector3 wd_body_temp = tf::Vector3(0,0,2*M_PIl/desired_period);// desired angular velocity of the body
			//tf::Vector3 vd_body_temp = tf::Vector3(0,-1*wd_body_temp.getZ()*desired_radius,0);// desired linear velocity of the body
			tf::Vector3 wd_body_temp = tf::Vector3(0,0,0);// desired angular velocity of the body
			tf::Vector3 vd_body_temp = tf::Vector3(0,0,0);// desired linear velocity of the body
			
			// desired camera linear velocity is vcd =  q_cam_wrt_body.inverse()*vd body_temp*q_cam_wrt_body
			tf::Quaternion vcd_body_temp = ((camera_wrt_body_init.getRotation().inverse())*tf::Quaternion(vd_body_temp.getX(), vd_body_temp.getY(), vd_body_temp.getZ(), 0)) * camera_wrt_body_init.getRotation();
			vcd = tf::Vector3(vcd_body_temp.getX(), vcd_body_temp.getY(), vcd_body_temp.getZ());
			// desired angular velocity of the camera is wcd = q_cam_wrt_body.inverse() * wd_body_temp * q_cam_wrt_body
			tf::Quaternion wcd_temp = ((camera_wrt_body_init.getRotation().inverse()) * tf::Quaternion(wd_body_temp.getX(), wd_body_temp.getY(), wd_body_temp.getZ(), 0)) * camera_wrt_body_init.getRotation();
			wcd.setValue(wcd_temp.getX(), wcd_temp.getY(), wcd_temp.getZ());
			wcd_cv = cv::Mat::zeros(3,1,CV_64F);
			wcd_cv.at<double>(0,0) = wcd.getX(); wcd_cv.at<double>(1,0) = wcd.getY(); wcd_cv.at<double>(2,0) = wcd.getZ();
			
			/********** pixels wrt desired  **********/
			tf::Vector3 temp_v;
			tf::Quaternion temp_Q;
			temp_v = P_red_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mrd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mgd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mcd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mpd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			prd = A*((1/mrd_bar.getZ())*mrd_bar); pgd = A*((1/mgd_bar.getZ())*mgd_bar); pcd = A*((1/mcd_bar.getZ())*mcd_bar); ppd = A*((1/mpd_bar.getZ())*mpd_bar);// camera points as pixels
			//alpha_red_d = red_wrt_reference.getOrigin().getZ()/mrd_bar.getZ();
			desired_pixels.pr.x = prd.getX(); desired_pixels.pr.x = prd.getY(); desired_pixels.pr.x = prd.getZ();//red
			desired_pixels.pg.x = pgd.getX(); desired_pixels.pg.x = pgd.getY(); desired_pixels.pg.x = pgd.getZ();//green
			desired_pixels.pc.x = pcd.getX(); desired_pixels.pc.x = pcd.getY(); desired_pixels.pc.x = pcd.getZ();//cyan
			desired_pixels.pp.x = ppd.getX(); desired_pixels.pp.x = ppd.getY(); desired_pixels.pp.x = ppd.getZ();//purple
			desired_pixels_pub.publish(desired_pixels);
			
			/********* update time *********/
			last_time = ros::Time::now();
			
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
			
			last_time_d = last_time;//desired last time
			start_time_d = last_time;//starting time
			
			/********* marker **********/
			vc_marker.header.frame_id = "bebop_image";
			vc_marker.frame_locked = true;
			vc_marker.header.stamp = ros::Time::now();
			vc_marker.ns = "vc_marker";
			vc_marker.id = 0;
			vc_marker.type = visualization_msgs::Marker::ARROW;
			vc_marker.action = visualization_msgs::Marker::ADD;
			vc_marker_end.x = 0; vc_marker_end.y = 0; vc_marker_end.z = 0;
			vc_marker_start.x = 0; vc_marker_start.y = 0; vc_marker_start.z = 0;
			vc_marker.points.push_back(vc_marker_start);
			vc_marker.points.push_back(vc_marker_end);
			vc_marker.scale.x = 0.01;
			vc_marker.scale.y = 0.01;
			vc_marker.scale.z = 0.01;
			vc_marker.color.a = 1.0; // Don't forget to set the alpha!
			vc_marker.color.r = 0.5;
			vc_marker.color.g = 0.5;
			vc_marker.color.b = 0.0;
			vc_marker.lifetime = ros::Duration();
			wc_marker.header.frame_id = "bebop_image";
			wc_marker.frame_locked = true;
			wc_marker.header.stamp = ros::Time::now();
			wc_marker.ns = "wc_marker";
			wc_marker.id = 1;
			wc_marker.type = visualization_msgs::Marker::ARROW;
			wc_marker.action = visualization_msgs::Marker::ADD;
			wc_marker_end.x = 0; wc_marker_end.y = 0; wc_marker_end.z = 0;
			wc_marker_start.x = 0; wc_marker_start.y = 0; wc_marker_start.z = 0;
			wc_marker.points.push_back(wc_marker_start);
			wc_marker.points.push_back(wc_marker_end);
			wc_marker.scale = vc_marker.scale;
			wc_marker.color.a = 1.0; // Don't forget to set the alpha!
			wc_marker.color.r = 0.5;
			wc_marker.color.g = 0.0;
			wc_marker.color.b = 0.5;
			wc_marker.lifetime = vc_marker.lifetime;
			
		}
		
		/********** callback for the uav body imu **********/
		void body_imu_callback(const geometry_msgs::TwistStamped& msg)
		{
			w_body = tf::Vector3(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
		}
		
		/********** callback for the image imu **********/
		void image_imu_callback(const geometry_msgs::TwistStamped& msg)
		{
			w_image = tf::Vector3(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
		}
		
		/********** callback for the decomp node **********/
		void decomp_callback(const homog_track::DecompMsg& msg)
		{
			tf::Matrix3x3 R_fc_temp = tf::Matrix3x3(0,0,0,0,0,0,0,0,0);
			tf::Vector3 t_fc_temp = tf::Vector3(0,0,0);
			bool rotation_found = false;
			//std::cout << "entered decomp callback" << std::endl;
			current_time = msg.header.stamp;
			
			// because the reference is set to the exact value when when n should have only a z componenet, the correct
			// choice should be the one closest to n_ref = [0,0,1]^T which will be the one with the greatest dot product with n_ref
			
			if (msg.n1[2] != -1 && msg.n2[2] != -1)
			{
				tf::Vector3 n1_temp = tf::Vector3(msg.n1[0],msg.n1[1],msg.n1[2]);
				tf::Vector3 n2_temp = tf::Vector3(msg.n2[0],msg.n2[1],msg.n2[2]);
				if (n1_temp.dot(n_ref) >= n2_temp.dot(n_ref))
				{
					for (int ii = 0; ii < 9; ii++)
					{
						R_fc_temp[ii/3][ii%3] = msg.R1[ii];
					}
					t_fc_temp.setValue(msg.t1[0],msg.t1[1],msg.t1[2]);
				}
				else
				{
					for (int ii = 0; ii < 9; ii++)
					{
						R_fc_temp[ii/3][ii%3] = msg.R2[ii];
					}
					t_fc_temp.setValue(msg.t2[0],msg.t2[1],msg.t2[2]);
				}
				t_fc_temp *= reference_wrt_world.getOrigin().getZ();
				rotation_found = true;
			}
			else if(msg.n1[2] != -1 && msg.n2[2] == -1)
			{
				for (int ii = 0; ii < 9; ii++)
				{
					R_fc_temp[ii/3][ii%3] = msg.R1[ii];
				}
				t_fc_temp.setValue(msg.t1[0],msg.t1[1],msg.t1[2]);
				t_fc_temp *= reference_wrt_world.getOrigin().getZ();
				rotation_found = true;
			}
			
			pr.setX(msg.cam_pixels.pr.x); pr.setY(msg.cam_pixels.pr.y); pr.setZ(1);//red pixels
			//std::cout << "pr:\n x: " << pr.getX() << " y: " << pr.getY() << " z: " << pr.getZ() << std::endl;
			pg.setX(msg.cam_pixels.pg.x); pg.setY(msg.cam_pixels.pg.y); pg.setZ(1);//green pixels
			//std::cout << "pg:\n x: " << pg.getX() << " y: " << pg.getY() << " z: " << pg.getZ() << std::endl;
			pc.setX(msg.cam_pixels.pc.x); pc.setY(msg.cam_pixels.pc.y); pc.setZ(1);//cyan pixels
			//std::cout << "pc:\n x: " << pc.getX() << " y: " << pc.getY() << " z: " << pc.getZ() << std::endl;
			pp.setX(msg.cam_pixels.pp.x); pp.setY(msg.cam_pixels.pp.y); pp.setZ(1);//purple pixels
			//std::cout << "pp:\n x: " << pp.getX() << " y: " << pp.getY() << " z: " << pp.getZ() << std::endl;
			alpha_red = msg.alphar;
			alpha_green = msg.alphag;
			alpha_cyan = msg.alphac;
			alpha_purple = msg.alphap;
			
			// get the image wrt body
			try
			{
				tf::StampedTransform camera_wrt_body_temp;
				listener.waitForTransform("bebop", "bebop_image", current_time, ros::Duration(1.0));
				listener.lookupTransform("bebop", "bebop_image", current_time, camera_wrt_body_temp);
				camera_wrt_body.setOrigin(camera_wrt_body_temp.getOrigin());
				camera_wrt_body.setRotation(camera_wrt_body_temp.getRotation());
				//br.sendTransform(tf::StampedTransform(camera_wrt_body, current_time, "bebop", "camera_image"));// send the camera image wrt body
				//std::cout << "sent camera image wrt bebop" << std::endl;
			}
			catch (tf::TransformException ex)
			{
				std::cout << "failed to send camera wrt body" << std::endl;
			}
			if (rotation_found && start_controller)
			{
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
				
				try
				{
					//std::cout << "entered rotation found" << std::endl;
					
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
					
					// sending static transforms
					br.sendTransform(tf::StampedTransform(reference_wrt_world,current_time,"world", "reference_image"));
					br.sendTransform(tf::StampedTransform(red_wrt_world, current_time, "world", "red_wrt_world"));
					br.sendTransform(tf::StampedTransform(green_wrt_world, current_time, "world", "green_wrt_world"));
					br.sendTransform(tf::StampedTransform(cyan_wrt_world, current_time, "world", "cyan_wrt_world"));
					br.sendTransform(tf::StampedTransform(purple_wrt_world, current_time, "world", "purple_wrt_world"));
					
					//camera_wrt_reference.setOrigin(-1*((R_fc_temp.transpose())*t_fc_temp));
					tf::StampedTransform camera_wrt_reference_calc_temp;
					listener.waitForTransform("reference_image", "bebop_image", current_time, ros::Duration(1.0));
					listener.lookupTransform("reference_image", "bebop_image", current_time, camera_wrt_reference_calc_temp);
					camera_wrt_reference.setOrigin(camera_wrt_reference_calc_temp.getOrigin());
					
					br.sendTransform(tf::StampedTransform(camera_wrt_reference, current_time, "reference_image", "camera_image_calc"));
					
					//estimating the position of the red green cyan and purple features
					mr = A.inverse()*pr;
					mg = A.inverse()*pg;
					mc = A.inverse()*pc;
					mp = A.inverse()*pp;
					mr_bar = mr*(zr_star_hat/alpha_red);
					mg_bar = mg*(zr_star_hat/alpha_green);
					mc_bar = mc*(zr_star_hat/alpha_cyan);
					mp_bar = mp*(zr_star_hat/alpha_purple);
					
					tf::StampedTransform camera_wrt_world_calc_temp;
					listener.waitForTransform("world", "camera_image_calc", current_time, ros::Duration(1.0));
					listener.lookupTransform("world", "camera_image_calc", current_time, camera_wrt_world_calc_temp);
					camera_wrt_world_calc = camera_wrt_world_calc_temp;
					
					//red_wrt_world_calc = camera_wrt_world_calc*tf::Transform(camera_wrt_world_calc.getRotation().inverse(),mr_bar);
					//green_wrt_world_calc = camera_wrt_world_calc*tf::Transform(camera_wrt_world_calc.getRotation().inverse(),mg_bar);
					//cyan_wrt_world_calc = camera_wrt_world_calc*tf::Transform(camera_wrt_world_calc.getRotation().inverse(),mc_bar);
					//purple_wrt_world_calc = camera_wrt_world_calc*tf::Transform(camera_wrt_world_calc.getRotation().inverse(),mp_bar);
					
					red_wrt_camera_calc.setOrigin(mr_bar);
					green_wrt_camera_calc.setOrigin(mg_bar);
					cyan_wrt_camera_calc.setOrigin(mc_bar);
					purple_wrt_camera_calc.setOrigin(mp_bar);
					
					br.sendTransform(tf::StampedTransform(red_wrt_camera_calc, current_time, "camera_image_calc", "red_calc"));
					br.sendTransform(tf::StampedTransform(green_wrt_camera_calc, current_time, "camera_image_calc", "green_calc"));
					br.sendTransform(tf::StampedTransform(cyan_wrt_camera_calc, current_time, "camera_image_calc", "cyan_calc"));
					br.sendTransform(tf::StampedTransform(purple_wrt_camera_calc, current_time, "camera_image_calc", "purple_calc"));
					
					geometry_msgs::Pose current_out; 
					current_out.position.x = camera_wrt_world_calc.getOrigin().getX(); current_out.position.y = camera_wrt_world_calc.getOrigin().getY(); current_out.position.z = camera_wrt_world_calc.getOrigin().getZ();
					current_out.orientation.x = camera_wrt_world_calc.getRotation().getX(); current_out.orientation.y = camera_wrt_world_calc.getRotation().getY(); current_out.orientation.z = camera_wrt_world_calc.getRotation().getZ(); current_out.orientation.w = camera_wrt_world_calc.getRotation().getW();
					current_pose_pub.publish(current_out);
				}
				catch (tf::TransformException ex)
				{
					std::cout << "failed to do transform" << std::endl;
				}
				
			}
			//std::cout << "end of decomp callback" << std::endl;
			new_decomp_recieved = true;//
		}
		
		/********** callback for the pose from the mocap **********/
		void body_pose_callback(const geometry_msgs::PoseStamped& msg)
		{
			body_wrt_world_mocap.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
			body_wrt_world_mocap.setRotation(tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w));
			camera_wrt_world = body_wrt_world_mocap*camera_wrt_body;
			last_body_pose_time = msg.header.stamp;
		}
		
		///********** callback for the controller **********/
		void joy_callback(const sensor_msgs::Joy& msg)
		{
			a_button_land_b0 = msg.buttons[0];// a button lands
			
			b_button_reset_b1 = msg.buttons[1];// b button resets
			
			y_button_takeoff_b3 = msg.buttons[3];// y button takes off
				
			lb_button_teleop_b4 = msg.buttons[4];// left bumper for autonomous mode
			rb_button_teleop_b5 = msg.buttons[5];// right bumper says to start controller
			if (rb_button_teleop_b5 > 0)
			{
				try
				{
					// setting the reference
					tf::StampedTransform reference_wrt_world_temp;
					listener.waitForTransform("world", "bebop_image", last_body_pose_time, ros::Duration(1.0));
					listener.lookupTransform("world", "bebop_image", last_body_pose_time, reference_wrt_world_temp);
					reference_wrt_world = reference_wrt_world_temp;
					tf::Quaternion n_ref_4 = ((reference_wrt_world.getRotation().inverse())*tf::Quaternion(n_world.getX(),n_world.getY(),n_world.getZ(),0))*reference_wrt_world.getRotation();
					n_ref.setValue(n_ref_4.getX(), n_ref_4.getY(), n_ref_4.getZ());
					std::cout << "set reference transform" << std::endl;
					
					// setting red wrt reference
					br.sendTransform(tf::StampedTransform(red_wrt_world, last_body_pose_time, "world", "red"));
					br.sendTransform(tf::StampedTransform(reference_wrt_world, last_body_pose_time, "world", "reference_image"));
					tf::StampedTransform red_wrt_reference_temp;
					listener.waitForTransform("reference_image", "red",last_body_pose_time,ros::Duration(1.0));
					listener.lookupTransform("reference_image", "red", last_body_pose_time, red_wrt_reference_temp);
					red_wrt_reference = red_wrt_reference_temp;
					
					std::cout << "reference red z: " << red_wrt_reference.getOrigin().getZ() << std::endl;
					// setting the desired
					
					
					start_controller = true;
				}
				catch (tf::TransformException ex)
				{
					std::cout << "failed to set reference transform" << std::endl;
				}
			}
			
			// if the bumper is pulled and it is not in autonomous mode will put it in autonomous mode
			// if the bumper is pulled and it is in autonomous mode will take it out of autonomous mode
			start_autonomous = lb_button_teleop_b4 > 0;
			
			//std::cout << "controller is started: " << start_controller << std::endl;
			//std::cout << "autonomous mode is: " << start_autonomous << std::endl;
			
			rt_stick_ud_x_a3 = joy_deadband(msg.axes[3]);// right thumbstick up and down controls linear x
			
			rt_stick_lr_y_a2 = joy_deadband(msg.axes[2]);// right thumbstick left and right controls linear y
			
			lt_stick_ud_z_a1 = joy_deadband(msg.axes[1]);// left thumbstick up and down controls linear z
			
			lt_stick_lr_th_a0 = joy_deadband(msg.axes[0]);// left thumbstick left and right controls angular z
			
		}
		
		/********** deadband for the xbox controller **********/
		double joy_deadband(double input_value)
		{
			double filtered_value = 0;
			if (std::abs(input_value) > 0.11)
			{
				filtered_value = input_value;
			}
			return filtered_value;
		}
		
		/********** update the desired pixels **********/
		void update_desired_pixels()
		{	
			if (start_controller)
			{
				current_time_d = ros::Time::now();
				double time_diff = current_time_d.toSec() - last_time_d.toSec();// get the time difference
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
				tf::Quaternion Q_P_dw_dot = (Q_dw*tf::Quaternion(vcd.getX(),vcd.getY(),vcd.getZ(),0.0))*Q_dw.inverse();//express vcd in world frame
				tf::Vector3 P_dw_dot = tf::Vector3(Q_P_dw_dot.getX(),Q_P_dw_dot.getY(),Q_P_dw_dot.getZ());
				tf::Vector3 P_dw = desired_wrt_world.getOrigin();//origin of camera wrt world
				tf::Vector3 P_dw_new = P_dw + P_dw_dot*time_diff;//update origin
				desired_wrt_world.setOrigin(P_dw_new);//updating the transform
	
				//std::cout << "wcd:\n x: " << wcd.getX() << " y: " << wcd.getY() << " z: " << wcd.getZ() << std::endl;
				//std::cout << "vcd:\n x: " << vcd.getX() << " y: " << vcd.getY() << " z: " << vcd.getZ() << std::endl;
				//std::cout << "Q_dw_dot:\n x: " << Q_dw_dot.at<double>(0,0) << " y: " << Q_dw_dot.at<double>(1,0) << " z: " << Q_dw_dot.at<double>(2,0) << " w: " << Q_dw_dot.at<double>(3,0) << std::endl;
				//std::cout << "Q_dw:\n x: " << Q_dw.getX() << " y: " << Q_dw.getY() << " z: " << Q_dw.getZ() << " w: " << Q_dw.getW() << std::endl;
				//std::cout << "P_dw_dot:\n x: " << P_dw_dot.getX() << " y: " << P_dw_dot.getY() << " z: " << P_dw_dot.getZ() << std::endl;
				//std::cout << "P_dw:\n x: " << P_dw.getX() << " y: " << P_dw.getY() << " z: " << P_dw.getZ() << std::endl;
				
				/********** update pixels wrt desired  **********/
				tf::Vector3 temp_v;
				tf::Quaternion temp_Q;
				temp_v = P_red_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mrd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
				temp_v = P_green_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mgd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
				temp_v = P_cyan_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mcd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
				temp_v = P_purple_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mpd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
				prd = A*((1/mrd_bar.getZ())*mrd_bar); pgd = A*((1/mgd_bar.getZ())*mgd_bar); pcd = A*((1/mcd_bar.getZ())*mcd_bar); ppd = A*((1/mpd_bar.getZ())*mpd_bar);// camera points as pixels
				
				//std::cout << "prd:\n x: " << prd.getX() << " y: " << prd.getY() << " z: " << prd.getZ() << std::endl;
				//std::cout << "pgd:\n x: " << pgd.getX() << " y: " << pgd.getY() << " z: " << pgd.getZ() << std::endl;
				//std::cout << "pcd:\n x: " << pcd.getX() << " y: " << pcd.getY() << " z: " << pcd.getZ() << std::endl;
				//std::cout << "ppd:\n x: " << ppd.getX() << " y: " << ppd.getY() << " z: " << ppd.getZ() << std::endl;
				
				desired_pixels.header.stamp = ros::Time::now();
				desired_pixels.pr.x = prd.getX(); desired_pixels.pr.y = prd.getY(); desired_pixels.pr.z = prd.getZ();//red
				desired_pixels.pg.x = pgd.getX(); desired_pixels.pg.y = pgd.getY(); desired_pixels.pg.z = pgd.getZ();//green
				desired_pixels.pc.x = pcd.getX(); desired_pixels.pc.y = pcd.getY(); desired_pixels.pc.z = pcd.getZ();//cyan
				desired_pixels.pp.x = ppd.getX(); desired_pixels.pp.y = ppd.getY(); desired_pixels.pp.z = ppd.getZ();//purple
				
				desired_pixels_pub.publish(desired_pixels);
				
				/********* desired wrt world *********/
				br.sendTransform(tf::StampedTransform(desired_wrt_world, current_time_d, "world", "desired_image"));
			
			
				/********* desired wrt reference *********/
				br.sendTransform(tf::StampedTransform(reference_wrt_world, current_time_d, "world", "reference_image"));//update the reference
				tf::StampedTransform desired_wrt_reference;
				listener.waitForTransform("reference_image", "desired_image", current_time_d, ros::Duration(1.0));
				listener.lookupTransform("reference_image", "desired_image", current_time_d, desired_wrt_reference);
				Q_df = desired_wrt_reference.getRotation();
				
				if (first_desired_recieved)
				{
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
				}
				else 
				{
					first_desired_recieved = true;
					//tf::Quaternion mi_temp;
					// set the reference values for alpha fake
					//mi_temp = (desired_wrt_reference.getRotation()*tf::Quaternion(mrd_bar.getX(),mrd_bar.getY(),mrd_bar.getZ(),0))*(desired_wrt_reference.getRotation().inverse());
					//mr_bar_ref.setValue(mi_temp.getX()+desired_wrt_reference.getOrigin().getX(), mi_temp.getY()+desired_wrt_reference.getOrigin().getY(), mi_temp.getZ()+desired_wrt_reference.getOrigin().getZ());//red
					//mi_temp = (desired_wrt_reference.getRotation()*tf::Quaternion(mgd_bar.getX(),mgd_bar.getY(),mgd_bar.getZ(),0))*(desired_wrt_reference.getRotation().inverse());
					//mg_bar_ref.setValue(mi_temp.getX()+desired_wrt_reference.getOrigin().getX(), mi_temp.getY()+desired_wrt_reference.getOrigin().getY(), mi_temp.getZ()+desired_wrt_reference.getOrigin().getZ());//green
					//mi_temp = (desired_wrt_reference.getRotation()*tf::Quaternion(mcd_bar.getX(),mcd_bar.getY(),mcd_bar.getZ(),0))*(desired_wrt_reference.getRotation().inverse());
					//mc_bar_ref.setValue(mi_temp.getX()+desired_wrt_reference.getOrigin().getX(), mi_temp.getY()+desired_wrt_reference.getOrigin().getY(), mi_temp.getZ()+desired_wrt_reference.getOrigin().getZ());//cyan
					//mi_temp = (desired_wrt_reference.getRotation()*tf::Quaternion(mpd_bar.getX(),mpd_bar.getY(),mpd_bar.getZ(),0))*(desired_wrt_reference.getRotation().inverse());
					//mp_bar_ref.setValue(mi_temp.getX()+desired_wrt_reference.getOrigin().getX(), mi_temp.getY()+desired_wrt_reference.getOrigin().getY(), mi_temp.getZ()+desired_wrt_reference.getOrigin().getZ());//purple
					
					 
				}
				alpha_red_d = red_wrt_reference.getOrigin().getZ()/mrd_bar.getZ();
				//std::cout << "reference red z: " << red_wrt_reference.getOrigin().getZ() << std::endl;
				//std::cout << "desired red z: " << mrd_bar.getZ() << std::endl;
				//std::cout << "alpha red desired: " << alpha_red_d << std::endl;
				Q_df_last = Q_df;// updating the last
				desired_wrt_reference.setRotation(Q_df);// updating the desired_wrt_reference
				
				red_wrt_des_camera_calc.setOrigin(mrd_bar);
				green_wrt_des_camera_calc.setOrigin(mgd_bar);
				cyan_wrt_des_camera_calc.setOrigin(mcd_bar);
				purple_wrt_des_camera_calc.setOrigin(mpd_bar);
				
				br.sendTransform(tf::StampedTransform(red_wrt_des_camera_calc, current_time, "desired_image", "red_wrt_des"));
				br.sendTransform(tf::StampedTransform(green_wrt_des_camera_calc, current_time, "desired_image", "green_wrt_des"));
				br.sendTransform(tf::StampedTransform(cyan_wrt_des_camera_calc, current_time, "desired_image", "cyan_wrt_des"));
				br.sendTransform(tf::StampedTransform(purple_wrt_des_camera_calc, current_time, "desired_image", "purple_wrt_des"));
			}
			
			last_time_d = current_time_d;
			geometry_msgs::Pose desired_out;		
			desired_out.position.x = desired_wrt_world.getOrigin().getX(); desired_out.position.y = desired_wrt_world.getOrigin().getY(); desired_out.position.z = desired_wrt_world.getOrigin().getZ();
			desired_out.orientation.x = desired_wrt_world.getRotation().getX(); desired_out.orientation.y = desired_wrt_world.getRotation().getY(); desired_out.orientation.z = desired_wrt_world.getRotation().getZ(); desired_out.orientation.w = desired_wrt_world.getRotation().getW();
			//desired_out.position.x = desired_wrt_reference.getOrigin().getX(); desired_out.position.y = desired_wrt_reference.getOrigin().getY(); desired_out.position.z = desired_wrt_reference.getOrigin().getZ();
			//desired_out.orientation.x = desired_wrt_reference.getRotation().getX(); desired_out.orientation.y = desired_wrt_reference.getRotation().getY(); desired_out.orientation.z = desired_wrt_reference.getRotation().getZ(); desired_out.orientation.w = desired_wrt_reference.getRotation().getW();
			desired_pose_pub.publish(desired_out);
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
			std::cout << "Qerror:\n x: " << Q_error.getX() << " y: " << Q_error.getY() << " z: " << Q_error.getZ() << " w: " << Q_error.getW() << std::endl;
			tf::Vector3 Qerror_ijk_temp(Qerror_temp.getX(),Qerror_temp.getY(),Qerror_temp.getZ());// temp to hold the rotational error ijk terms
			tf::Quaternion Qwcd_temp(wcd.getX(),wcd.getY(),wcd.getZ(),0.0);// getting the desired angular velocity as a quaternion to use for the camera angular velocity command
			tf::Vector3 wc_term1_temp;
			wc_term1_temp = -1*(Kw*Qerror_ijk_temp); // getting the first term of the wc calc
			std::cout << "wc term 1:\n x: " << wc_term1_temp.getX() << " y: " << wc_term1_temp.getY() << " z: " << wc_term1_temp.getZ() << std::endl;
			tf::Quaternion Qwc_term2_temp = (Qerror_temp.inverse()*Qwcd_temp)*Qerror_temp;// calculating the double product for the reporjection of wcd as a quaternion
			tf::Vector3 wc_term2_temp(Qwc_term2_temp.getX(), Qwc_term2_temp.getY(), Qwc_term2_temp.getZ());// getting the second term as a vector		  
			std::cout << "wc term 2:\n x: " << wc_term2_temp.getX() << " y: " << wc_term2_temp.getY() << " z: " << wc_term2_temp.getZ() << std::endl;
			tf::Vector3 wc_temp = wc_term1_temp + wc_term2_temp;
			wc = wc_temp;// update the velocity
			std::cout << "wc:\n x: " << wc.getX() << " y: " << wc.getY() << " z: " << wc.getZ() << std::endl;
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
			tf::Vector3 vc_term2_temp = (1/alpha_red)*(Lv.inverse()*(phi*red_wrt_reference.getOrigin().getZ()));// term 2 for the vc calculation
			//tf::Vector3 vc_term2_temp = (1/alpha_red)*(Lv.inverse()*(phi*red_wrt_reference.getOrigin().getZ()));// term 2 for the vc calculation
			//std::cout << "alpha red: " << alpha_red << std::endl;
			//std::cout << "pr:\n x: " << pr.getX() << " y: " << pr.getY() << " z: " << pr.getZ() << std::endl;
			//std::cout << "A inverse:"
					  //<< "\n row 0:"
					  //<< " x: " << A.inverse().getRow(0).getX()
					  //<< " y: " << A.inverse().getRow(0).getY()
					  //<< " z: " << A.inverse().getRow(0).getZ()
					  //<< "\n row 1:"
					  //<< " x: " << A.inverse().getRow(1).getX()
					  //<< " y: " << A.inverse().getRow(1).getY()
					  //<< " z: " << A.inverse().getRow(1).getZ()
					  //<< "\n row 2:"
					  //<< " x: " << A.inverse().getRow(2).getX()
					  //<< " y: " << A.inverse().getRow(2).getY()
					  //<< " z: " << A.inverse().getRow(2).getZ()
					  //<<std::endl;
			//std::cout << "mr:\n x: " << mr.getX() << " y: " << mr.getY() << " z: " << mr.getZ() << std::endl;
			//std::cout << "A - Principle Point mat for Lv:"
					  //<< "\n row 0:"
					  //<< " x: " << Lv_term1.getRow(0).getX()
					  //<< " y: " << Lv_term1.getRow(0).getY()
					  //<< " z: " << Lv_term1.getRow(0).getZ()
					  //<< "\n row 1:"
					  //<< " x: " << Lv_term1.getRow(1).getX()
					  //<< " y: " << Lv_term1.getRow(1).getY()
					  //<< " z: " << Lv_term1.getRow(1).getZ()
					  //<< "\n row 2:"
					  //<< " x: " << Lv_term1.getRow(2).getX()
					  //<< " y: " << Lv_term1.getRow(2).getY()
					  //<< " z: " << Lv_term1.getRow(2).getZ()
					  //<<std::endl;
			//std::cout << "Current mr mat for Lv:"
					  //<< "\n row 0:"
					  //<< " x: " << mr_mat.getRow(0).getX()
					  //<< " y: " << mr_mat.getRow(0).getY()
					  //<< " z: " << mr_mat.getRow(0).getZ()
					  //<< "\n row 1:"
					  //<< " x: " << mr_mat.getRow(1).getX()
					  //<< " y: " << mr_mat.getRow(1).getY()
					  //<< " z: " << mr_mat.getRow(1).getZ()
					  //<< "\n row 2:"
					  //<< " x: " << mr_mat.getRow(2).getX()
					  //<< " y: " << mr_mat.getRow(2).getY()
					  //<< " z: " << mr_mat.getRow(2).getZ()
					  //<<std::endl;
			//std::cout << "Current Lv:"
					  //<< "\n row 0:"
					  //<< " x: " << Lv.getRow(0).getX()
					  //<< " y: " << Lv.getRow(0).getY()
					  //<< " z: " << Lv.getRow(0).getZ()
					  //<< "\n row 1:"
					  //<< " x: " << Lv.getRow(1).getX()
					  //<< " y: " << Lv.getRow(1).getY()
					  //<< " z: " << Lv.getRow(1).getZ()
					  //<< "\n row 2:"
					  //<< " x: " << Lv.getRow(2).getX()
					  //<< " y: " << Lv.getRow(2).getY()
					  //<< " z: " << Lv.getRow(2).getZ()
					  //<<std::endl;
			//std::cout << "Current Lv inverse:"
					  //<< "\n row 0:"
					  //<< " x: " << Lv.inverse().getRow(0).getX()
					  //<< " y: " << Lv.inverse().getRow(0).getY()
					  //<< " z: " << Lv.inverse().getRow(0).getZ()
					  //<< "\n row 1:"
					  //<< " x: " << Lv.inverse().getRow(1).getX()
					  //<< " y: " << Lv.inverse().getRow(1).getY()
					  //<< " z: " << Lv.inverse().getRow(1).getZ()
					  //<< "\n row 2:"
					  //<< " x: " << Lv.inverse().getRow(2).getX()
					  //<< " y: " << Lv.inverse().getRow(2).getY()
					  //<< " z: " << Lv.inverse().getRow(2).getZ()
					  //<<std::endl;
			//std::cout << "Kv:"
					  //<< "\n row 0:"
					  //<< " x: " << Kv.getRow(0).getX()
					  //<< " y: " << Kv.getRow(0).getY()
					  //<< " z: " << Kv.getRow(0).getZ()
					  //<< "\n row 1:"
					  //<< " x: " << Kv.getRow(1).getX()
					  //<< " y: " << Kv.getRow(1).getY()
					  //<< " z: " << Kv.getRow(1).getZ()
					  //<< "\n row 2:"
					  //<< " x: " << Kv.getRow(2).getX()
					  //<< " y: " << Kv.getRow(2).getY()
					  //<< " z: " << Kv.getRow(2).getZ()
					  //<<std::endl;
			std::cout << "pe:\n x: " << pe.getX() << " y: " << pe.getY() << " z: " << pe.getZ() << std::endl;
			std::cout << "ped:\n x: " << ped.getX() << " y: " << ped.getY() << " z: " << ped.getZ() << std::endl;
			std::cout << "ev:\n x: " << ev.getX() << " y: " << ev.getY() << " z: " << ev.getZ() << std::endl;
			std::cout << "phi:\n x: " << phi.getX() << " y: " << phi.getY() << " z: " << phi.getZ() << std::endl;
			std::cout << "zr star hat:\n x: " << zr_star_hat << std::endl;
			std::cout << "vc term1:\n x: " << vc_term1_temp.getX() << " y: " << vc_term1_temp.getY() << " z: " << vc_term1_temp.getZ() << std::endl;
			std::cout << "vc term2:\n x: " << vc_term2_temp.getX() << " y: " << vc_term2_temp.getY() << " z: " << vc_term2_temp.getZ() << std::endl;
			
			tf::Vector3 vc_temp = vc_term1_temp + vc_term2_temp;// sum them together for vc
			vc = vc_temp;
			
		}
		/********** update the pixels and everything with them **********/
		void update_pixels()
		{
			tf::Matrix3x3 ss_mr_temp(           0, -1*mr.getZ(),    mr.getY(),
									    mr.getZ(),            0, -1*mr.getX(),
									 -1*mr.getY(),    mr.getX(),           0);
			ss_mr = ss_mr_temp;// skew symmetrix mr
			tf::Vector3 mrd_temp = A.inverse()*prd; 
			mrd = mrd_temp;// desried feature point normalized euclidean position
			tf::Matrix3x3 ss_mrd_temp(            0,    -1*mrd.getZ(),    mrd.getY(),
										 mrd.getZ(),                0, -1*mrd.getX(),
									  -1*mrd.getY(),       mrd.getX(),            0);
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
				std::cout << "wc:\n x: " << wc.getX() << " y: " << wc.getY() << " z: " << wc.getZ() << std::endl;
				std::cout << "phi term 1:\n x: " << phi_term1.getX() << " y: " << phi_term1.getY() << " z: " << phi_term1.getZ() << std::endl;
				std::cout << "phi term 2, ped_dot:\n x: " << ped_dot.getX() << " y: " << ped_dot.getY() << " z: " << ped_dot.getZ() << std::endl;
		}
		
		/********** calculating the value for ped_dot **********/
		void get_ped_dot()
		{
			tf::Vector3 ped_dot_term1_temp = (-1*alpha_red_d/red_wrt_reference.getOrigin().getZ())*(Lvd*vcd);// getting term 1 for the ped_dot equation
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
			std::cout << "time from start: " << current_time.toSec() - start_time.toSec() << std::endl;
			std::cout << "zr star hat before: " << zr_star_hat << std::endl;
			zr_star_hat += zr_star_hat_dot*time_from_last;
			std::cout << "zr star hat: " << zr_star_hat << std::endl;
			std::cout << "zr star: " << red_wrt_reference.getOrigin().getZ() << std::endl;
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
			//zr_star_hat_dot = gamma_1*ev.dot(phi) + gamma_1*gamma_2*zr_star_hat_dot_sum;
			/*****************************************************************************************************************************************************************************************/
			zr_star_hat_dot = gamma_1*gamma_2*zr_star_hat_dot_sum;
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
					temp_integral += ((time_values.at(ii+1) - time_values.at(ii)) * (function_values.at(ii+1) + function_values.at(ii)));
				}
			}
			integral = 0.5*temp_integral;
		}
		
		/********** velocity command **********/
		void output_velocity_command()
		{
			if (alpha_red > 0 && start_controller)
			{
				generate_velocity_command_from_tracking();
				last_time = current_time;
				geometry_msgs::Pose error_out;
				//error_out.header.stamp = ros::Time::now();
				error_out.position.x = ev.getX(); error_out.position.y = ev.getY(); error_out.position.z = ev.getZ(); 
				error_out.orientation.x = Q_error.getX(); error_out.orientation.y = Q_error.getY(); error_out.orientation.z = Q_error.getZ(); error_out.orientation.w = Q_error.getW(); 
				current_error_pub.publish(error_out);
				double scale = 1;
				vc_marker_end.x = scale*vc.getX(); vc_marker_end.y = scale*vc.getY(); vc_marker_end.z = scale*vc.getZ();
				vc_marker.points.clear();
				vc_marker.points.push_back(vc_marker_start);
				vc_marker.points.push_back(vc_marker_end);
				wc_marker_end.x = scale*wc.getX(); wc_marker_end.y = scale*wc.getY(); wc_marker_end.z = scale*wc.getZ();
				wc_marker.points.clear();
				wc_marker.points.push_back(wc_marker_start);
				wc_marker.points.push_back(wc_marker_end);
				
			}
			
			if (start_autonomous && start_controller)
			{
				std::cout << "command from controller" << std::endl;
				std::cout << "vc:\n x: " << vc.getX() << " y: " << vc.getY() << " z: " << vc.getZ() << std::endl;
				std::cout << "wc:\n x: " << wc.getX() << " y: " << wc.getY() << " z: " << wc.getZ() << std::endl;
				std::cout << "w_body:\n x: " << w_body.getX() << " y: " << w_body.getY() << " z: " << w_body.getZ() << std::endl;
				std::cout << "alpha_red: " << alpha_red  << std::endl;
				std::cout << "alpha_green: " << alpha_green  << std::endl;
				std::cout << "alpha_cyan: " << alpha_cyan  << std::endl;
				std::cout << "alpha_purple: " << alpha_purple  << std::endl;
				if (!std::isnan(vc.getX()) && !std::isnan(vc.getY()) && !std::isnan(vc.getY()) && !std::isnan(wc.getZ()) && alpha_red > 0 && alpha_green > 0 && alpha_cyan > 0 && alpha_purple > 0)
				{	
					// output linear velocity is q_cam_wrt_body * vc * q_cam_wrt_body.inverse() - w_body_wrt_world X p_cam_wrt_body
					tf::Quaternion vc_body_term1 = (camera_wrt_body.getRotation() * tf::Quaternion(vc.getX(), vc.getY(), vc.getZ(), 0)) * (camera_wrt_body.getRotation().inverse());
					tf::Vector3 vc_body_term2 = w_body.cross(camera_wrt_body.getOrigin());
					velocity_command.linear.x = vc_body_term1.getX() - vc_body_term2.getX();
					velocity_command.linear.y = vc_body_term1.getY() - vc_body_term2.getY();
					velocity_command.linear.z = vc_body_term1.getZ() - vc_body_term2.getZ();
				
					// output angular velocity is q_cam_wrt_body * wc * q_cam_wrt_body.inverse() - (q_cam_wrt_body * w_image * q_cam_wrt_body.inverse() - w_body)
					tf::Quaternion wc_body_term1 = (camera_wrt_body.getRotation() * tf::Quaternion(wc.getX(), wc.getY(), wc.getZ(), 0)) * (camera_wrt_body.getRotation().inverse());
					tf::Quaternion wc_body_term2 = (camera_wrt_body.getRotation() * tf::Quaternion(w_image.getX(), w_image.getY(), w_image.getZ(), 0)) * (camera_wrt_body.getRotation().inverse());
					velocity_command.angular.z = wc_body_term1.getZ() - (wc_body_term2.getZ() - w_body.getZ());
					
					std::cout << "none are nan" << std::endl;
					std::cout << "vc_body_term1:\n x: " << vc_body_term1.getX() << " y: " << vc_body_term1.getY() << " z: " << vc_body_term1.getZ() << " w: " << vc_body_term1.getW() << std::endl;
					std::cout << "vc_body_term2:\n x: " << vc_body_term2.getX() << " y: " << vc_body_term2.getY() << " z: " << vc_body_term2.getZ() << std::endl;
					std::cout << "vc_body:\n x: " << velocity_command.linear.x << " y: " << velocity_command.linear.y << " z: " << velocity_command.linear.z << std::endl;
					std::cout << "wc_body_term1:\n x: " << wc_body_term1.getX() << " y: " << wc_body_term1.getY() << " z: " << wc_body_term1.getZ() << std::endl;
					std::cout << "wc_body_term1:\n x: " << wc_body_term2.getX() << " y: " << wc_body_term2.getY() << " z: " << wc_body_term2.getZ() << std::endl;
					std::cout << "wc_body_term1:\n x: " << w_body.getX() << " y: " << w_body.getY() << " z: " << w_body.getZ() << std::endl;
					std::cout << "wc_body:\n z: " << velocity_command.angular.z << std::endl;
				}
				else
				{
					velocity_command.linear.x = 0;
					velocity_command.linear.y = 0;
					velocity_command.linear.z = 0;
					velocity_command.angular.z = 0;
				}
				std::cout << "linear x: " << velocity_command.linear.x << std::endl;
				std::cout << "linear y: " << velocity_command.linear.y << std::endl;
				std::cout << "linear z: " << velocity_command.linear.z << std::endl;
				std::cout << "angular z: " << velocity_command.angular.z << std::endl;
				cmd_vel_pub.publish(velocity_command);
				
			}
			
			if (write_to_file)
			{
				output_file.open(output_file_name, std::fstream::out | std::fstream::app);
			}
			if (output_file.is_open())
			{
				output_file  << current_time.toSec() - start_time.toSec() << "," << lb_button_teleop_b4 << ","
				<< pr.getX() << "," << pr.getY() << "," << pr.getZ() << ","
				<< prd.getX() << "," << prd.getY() << "," << prd.getZ() << ","
				<< mr.getX() << "," << mr.getY() << "," << mr.getZ() << ","
				<< mrd.getX() << "," << mrd.getY() << "," << mrd.getZ() << ","
				<< red_wrt_world_calc.getOrigin().getX() << "," << red_wrt_world_calc.getOrigin().getY() << "," << red_wrt_world_calc.getOrigin().getZ() << ","
				<< green_wrt_world_calc.getOrigin().getX() << "," << green_wrt_world_calc.getOrigin().getY() << "," << green_wrt_world_calc.getOrigin().getZ() << ","
				<< cyan_wrt_world_calc.getOrigin().getX() << "," << cyan_wrt_world_calc.getOrigin().getY() << "," << cyan_wrt_world_calc.getOrigin().getZ() << ","
				<< purple_wrt_world_calc.getOrigin().getX() << "," << purple_wrt_world_calc.getOrigin().getY() << "," << purple_wrt_world_calc.getOrigin().getZ() << ","
				<< red_wrt_world.getOrigin().getX() << "," << red_wrt_world.getOrigin().getY() << "," << red_wrt_world.getOrigin().getZ() << ","
				<< green_wrt_world.getOrigin().getX() << "," << green_wrt_world.getOrigin().getY() << "," << green_wrt_world.getOrigin().getZ() << ","
				<< cyan_wrt_world.getOrigin().getX() << "," << cyan_wrt_world.getOrigin().getY() << "," << cyan_wrt_world.getOrigin().getZ() << ","
				<< purple_wrt_world.getOrigin().getX() << "," << purple_wrt_world.getOrigin().getY() << "," << purple_wrt_world.getOrigin().getZ() << ","
				<< desired_wrt_world.getOrigin().getX() << "," << desired_wrt_world.getOrigin().getY() << "," << desired_wrt_world.getOrigin().getZ() << ","
				<< camera_wrt_world_calc.getOrigin().getX() << "," << camera_wrt_world_calc.getOrigin().getY() << "," << camera_wrt_world_calc.getOrigin().getZ() << ","
				<< camera_wrt_world.getOrigin().getX() << "," << camera_wrt_world.getOrigin().getY() << "," << camera_wrt_world.getOrigin().getZ() << ","
				<< desired_wrt_world.getRotation().getW() << "," << desired_wrt_world.getRotation().getX() << "," << desired_wrt_world.getRotation().getY() << "," << desired_wrt_world.getRotation().getZ() << ","
				<< camera_wrt_world_calc.getRotation().getW() << "," << camera_wrt_world_calc.getRotation().getX() << "," << camera_wrt_world_calc.getRotation().getY() << "," << camera_wrt_world_calc.getRotation().getZ() << ","
				<< camera_wrt_world.getRotation().getW() << "," << camera_wrt_world.getRotation().getX() << "," << camera_wrt_world.getRotation().getY() << "," << camera_wrt_world.getRotation().getZ() << ","
				
				<< body_wrt_world_mocap.getOrigin().getX() << "," << body_wrt_world_mocap.getOrigin().getY() << "," << body_wrt_world_mocap.getOrigin().getZ() << ","
				
				<< body_wrt_world_mocap.getRotation().getW() << "," << body_wrt_world_mocap.getRotation().getX() << "," << body_wrt_world_mocap.getRotation().getY() << "," << body_wrt_world_mocap.getRotation().getZ() << ","
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
				<< w_body.getX() << "," << w_body.getY() << "," << w_body.getZ() << ","
				<< velocity_command.linear.x << "," << velocity_command.linear.y << "," << velocity_command.linear.z << ","
				<< velocity_command.angular.x << "," << velocity_command.angular.y << "," << velocity_command.angular.z << ","
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

};

// main
int main(int argc, char** argv)
{   
	ros::init(argc,argv,"experiment_node");

	double loop_rate_hz = 30;
	bool write_to_file = true;
	
	std::string filename = "/home/ncr/ncr_ws/src/homog_track/testing_files/experiment_8.txt";
	if( (std::remove( filename.c_str() ) != 0) && write_to_file)
	{
		std::cout << "file does not exist" << std::endl;
	}
	else
	{
		std::cout << "file deleted" << std::endl;
	}
	std::cout << "hi" << std::endl;
	
	ImageProcessing image_processing;// image processing
	HomogDecomp homog_decomp;// homography decomp
	Controller controller(loop_rate_hz, write_to_file, filename);// controller
	
	ros::Rate loop_rate(300);
	
	while (ros::ok())
	{
		controller.update_desired_pixels();
		if (controller.new_decomp_recieved)
		{
			controller.output_velocity_command();
			controller.new_decomp_recieved = false;
			
		}
		controller.vc_marker_pub.publish(controller.vc_marker);
		controller.wc_marker_pub.publish(controller.wc_marker);
		ros::spinOnce();
		loop_rate.sleep();
	}
	//ros::spin();

    return 0;
}
