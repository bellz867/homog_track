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
		float low_pass_gain = 0.75;// tuning gain for the low pass
		bool first_run = false;// boolean to tell if this is the first run
		homog_track::ImageProcessingMsg pixels_out;//message
		bool camera_updated = false;
		
		/********** threshold declarations and initializations **********/
		// order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
		int r_thresh[6] = {165, 5, 60, 255, 85, 255};// values for the red color threshold values: hue, staturation, and value
		int g_thresh[6] = {65, 90, 45, 255, 40, 255};// values for the green color threshold values: hue, staturation, and value
		int c_thresh[6] = {95, 105, 125, 255, 80, 255};// values for the cyan color threshold values: hue, staturation, and value
		int p_thresh[6] = {110, 135, 45, 255, 50, 255};// values for the violet color threshold values: hue, staturation, and value
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
			image_sub_ = it_.subscribe( "/ardrone/front/image_raw", 1, &ImageProcessing::image_callback, this);// subscribe to ardrone front camera
			image_pub_ = it_.advertise("processed_image", 1);// publish processed image
			pixel_pub = nh.advertise<homog_track::ImageProcessingMsg>("feature_pixels", 1);//pixel publisher
			A = cv::Mat::zeros(3,3,CV_64F); A.at<double>(0,0) = 567.79; A.at<double>(0,2) = 337.35; A.at<double>(1,1) = 564.52; A.at<double>(1,2) = 169.54; A.at<double>(2,2) = 1; // camera matrix for the ardrone
			dC.push_back(-0.5122398601387984); dC.push_back(0.2625218940944695); dC.push_back(-0.0009892579344331395); dC.push_back(0.002480111502028633); dC.push_back(0.0); // distortion matrix for the ardrone
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
					std::vector<cv::Point2f> center( contours[ii].size() );// declaring centers to be all zeros the size of contours to hold all the contours circles centers
					std::vector<float> radius( contours[ii].size() );// declaring radius to be all zeros the size of contours to hold all the contours circles radius
					// getting the minimum enclcosing circle for the contours
					for (int jj = 0; jj < contours[ii].size(); jj++)
					{
						minEnclosingCircle( contours[ii][jj], center[jj], radius[jj] );// getting the circle for the current contour
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
							red_circle_p.z = low_pass_gain*red_circle_p_curr.z + (1 - low_pass_gain)*red_circle_p_last.z;
							green_circle_p.x = low_pass_gain*green_circle_p_curr.x + (1 - low_pass_gain)*green_circle_p_last.x;
							green_circle_p.y = low_pass_gain*green_circle_p_curr.y + (1 - low_pass_gain)*green_circle_p_last.y;
							green_circle_p.z = low_pass_gain*green_circle_p_curr.z + (1 - low_pass_gain)*green_circle_p_last.z;
							cyan_circle_p.x = low_pass_gain*cyan_circle_p_curr.x + (1 - low_pass_gain)*cyan_circle_p_last.x;
							cyan_circle_p.y = low_pass_gain*cyan_circle_p_curr.y + (1 - low_pass_gain)*cyan_circle_p_last.y;
							cyan_circle_p.z = low_pass_gain*cyan_circle_p_curr.z + (1 - low_pass_gain)*cyan_circle_p_last.z;
							purple_circle_p.x = low_pass_gain*purple_circle_p_curr.x + (1 - low_pass_gain)*purple_circle_p_last.x;
							purple_circle_p.y = low_pass_gain*purple_circle_p_curr.y + (1 - low_pass_gain)*purple_circle_p_last.y;
							purple_circle_p.z = low_pass_gain*purple_circle_p_curr.z + (1 - low_pass_gain)*purple_circle_p_last.z;

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
								// assigning the values of the circle to the circle
								purple_circle_p.x = -1;
								purple_circle_p.y = -1;
								purple_circle_p.z = -1;
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

/********** Processes incoming images and outputs the pixels **********/
class SimulatedImageProcessing
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
		tf::Matrix3x3 A = tf::Matrix3x3(567.79, 0, 337.35,
										0, 564.52, 169.54,
										0,0,1);// camera matrix
										
		/********** Velocity Commands **********/
		tf::Vector3 vc, wc;//linear and angular velocity commands
		cv::Mat wc_cv;
		
		SimulatedImageProcessing(double loop_rate_des)
		{
			loop_rate_hz = loop_rate_des;
			/********** topics **********/
			pixel_pub = nh.advertise<homog_track::ImageProcessingMsg>("feature_pixels", 1);
			cam_vel_sub = nh.subscribe("/cmd_vel", 1, &SimulatedImageProcessing::update_camera_pixels, this);
			
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
		tf::Matrix3x3 A_tf = tf::Matrix3x3(567.79, 0, 337.35,
										   0, 564.52, 169.54,
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
			A = cv::Mat::zeros(3,3,CV_64F); A.at<double>(0,0) = 567.79; A.at<double>(0,2) = 337.35; A.at<double>(1,1) = 564.52; A.at<double>(1,2) = 169.54; A.at<double>(2,2) = 1; // camera matrix for the ardrone
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
		ros::Subscriber decomp_sub, joy_sub, body_imu_sub;
		ros::Publisher takeoff_pub, land_pub, reset_pub, cmd_vel_pub;
		
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
		double Kws = 1;// K_w scalar
		tf::Matrix3x3 Kw = tf::Matrix3x3(Kws,0,0,
										 0,Kws,0,
										 0,0,Kws);// rotational gain matrix initialize to identity
		double Kvs = 0.1;
		tf::Matrix3x3 Kv = tf::Matrix3x3(Kvs,0,0,
										 0,Kvs,0,
										 0,0,Kvs);// linear velocity gain matrix
		double gamma_1 = std::pow(10,-2);// control gains for the z_star_hat_dot calculation
		double gamma_2 = std::pow(10,-4);
		//double gamma_1 = 0;
		//double gamma_2 = 0;
		double zr_star_hat = 2;// current value for zr_star_hat
		
		/********** Points **********/
		tf::Vector3 P_red_wrt_world, P_green_wrt_world, P_cyan_wrt_world, P_purple_wrt_world;// homography points wrt world
		tf::Transform red_wrt_world, green_wrt_world, cyan_wrt_world, purple_wrt_world;//transforms for the marker points
		
		/********** UAV body **********/
		tf::Transform camera_wrt_body;//transform of the camera wrt the body
		tf::Vector3 w_body = tf::Vector3(0,0,0);// angular velocity of the body wrt world expressed in the body, from imu
		
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
		tf::Matrix3x3 A = tf::Matrix3x3(567.79, 0, 337.35,
										0, 564.52, 169.54,
										0, 0, 1);// camera matrix// camera matrix
										
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
		bool start_controller = false;// start the controller when the right bumper is pressed
		bool start_autonomous = false;
		bool new_decomp_recieved = false;
		bool new_joy_recieved = false;
		/********** output command choice and xbox controller **********/
		geometry_msgs::Twist velocity_command;
		geometry_msgs::Twist command_from_xbox;
		int a_button_land_b0 = 0, b_button_reset_b1 = 0, y_button_takeoff_b3 = 0, lb_button_teleop_b4 = 0, rb_button_teleop_b5 = 0;
		double rt_stick_ud_x_a3 = 0, rt_stick_lr_y_a2 = 0, lt_stick_ud_z_a1 = 0, lt_stick_lr_th_a0 = 0;
		
		Controller(double loop_rate_des, bool write_to_file_des, std::string& output_file_name_des)
		{	
			output_file_name = output_file_name_des;
			//std::cout << "output_file_name: " << output_file_name << std::endl;
			write_to_file = write_to_file_des;
			loop_rate_hz = loop_rate_des;
			number_of_integrating_samples = loop_rate_hz*0.2;
			decomp_sub = nh.subscribe("decomposed_homography",1, &Controller::decomp_callback, this);// subscribing to the decomp message
			body_imu_sub = nh.subscribe("/ardrone/imu", 1, &Controller::body_imu_callback, this);// subscribing to the imu
			cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);// publisher for the decomposed stuff
			takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff",1);
			land_pub = nh.advertise<std_msgs::Empty>("ardrone/land",1);
			reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset",1);
			joy_sub = nh.subscribe("joy",1,&Controller::joy_callback,this);
			
			cmd_vel_pub.publish(geometry_msgs::Twist());// initially sending it a desired command of 0
			/********** Set transform for camera wrt frame **********/
			camera_wrt_body.setOrigin(tf::Vector3(0.147307457005166, 0.006462951419338, -0.037134150975938));//origin
			camera_wrt_body.setRotation(tf::Quaternion(-0.686755898248694, 0.670811209506236, -0.165804584561225, 0.225439733875439));
			
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
			
			
			/********** markers wrt world **********/
			P_red_wrt_world = tf::Vector3(-0.05,-0.05,0); P_green_wrt_world = tf::Vector3(-0.05,0.05,0); P_cyan_wrt_world = tf::Vector3(0.05,0.05,0); P_purple_wrt_world = tf::Vector3(0.05,-0.05,0);// homography points wrt world
			red_wrt_world.setIdentity(); red_wrt_world.setOrigin(P_red_wrt_world);//red
			green_wrt_world.setIdentity(); green_wrt_world.setOrigin(P_green_wrt_world);//green
			cyan_wrt_world.setIdentity(); cyan_wrt_world.setOrigin(P_cyan_wrt_world);//cyan
			purple_wrt_world.setIdentity(); purple_wrt_world.setOrigin(P_purple_wrt_world);//purple

			/********** reference wrt world  **********/
			double z_ref = 2; //reference height
			reference_wrt_world.setOrigin(tf::Vector3(0,0,z_ref));//origin
			tf::Matrix3x3 R_fw(1,0,0,
							   0,-1,0,
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
			tf::Matrix3x3 R_fake_drone_wrt_w(-1,0,0,
											  0,-1,0,
											  0,0,1);
			tf::Quaternion Q_dw;// as a quaternion
			R_fake_drone_wrt_w.getRotation(Q_dw);// initialize quaternion
			desired_wrt_world.setRotation(Q_dw*camera_wrt_body.getRotation());// set the rotation
			wcd_cv = cv::Mat::zeros(3,1,CV_64F);
			//tf::Vector3 wcd_temp = tf::Vector3(0,0,0);
			tf::Vector3 wcd_temp = tf::Vector3(0,0,2*M_PIl/desired_period);
			tf::Vector3 vcd_temp = tf::Vector3(0,-1*wcd_temp.getZ()*desired_radius,0);
			
			// vcd is q_cam_wrt_body * vcd_temp * q_cam_wrt_body.inverse(), dont need the cross because there is no virtual drone for the vector
			tf::Quaternion vcd_body_temp = ((camera_wrt_body.getRotation().inverse())*tf::Quaternion(vcd_temp.getX(), vcd_temp.getY(), vcd_temp.getZ(), 0)) * camera_wrt_body.getRotation();
			vcd = tf::Vector3(vcd_body_temp.getX(), vcd_body_temp.getY(), vcd_body_temp.getZ());
			// wcd is q_cam_wrt_body * wcd_temp * q_cam_wrt_body.inverse()
			tf::Quaternion wcd = ((camera_wrt_body.getRotation().inverse()) * tf::Quaternion(wcd_temp.getX(), wcd_temp.getY(), wcd_temp.getZ(), 0)) * camera_wrt_body.getRotation();
			wcd_cv.at<double>(0,0) = wcd.getX(); wcd_cv.at<double>(1,0) = wcd.getY(); wcd_cv.at<double>(2,0) = wcd.getZ(); 
			
			//std::cout << "wcd:/n x: " << wcd.getX() << " y: " << wcd.getY() << " z: " << wcd.getZ() << std::endl;
			//std::cout << "vcd:/n x: " << vcd.getX() << " y: " << vcd.getY() << " z: " << vcd.getZ() << std::endl;  
			
			/********** pixels wrt desired  **********/
			temp_v = P_red_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mrd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // red
			temp_v = P_green_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mgd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // green
			temp_v = P_cyan_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mcd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // cyan
			temp_v = P_purple_wrt_world-desired_wrt_world.getOrigin(); temp_Q = ((desired_wrt_world.getRotation().inverse())*tf::Quaternion(temp_v.getX(),temp_v.getY(),temp_v.getZ(),0.0))*desired_wrt_world.getRotation(); mpd_bar = tf::Vector3(temp_Q.getX(),temp_Q.getY(),temp_Q.getZ()); // purple
			prd = A*((1/mrd_bar.getZ())*mrd_bar); pgd = A*((1/mgd_bar.getZ())*mgd_bar); pcd = A*((1/mcd_bar.getZ())*mcd_bar); ppd = A*((1/mpd_bar.getZ())*mpd_bar);// camera points as pixels
			alpha_red_d = mr_bar_ref.getZ()/mrd_bar.getZ();
			
			/********* update time and reference*********/
			last_time = ros::Time::now();
			br.sendTransform(tf::StampedTransform(reference_wrt_world, last_time, "world", "reference_image"));
			
			/********* desired wrt reference *********/
			br.sendTransform(tf::StampedTransform(desired_wrt_world, last_time, "world", "desired_image")); 
			tf::StampedTransform desired_wrt_reference;
			listener.waitForTransform("reference_image", "desired_image", last_time, ros::Duration(1.0));
			listener.lookupTransform("reference_image", "desired_image", last_time, desired_wrt_reference);
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
			
			last_time_d = last_time;//desired last time
			start_time_d = last_time;//starting time
			
		}
		
		/********** callback for the uav body imu **********/
		void body_imu_callback(const sensor_msgs::Imu& msg)
		{
			w_body = tf::Vector3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
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
			
			// if it is not the first run and start autonomous is true want to update the desired and then output a new command otherwise just pass
			if (!first_run && start_autonomous)
			{
				new_decomp_recieved = true;
			}
			last_time = current_time;
			
		}
		
		/********** callback for the controller **********/
		void joy_callback(const sensor_msgs::Joy& msg)
		{
			command_from_xbox = geometry_msgs::Twist();// cleaning the xbox twist message
			double joy_gain = 0.1;
			a_button_land_b0 = msg.buttons[0];// a button lands
			if (a_button_land_b0 > 0)
			{
				land_pub.publish(std_msgs::Empty());
			}
			
			b_button_reset_b1 = msg.buttons[1];// b button resets
			if (b_button_reset_b1 > 0)
			{
				reset_pub.publish(std_msgs::Empty());
			}
			
			y_button_takeoff_b3 = msg.buttons[3];// y button takes off
			if (y_button_takeoff_b3 > 0)
			{
				takeoff_pub.publish(std_msgs::Empty());
			}
				
			lb_button_teleop_b4 = msg.buttons[4];// left bumper for autonomous mode
			rb_button_teleop_b5 = msg.buttons[5];// right bumper says to start controller
			if (rb_button_teleop_b5 > 0)
			{
				if (!start_controller)
				{
					start_controller = true;
					std::cout << "start controller" << std::endl;
				}
			}
			
			// if the bumper is pulled and it is not in autonomous mode will put it in autonomous mode
			// if the bumper is pulled and it is in autonomous mode will take it out of autonomous mode
			start_autonomous = lb_button_teleop_b4 > 0;
			
			std::cout << "controller is started: " << start_controller << std::endl;
			std::cout << "autonomous mode is: " << start_autonomous << std::endl;
			
			rt_stick_ud_x_a3 = joy_deadband(msg.axes[3]);// right thumbstick up and down controls linear x
			command_from_xbox.linear.x = joy_gain*rt_stick_ud_x_a3;
			
			rt_stick_lr_y_a2 = joy_deadband(msg.axes[2]);// right thumbstick left and right controls linear y
			command_from_xbox.linear.y = joy_gain*rt_stick_lr_y_a2;
			
			lt_stick_ud_z_a1 = joy_deadband(msg.axes[1]);// left thumbstick up and down controls linear z
			command_from_xbox.linear.z = joy_gain*lt_stick_ud_z_a1;
			
			lt_stick_lr_th_a0 = joy_deadband(msg.axes[0]);// left thumbstick left and right controls angular z
			command_from_xbox.angular.z = joy_gain*lt_stick_lr_th_a0;
			std::cout << "xbox callback" << std::endl;
			
			std::cout << "linear x: " << command_from_xbox.linear.x << std::endl;
			std::cout << "linear y: " << command_from_xbox.linear.y << std::endl;
			std::cout << "linear z: " << command_from_xbox.linear.z << std::endl;
			std::cout << "angular z: " << command_from_xbox.linear.z << std::endl;
			
			if (first_run || (!first_run && !start_autonomous ) )
			{
				new_joy_recieved = true;
			}
			
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
			alpha_red_d = mr_bar_ref.getZ()/mrd_bar.getZ();
			
			/********* desired wrt reference *********/
			//std::cout << "desired 1" << std::endl;
			
			/********* update reference*********/
			br.sendTransform(tf::StampedTransform(reference_wrt_world, current_time_d, "world", "reference_image"));
			
			//std::cout << "desired 2" << std::endl;
			
			/********* desired wrt reference *********/
			br.sendTransform(tf::StampedTransform(desired_wrt_world, current_time_d, "world", "desired_image"));
			
			//std::cout << "desired 3" << std::endl;				
			
			tf::StampedTransform desired_wrt_reference;
			listener.waitForTransform("reference_image", "desired_image", current_time_d, ros::Duration(5.0));
									 
			//std::cout << "desired 4" << std::endl;
			
			listener.lookupTransform("reference_image", "desired_image", current_time_d, desired_wrt_reference);
			
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
			last_time_d = current_time_d;
			
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
			if (alpha_red > 0 && start_controller)
			{
				std::cout << "controller" << std::endl;
				generate_velocity_command_from_tracking();
			}
			
			if (start_autonomous && start_controller)
			{
				std::cout << "command from controller" << std::endl;
				std::cout << "wc:\n x: " << wcd.getX() << " y: " << wcd.getY() << " z: " << wcd.getZ() << std::endl;
				std::cout << "vc:\n x: " << vcd.getX() << " y: " << vcd.getY() << " z: " << vcd.getZ() << std::endl;
				std::cout << "w_body:\n x: " << w_body.getX() << " y: " << w_body.getY() << " z: " << w_body.getZ() << std::endl;
			
				if (!std::isnan(vc.getX()) && !std::isnan(vc.getY()) && !std::isnan(vc.getY()) && !std::isnan(wc.getZ()))
				{	
					// output linear velocity is q_cam_wrt_body * vc * q_cam_wrt_body.inverse() - w_body_wrt_world X p_cam_wrt_body
					tf::Quaternion vc_body_term1 = (camera_wrt_body.getRotation() * tf::Quaternion(vc.getX(), vc.getY(), vc.getZ(), 0)) * (camera_wrt_body.getRotation().inverse());
					tf::Vector3 vc_body_term2 = w_body.cross(camera_wrt_body.getOrigin());
					velocity_command.linear.x = vc_body_term1.getX() + vc_body_term2.getX();
					velocity_command.linear.y = vc_body_term1.getY() + vc_body_term2.getY();
					velocity_command.linear.z = vc_body_term1.getZ() + vc_body_term2.getZ();
					
				
					// output angular velocity is q_cam_wrt_body * wc * q_cam_wrt_body.inverse()
					tf::Quaternion wc_body = (camera_wrt_body.getRotation() * tf::Quaternion(wc.getX(), wc.getY(), wc.getZ(), 0)) * (camera_wrt_body.getRotation().inverse());
					velocity_command.angular.z = wc_body.getZ();
					//std::cout << "wc_body:\n x: " << wcd.getX() << " y: " << wcd.getY() << " z: " << wcd.getZ() << std::endl;
					//std::cout << "vc_body:\n x: " << velocity_command.linear.x << " y: " << velocity_command.linear.y << " z: " << velocity_command.linear.z << std::endl;
					std::cout << "none are nan" << std::endl;
					std::cout << "vc_body_term1:\n x: " << vc_body_term1.getX() << " y: " << vc_body_term1.getY() << " z: " << vc_body_term1.getZ() << " w: " << vc_body_term1.getW() << std::endl;
					std::cout << "vc_body_term2:\n x: " << vc_body_term2.getX() << " y: " << vc_body_term2.getY() << " z: " << vc_body_term2.getZ() << std::endl;
					std::cout << "wc_body:\n x: " << wc_body.getX() << " y: " << wc_body.getY() << " z: " << wc_body.getZ() << std::endl;
				}
				else
				{
					velocity_command.linear.x = 0;
					velocity_command.linear.y = 0;
					velocity_command.linear.z = 0;
					velocity_command.angular.z = 0;
				}
			}
			else
			{
				std::cout << "command from xbox" << std::endl;
				velocity_command = command_from_xbox;
			}
			std::cout << "linear x: " << velocity_command.linear.x << std::endl;
			std::cout << "linear y: " << velocity_command.linear.y << std::endl;
			std::cout << "linear z: " << velocity_command.linear.z << std::endl;
			std::cout << "angular z: " << velocity_command.linear.z << std::endl;
			cmd_vel_pub.publish(velocity_command);
			
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
	
	std::string filename = "/home/ncr/ncr_ws/src/homog_track/testing_files/experiment_3.txt";
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
	//SimulatedImageProcessing simulated_image_processing(loop_rate_hz);// simulated image processing 
	HomogDecomp homog_decomp;// homography decomp
	Controller controller(loop_rate_hz, write_to_file, filename);// controller
	
	ros::Rate loop_rate(loop_rate_hz*10);
	
	while (ros::ok())
	{
		controller.update_desired_pixels();
		if (controller.new_joy_recieved || controller.new_decomp_recieved)
		{
			controller.output_velocity_command();
			controller.new_joy_recieved = false;
			controller.new_decomp_recieved = false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	//ros::spin();

    return 0;
}
