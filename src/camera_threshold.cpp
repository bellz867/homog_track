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

/********** Image generating class **********/
class ImageGenerator
{
	public:
		// node for the image converter
		ros::NodeHandle nh;
		
		/********** Topic Declarations **********/
		image_transport::ImageTransport it_;
		image_transport::Publisher image_pub;
		
		/********** Image Declarations **********/    
		sensor_msgs::ImagePtr toImageMsg() const;
		cv::VideoCapture cap;
		std::string video;
		
		/********* Constructor for the image processing *********/
		ImageGenerator(std::string& input_video) : it_(nh)
		{
			image_pub = it_.advertise("/ardrone/front/image_raw", 1);// publish processed image
			video = input_video;
			cap = cv::VideoCapture(video);
		}
		
		/********** Callback to publish the next image **********/
		void update_image()
		{
			if (cap.isOpened())
			{
				cv::Mat image;
				if (cap.read(image))
				{
					cv_bridge::CvImage bridge_image;
					bridge_image.header.stamp = ros::Time::now();
					bridge_image.header.frame_id = "front_camera";
					bridge_image.encoding = "bgr8";
					bridge_image.image = image;
					sensor_msgs::Image sensor_image;
					bridge_image.toImageMsg(sensor_image);
					image_pub.publish(sensor_image);
					std::cout << "image published" << std::endl;
				}
				else
				{
					std::cout << "video ended, releasing" << std::endl;
					cap.release();
				}
			}
			else
			{
				std::cout << "video is closed, reopening" << std::endl;
				cap = cv::VideoCapture(video);
			}
			cv::waitKey(1);
		}
		
		~ImageGenerator()
		{
			if (cap.isOpened())
			{
				cap.release();
			}
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
		int blur_kernel_size = 5;// kernel size for blurring
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
			//A = cv::Mat::zeros(3,3,CV_64F); A.at<double>(0,0) = 567.79; A.at<double>(0,2) = 337.35; A.at<double>(1,1) = 564.52; A.at<double>(1,2) = 169.54; A.at<double>(2,2) = 1; // camera matrix for the ardrone
			//dC.push_back(-0.5122398601387984); dC.push_back(0.2625218940944695); dC.push_back(-0.0009892579344331395); dC.push_back(0.002480111502028633); dC.push_back(0.0); // distortion matrix for the ardrone
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

// main
int main(int argc, char** argv)
{   
	ros::init(argc,argv,"threshold_testing_node");

	double loop_rate_hz = 15;
	std::string video_file = "/home/v1_ws/overshoot_experiment_8_regulation.avi";
	ImageGenerator image_generator(video_file);
	ImageProcessing image_processing;// image processing
	
	ros::Rate loop_rate(loop_rate_hz);
	
	while (ros::ok())
	{
		image_generator.update_image();
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}
