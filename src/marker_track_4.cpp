#include <iostream>
#include <string>
#include <vector>

// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <homog_track/HomogMarker.h>
#include <homog_track/HomogComplete.h>
#include <homog_track/HomogReference.h>


// prototype for the filtering function
void filter_image_for_color(cv::Mat&, cv::Mat&, int*, int*,cv::Mat, cv::Mat);

// class to contain a circle
// order will be radius, x, y
class Circle
{       
    public:
        // values for the circle
        float radius;
        float x;
        float y;
        
        // first constructor callback
		Circle(float self_radius, float self_x, float self_y)
		{
			radius = self_radius;
			x = self_x;
			y = self_y;
		}

		// second constructor callback
		Circle()
		{
			radius = 0;
			x = 0;
			y = 0;
		}

		// set callback
		void setCircle(float self_radius, float self_x, float self_y)
		{
			radius = self_radius;
			x = self_x;
			y = self_y;
		}

		// function to print circle
		void printCircle()
		{
			std::cout << "x: "
					  << x
					  << " y: "
					  << y
					  << " r: " 
					  << radius 
					  << std::endl;
		}

		// deconstructor callback
		~Circle()
		{
			// std::cout << "\n\tCircle Destroyed\n" << std::endl;
		}
};


// class to convert images
class ImageConverter
{
	public:
		// node for the image converter
		ros::NodeHandle nh_;
		
		/********** Begin Topic Declarations **********/
		// image transport publisher and subscriber
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;
		
		// circles publishers
		ros::Publisher circle_pub;
		
		// reference service
		ros::ServiceServer ref_service;
		
		/********** End Topic Declarations **********/
		
		/********** Begin Kalman Declarations **********/
		cv::KalmanFilter KF {4,2,0,CV_64F};
		
		
		/********** End Kalman Declarations **********/
		
		/********** Begin Point Declarations **********/
		// the four points for the circles
		geometry_msgs::Point32 red_circle_p;
		geometry_msgs::Point32 green_circle_p;
		geometry_msgs::Point32 cyan_circle_p;
		geometry_msgs::Point32 purple_circle_p;
		
		// constant circles for reference
		geometry_msgs::Point32 red_circle_const;
		geometry_msgs::Point32 green_circle_const;
		geometry_msgs::Point32 cyan_circle_const;
		geometry_msgs::Point32 purple_circle_const;
		
		// d*           = 1 m
		// red:   (x,y) = (-0.05, -0.05) m
		// green: (x,y) = ( 0.05, -0.05) m
		// cyan:  (x,y) = ( 0.05,  0.05) m
		// blue:  (x,y) = (-0.05,  0.05) m
		
		// if the camera reference is directly centered as defined by these points then 
		// n*           = [0,0,1]^T
		
		// also the four points normalized will be
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
		
		// tells the reference has been set
		bool reference_set;
		
		// current marker
		homog_track::HomogMarker homog_circles_curr;
		
		// reference marker
		homog_track::HomogMarker homog_circles_ref;
		
		// constant reference marker
		homog_track::HomogMarker homog_circles_ref_const;
		
		// holds the complete message
		homog_track::HomogComplete complete_msg;
		
		/********** End Point Declarations **********/

		/********** Begin threshold declarations and initializations **********/
		// order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
		// values for the red color threshold values: hue, staturation, and value
		int r_thresh[6] = {170, 5, 45, 255, 85, 255};

		// values for the green color threshold values: hue, staturation, and value
		int g_thresh[6] = {65, 90, 55, 255, 40, 255};

		// values for the cyan color threshold values: hue, staturation, and value
		int c_thresh[6] = {95, 105, 125, 255, 80, 255};
		
		// values for the violet color threshold values: hue, staturation, and value
		int p_thresh[6] = {110, 135, 45, 255, 50, 255};

		// putting the start arrays into a vector
		std::vector<int*> thresh_current{ r_thresh, g_thresh, c_thresh, p_thresh };

		// max values for the primary colorthreshold value: hue, saturation, and value
		// order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
		int thresh_max[6] = {180, 180, 255, 255, 255, 255};

		/********** End threshold declarations and initializations **********/

		
		/********** Begin Image Matrix Declarations **********/    
		// image pointer for the converted images
		cv_bridge::CvImagePtr cv_ptr;
		
		// distorted frame
		cv::Mat distorted_frame;

		// frame will be a 3 channel 8 bit image
		cv::Mat frame;
		
		// blur frame is the same as frame
		cv::Mat blur_frame;
		
		// hsv is the same as frame
		cv::Mat hsv_frame;
		
		// red binary frame is a 1 channel 8 bit image
		cv::Mat r_binary_frame;
		
		// green binary frame is same as red
		cv::Mat g_binary_frame;

		// cyan binary frame is same as red
		cv::Mat c_binary_frame;
		
		// purple binary frame is same as red
		cv::Mat p_binary_frame;

		// putting the binary frames into a vector
		std::vector<cv::Mat> binary_frames{r_binary_frame, g_binary_frame, c_binary_frame, p_binary_frame};
		
		// camera matrix
		cv::Mat K;
		
		// distortion coefficients
		std::vector<double> dC;
		
		
		
		/********** End Image Matrix Declarations **********/
		
		/********** Begin Kernel Declarations **********/
		// kernel size for blurring
		int blur_kernel_size = 11;
		
		// kernel size for erode
		int erode_kernel_size = 5;

		// kernel size for dilate
		int dilate_kernel_size = 3;

		// getting the structuring element for the erode
		cv::Mat erode_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
														   cv::Size(  erode_kernel_size, erode_kernel_size ),
														   cv::Point( -1, -1) 
														 );
		
		// getting the structuring element for the dilate 
		cv::Mat dilate_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
															cv::Size( dilate_kernel_size, dilate_kernel_size ),
															cv::Point( -1, -1) 
														  );

		/********** End Kernel Declarations **********/    

		/********** Begin Decarations for the contours and the shapes **********/
		// red contour
		std::vector< std::vector<cv::Point>> r_contours;
		
		// green contour
		std::vector< std::vector<cv::Point>> g_contours;
		
		// cyan contour
		std::vector< std::vector<cv::Point>> c_contours;
		
		// violet contour
		std::vector< std::vector<cv::Point>> p_contours;
		
		// putting the contours into a vector
		std::vector< std::vector< std::vector<cv::Point> > > contours{r_contours, g_contours, c_contours, p_contours};

		// names for the contours
		std::vector<std::string> contour_names{ "Red Contour", "Green Contour", "Cyan Contour", "Purple Contour" };

		// enclosing circle radius and center for the contours
		// order is (radius, center x, center y)
		Circle *temp_circle = new Circle;

		// temp_circle->printCircle();

		// red circles
		std::vector<Circle> r_circles1;
		
		// green circles
		std::vector<Circle> g_circles1;
	 
		// cyan circles
		std::vector<Circle> c_circles1;
		
		// purple circles
		std::vector<Circle> p_circles1;

		// red circles
		std::vector<Circle> r_circles2;
		
		// green circles
		std::vector<Circle> g_circles2;
		
		// cyan circles
		std::vector<Circle> c_circles2;
		
		// purple circles
		std::vector<Circle> p_circles2;

		// // print out radius
		// std::cout << temp_circle->radius;

		// putting them in a vector
		std::vector< std::vector<Circle> > temp_circles{r_circles1, g_circles1, c_circles1, p_circles1};

		// final circles
		std::vector< std::vector<Circle> > final_circles{r_circles2, g_circles2, c_circles2, p_circles2};

		// tells which index is currently the greatest
		int current_index;

		// tells which index is the compare index
		int current_compare;

		// current number of circles found
		int current_number;

		// desired number of circles to findhomog_track/HomogMarker
		int desired_number;

		// tells when the current compare has exceeded the length of the vector
		bool end_reached;

		/********** End Decarations for the contours and the shapes **********/
		
		// constructor for the image converter
		ImageConverter() : it_(nh_)
		{
			//// Subscribe to input video feed from bluefox
			//image_sub_ = it_.subscribe( "/mv_BF001066/image_raw", 1, &ImageConverter::image_callback, this);
										
										
			/*********dont forget the distortion coef***********/
			// subscribe to ardrone front camera
			image_sub_ = it_.subscribe( "/ardrone/front/image_raw", 1, &ImageConverter::image_callback, this);
										
			// publish processed image from homography
			image_pub_ = it_.advertise("current_homog_marker_image", 1);
			
			/********** Initialize the publisher for the 4 circles **********/
			// each will be sent as a point message
			circle_pub = nh_.advertise<homog_track::HomogComplete>("complete_homog_set",1);
			
			/********** Initialize the service for setting reference **********/
			ref_service = nh_.advertiseService("set_reference", &ImageConverter::set_reference_service_handler,this);
			
			// command the virtual camera by piggy backing of cmd_vel
			virtual_camera_sub = nh_.subscribe("/cmd_vel", 1, &ImageConverter::virtual_camera_callback, this);
			
			
			reference_set = false;
			
			homog_circles_ref.red_circle.x = -1;
			homog_circles_ref.red_circle.y = -1;
			homog_circles_ref.red_circle.z = -1;
			homog_circles_ref.green_circle.x = -1;
			homog_circles_ref.green_circle.y = -1;
			homog_circles_ref.green_circle.z = -1;
			homog_circles_ref.cyan_circle.x = -1;
			homog_circles_ref.cyan_circle.y = -1;
			homog_circles_ref.cyan_circle.z = -1;
			homog_circles_ref.purple_circle.x = -1;
			homog_circles_ref.purple_circle.y = -1;
			homog_circles_ref.purple_circle.z = -1;
			
			
			// camera matrix for the ardrone
			K = cv::Mat::zeros(3,3,CV_64F);
			K.at<double>(0,0) = 567.79;
			K.at<double>(0,2) = 337.35;
			K.at<double>(1,1) = 564.52;
			K.at<double>(1,2) = 169.54;
			K.at<double>(2,2) = 1;
			
			std::cout << " K value: " << K << std::endl;
			
			/********** actual marker values from 1 m when reference starts as the direct center **********/
			// d*           = 1 m
			// red:   (x,y) = (-0.05, -0.05) m
			// green: (x,y) = ( 0.05, -0.05) m
			// cyan:  (x,y) = ( 0.05,  0.05) m
			// blue:  (x,y) = (-0.05,  0.05) m
			
			// if the camera reference is directly centered as defined by these points then 
			// n*           = [0,0,1]^T
			
			// also the four points normalized will be
			// m*n_red      = [-0.05, -0.05, 1]^T
			// m*n_green    = [ 0.05, -0.05, 1]^T
			// m*n_cyan     = [ 0.05,  0.05, 1]^T
			// m*n_purple   = [-0.05,  0.05, 1]^T
			
			// converting the normalized coordinates into pixel coordinates using p* = (K)(m*_n)
			// p*_red      = K[-0.05, -0.05, 1]^T
			// p*_green    = K[ 0.05, -0.05, 1]^T
			// p*_cyan     = K[ 0.05,  0.05, 1]^T
			// p*_purple   = K[-0.05,  0.05, 1]^T
			
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
			mr_ref_norm = 1.0/z_ref*mr_ref_bar;
			mg_ref_norm = 1.0/z_ref*mg_ref_bar;
			mc_ref_norm = 1.0/z_ref*mc_ref_bar;
			mp_ref_norm = 1.0/z_ref*mp_ref_bar;
			
			// as pixels
			pr_ref = K*mr_ref_norm;
			pg_ref = K*mg_ref_norm;
			pc_ref = K*mc_ref_norm;
			pp_ref = K*mp_ref_norm;
			
			// red constant
			homog_circles_ref_const.red_circle.x = pr_ref.at<double>(0,0);
			homog_circles_ref_const.red_circle.y = pr_ref.at<double>(1,0);
			homog_circles_ref_const.red_circle.z = pr_ref.at<double>(2,0);
			
			// green constant
			homog_circles_ref_const.green_circle.x = pg_ref.at<double>(0,0);
			homog_circles_ref_const.green_circle.y = pg_ref.at<double>(1,0);
			homog_circles_ref_const.green_circle.z = pg_ref.at<double>(2,0);
			
			// cyan constant
			homog_circles_ref_const.cyan_circle.x = pc_ref.at<double>(0,0);
			homog_circles_ref_const.cyan_circle.y = pc_ref.at<double>(1,0);
			homog_circles_ref_const.cyan_circle.z = pc_ref.at<double>(2,0);
			
			// purple constant
			homog_circles_ref_const.purple_circle.x = pp_ref.at<double>(0,0);
			homog_circles_ref_const.purple_circle.y = pp_ref.at<double>(1,0);
			homog_circles_ref_const.purple_circle.z = pp_ref.at<double>(2,0);
			
			// distortion matrix for the ardrone
			dC.push_back(-0.5122398601387984);
			dC.push_back(0.2625218940944695);
			dC.push_back(-0.0009892579344331395);
			dC.push_back(0.002480111502028633);
			dC.push_back(0.0);
			
			std::cout << "dC value: ";
			for (float ii : dC)
			{
				std::cout << ii << " ";
			}
			std::cout << std::endl;
			
			////////// initialize KF //////////
			KF.transitionMatrix = cv::Mat::eye(4,4,CV_64F);
			KF.transitionMatrix.at<double>(0,2) = 1;
			KF.transitionMatrix.at<double>(1,1) = 1;
			cv::Mat_<double> measurement(2,1);
			measurement.setTo(Scalar(0));
 
			KF.statePre.at<float>(0) = 0;
			KF.statePre.at<float>(1) = 0;
			KF.statePre.at<float>(2) = 0;
			KF.statePre.at<float>(3) = 0;

			setIdentity(KF.measurementMatrix);
			setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
			setIdentity(KF.measurementNoiseCov, Scalar::all(10));
			setIdentity(KF.errorCovPost, Scalar::all(.1));

		}
		
		// destructor for the image converter
		~ImageConverter()
		{

		}

		// service callback/handler
		bool set_reference_service_handler(homog_track::HomogReference::Request &req, homog_track::HomogReference::Response &res)
		{
			// if the reference is 0 will use the constant reference
			if (req.reference_choice == 0)
			{
				homog_circles_ref = homog_circles_ref_const;
				reference_set = true;
				res.success = true;
			}
			// if the reference is 1 will use the current value choice
			else if (req.reference_choice == 1)
			{
				if (homog_circles_curr.red_circle.x == -1 || homog_circles_curr.green_circle.x == -1 || homog_circles_curr.cyan_circle.x == -1 || homog_circles_curr.purple_circle.x == -1)
				{
					res.success = false;
				}
				else
				{
					homog_circles_ref = homog_circles_curr;
					reference_set = true;
					res.success = true;
				}
			}
			else
			{
				res.success = false;
			}
			
			return true;
		}

		// callback to update the virtual camera
		void virtual_camera_callback(const geometry_msg::geometry_msgs::Twist &msg)
		{
			if ()
		}

		// callback for the image subscriber
		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{
			// trying to initialize the ptr to the image passed in and
			// converting it to a bgr8 image so opencv can use it
			// if it fails will through the error
			try
			{
				// converting the image
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				
				// getting the frame from the converted image can use clone on image
				distorted_frame = cv_ptr->image;
				
				// undistorting the frame
				cv::undistort(distorted_frame,frame,K,dC);
				
				//std::cout << "image converted" << std::endl;
			  
				/********** begin processing the image **********/
				// median blur on the current frame
				cv::GaussianBlur( frame, blur_frame, cv::Size(blur_kernel_size,blur_kernel_size),0,0 );
				
				// converting the frame to hsv
				cv::cvtColor(blur_frame,hsv_frame,CV_BGR2HSV);

				// filtering the image for the  different colors
				// need to do it four times, once for each circle color
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
					
					// declaring centers to be all zeros the size of contours to hold all the contours circles centers
					std::vector<cv::Point2f> center( contours[ii].size() );

					// declaring radius to be all zeros the size of contours to hold all the contours circles radius
					std::vector<float> radius( contours[ii].size() );

					// getting the minimum enclcosing circle for the contours
					for (int jj = 0; jj < contours[ii].size(); jj++)
					{
						// getting the circle for the current contour
						minEnclosingCircle( contours[ii][jj], center[jj], radius[jj] );
						
						// create the new var for storing
						temp_circle = new Circle;

						// giving it to the circle
						temp_circle->setCircle(radius[jj], center[jj].x, center[jj].y);

						// adding the circle to the temp array                
						temp_circles[ii].push_back (*temp_circle);

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

					// getting the number of maxes needed and adding them to the final vector as long as it
					// is not empty
					while ( (desired_number > current_number) && !temp_circles[ii].empty() )
					{
						// first checking if the current index and current comapre are the same will increment
						// to the next compare. if it exceeds loop will take the current index for the max
						if (current_index == current_compare)
						{
							// incrementing to the next compare
							current_compare++;

							// if the current index sends current compare past the end will take the 
							// current index to be the max
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
							// resetting the index to be the compare
							current_index = current_compare;
							
							// restting the index
							current_compare = 0;
						}

						// else is if the radius at the current index is greater than the current compare will increment
						// to the next compare. if it exceeds the vector will take the current index for the max.
						else
						{
							// incrementing to the next compare
							current_compare++;

							// if the current index sends current compare past the end will take the 
							// current index to be the max
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
							// adding the circle to the final circle vector
							final_circles[ii].push_back ( temp_circles[ii][current_index] );

							// removing the value form the temporary circles
							temp_circles[ii].erase( temp_circles[ii].begin() + current_index );

							// incrementing the current number
							current_number++;

							// reset current index
							current_index = 0;
							
							// reset current compare
							current_compare = 0;

							// resetting end reached
							end_reached = false;
						}
					}

					// erasing all the temp circles
					temp_circles[ii].erase(temp_circles[ii].begin(),temp_circles[ii].end());
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
						// color for the drawing
						cv::Scalar* color_con = new cv::Scalar();
						
						// choosing which circle based on ii value
						switch (ii)
						{
							// red
							case 0:
								//std::cout << "red found" << std::endl;
								color_con = new cv::Scalar(0, 0, 255);
								// assigning the values of the circle to the circle
								red_circle_p.x = final_circles[ii][0].x;
								red_circle_p.y = final_circles[ii][0].y;
								red_circle_p.z = 1;
								break;
								
							// green
							case 1:
								//std::cout << "green found" << std::endl;
								color_con = new cv::Scalar(0, 255, 0);
								// assigning the values of the circle to the circle
								green_circle_p.x = final_circles[ii][0].x;
								green_circle_p.y = final_circles[ii][0].y;
								green_circle_p.z = 1;
								break;
								
							// cyan
							case 2:
								//std::cout << "cyan found" << std::endl;
								color_con = new cv::Scalar(255, 255, 0);
								// assigning the values of the circle to the circle
								cyan_circle_p.x = final_circles[ii][0].x;
								cyan_circle_p.y = final_circles[ii][0].y;
								cyan_circle_p.z = 1;
								break;
								
							// purple
							case 3:
								//std::cout << "purple found" << std::endl;
								color_con = new cv::Scalar(255, 0, 255);
								// assigning the values of the circle to the circle
								purple_circle_p.x = final_circles[ii][0].x;
								purple_circle_p.y = final_circles[ii][0].y;
								purple_circle_p.z = 1;
								break;
						}
						
						// draw all the contours
						cv::drawContours(blur_frame, contours[ii], -1, *color_con, 1);
						
						cv::circle( blur_frame, 
									cv::Point(final_circles[ii][0].x, final_circles[ii][0].y ),
									final_circles[ii][0].radius,
									*color_con,
									2
									);
						 // First predict, to update the internal statePre variable
						 cv::Mat prediction = KF.predict();
						 cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
						 // update using the red position
						 measurement(0) = final_circles[0][0].x;
						 measurement(1) = final_circles[0][0].y; 
						  
						 // The update phase 
						 cv::Mat estimated = KF.correct(measurement);
						 // printing out the circles values
						 //final_circles[ii][0].printCircle();
						 cv::circle( blur_frame, 
									cv::Point(estimated.at<double>(0), estimated.at<double>(1)),
									4,
									cv::Scalar(0,0,0),
 									4
 									);
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
								// assigning the values of the circle to the circle
								red_circle_p.x = -1;
								red_circle_p.y = -1;
								red_circle_p.z = -1;
								break;
								
							// green
							case 1:
								//std::cout << "green missing" << std::endl;
								// assigning the values of the circle to the circle
								green_circle_p.x = -1;
								green_circle_p.y = -1;
								green_circle_p.z = -1;
								break;
								
							// cyan
							case 2:
								//std::cout << "cyan missing" << std::endl;
								// assigning the values of the circle to the circle
								cyan_circle_p.x = -1;
								cyan_circle_p.y = -1;
								cyan_circle_p.z = -1;
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
				homog_circles_curr.header.stamp = msg->header.stamp;
				homog_circles_curr.red_circle = red_circle_p;
				homog_circles_curr.green_circle = green_circle_p;
				homog_circles_curr.cyan_circle = cyan_circle_p;
				homog_circles_curr.purple_circle = purple_circle_p;
				homog_circles_ref.header.stamp = msg->header.stamp;
				
				complete_msg.header.stamp = msg->header.stamp;
				complete_msg.reference_set = reference_set;
				complete_msg.current_points = homog_circles_curr;
				complete_msg.reference_points = homog_circles_ref;
				
				// publish the circle points
				circle_pub.publish(complete_msg);
				
				std::cout << "complete message\n" << complete_msg << std::endl << std::endl;
				
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

// main
int main(int argc, char** argv)
{   
	// initialize node
	ros::init(argc,argv,"tracking_node");
	
	// initialize image converter    
	ImageConverter image_converter;

    // release and sleep until next time image is available
    ros::spin();
    
    return 0;
}

// function to filter for color blobs
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
