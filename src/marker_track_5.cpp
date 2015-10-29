#include <iostream>
#include <string>
#include <vector>
#include <cmath>

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
		
		// virtual camera subscriber
		ros::Subscriber virtual_camera_sub;
		
		// reference service
		ros::ServiceServer ref_service;
		
		// tf broadcaster
		tf::TransformBroadcaster br;
		
		/********** End Topic Declarations **********/
		
		// tf for the world, reference, and current camera
		tf::Transform world_to_reference;
		tf::Transform reference_to_current;
		
		
		/********** Begin Point Declarations **********/
		// the four points for the circles
		geometry_msgs::Point32 red_circle_p;
		geometry_msgs::Point32 green_circle_p;
		geometry_msgs::Point32 cyan_circle_p;
		geometry_msgs::Point32 purple_circle_p;
		
		// the four points for the circles currently
		geometry_msgs::Point32 red_circle_p_curr;
		geometry_msgs::Point32 green_circle_p_curr;
		geometry_msgs::Point32 cyan_circle_p_curr;
		geometry_msgs::Point32 purple_circle_p_curr;
		
		// the four points for the circles last time
		geometry_msgs::Point32 red_circle_p_last;
		geometry_msgs::Point32 green_circle_p_last;
		geometry_msgs::Point32 cyan_circle_p_last;
		geometry_msgs::Point32 purple_circle_p_last;
		
		// tuning gain for the low pass
		float low_pass_gain = 0.75;
		
		// boolean to tell if this is the first run
		bool first_run = false;
		
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
		double z_ref;
		
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
		
		// rotation and translation from reference to the desired virtual camera
		cv::Mat vir_R;
		tf::Matrix3x3 vir_R_3x3;
		tf::Quaternion vir_Q;
		cv::Mat vir_T;
		cv::Mat H;
		cv::Mat G;
		cv::Mat n_ref;
		double d_ref;
		std::vector<cv::Point2f> curr_points_p;
		std::vector<cv::Point2f> ref_points_p;
		cv::Mat G_fH;
		
		// real coordinate values for a fixed virtual camera at some location
		cv::Mat mr_vir_bar;
		cv::Mat mg_vir_bar;
		cv::Mat mc_vir_bar;
		cv::Mat mp_vir_bar;
		
		// normalized
		cv::Mat mr_vir_norm;
		cv::Mat mg_vir_norm;
		cv::Mat mc_vir_norm;
		cv::Mat mp_vir_norm;
		
		// as pixels
		cv::Mat pr_vir;
		cv::Mat pg_vir;
		cv::Mat pc_vir;
		cv::Mat pp_vir;
		
		// tells if want to use the virtual reference
		bool using_virtual_reference = false;
		
		// last time the virtual reference was updated
		ros::Time last_virtual_update_time;
		
		// holds the current yaw angle
		double virtual_yaw = 0;
		double virtual_roll = 0;
		double virtual_pitch = 0;
		
		// tells the reference has been set
		bool reference_set;
		
		// current marker
		homog_track::HomogMarker homog_circles_curr;
		
		// reference marker
		homog_track::HomogMarker homog_circles_ref;
		
		// constant reference marker
		homog_track::HomogMarker homog_circles_ref_const;
		
		// virtual marker
		homog_track::HomogMarker homog_circles_virtual;
		
		// holds the complete message
		homog_track::HomogComplete complete_msg;
		
		/********** End Point Declarations **********/

		/********** Begin threshold declarations and initializations **********/
		// order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
		// values for the red color threshold values: hue, staturation, and value
		int r_thresh[6] = {170, 5, 45, 255, 85, 255};

		// values for the green color threshold values: hue, staturation, and value
		int g_thresh[6] = {65, 90, 45, 255, 40, 255};

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
			
			// update the virtual camera by piggy backing off the cmd_vel
			virtual_camera_sub = nh_.subscribe("/cmd_vel",1, &ImageConverter::virtual_camera_callback, this);
			
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
			
			
			//// camera matrix for the ardrone
			//K = cv::Mat::zeros(3,3,CV_64F);
			//K.at<double>(0,0) = 567.79;
			//K.at<double>(0,2) = 337.35;
			//K.at<double>(1,1) = 564.52;
			//K.at<double>(1,2) = 169.54;
			//K.at<double>(2,2) = 1;
			K = cv::Mat::eye(3,3,CV_64F);
			
			std::cout << " K value:\n" << K << std::endl;
			
			/********** actual marker values from z_ref m when reference starts as the direct center **********/
			////////// setting zref //////////
			z_ref = 2;
			// d*           = z m
			// red:   (x,y) = (-0.05, -0.05) m
			// green: (x,y) = ( 0.05, -0.05) m
			// cyan:  (x,y) = ( 0.05,  0.05) m
			// blue:  (x,y) = (-0.05,  0.05) m
			
			// if the camera reference is directly centered as defined by these points then 
			// n*           = [0,0,1]^T
			
			// also the four points normalized will be
			// m*n_red      = [-0.05/zref, -0.05/zref, 1]^T
			// m*n_green    = [ 0.05/zref, -0.05/zref, 1]^T
			// m*n_cyan     = [ 0.05/zref,  0.05/zref, 1]^T
			// m*n_purple   = [-0.05/zref,  0.05/zref, 1]^T
			
			// converting the normalized coordinates into pixel coordinates using p* = (K)(m*_n)
			// p*_red      = K[-0.05/zref, -0.05/zref, 1]^T
			// p*_green    = K[ 0.05/zref, -0.05/zref, 1]^T
			// p*_cyan     = K[ 0.05/zref,  0.05/zref, 1]^T
			// p*_purple   = K[-0.05/zref,  0.05/zref, 1]^T
			
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
			mr_ref_norm = (1.0/z_ref)*mr_ref_bar;
			mg_ref_norm = (1.0/z_ref)*mg_ref_bar;
			mc_ref_norm = (1.0/z_ref)*mc_ref_bar;
			mp_ref_norm = (1.0/z_ref)*mp_ref_bar;
			
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
			
			/********** real marker values  for a virtual camera  **********/
			///////////// rotation and translation from desired to the reference virtual camera /////////
			vir_R = cv::Mat::zeros(3,3,CV_64F);
			
			// initialize the transforms
			tf::Matrix3x3 temp_rotation3x3;
			
			double yaw_init = 3*M_PIl/2;
			double pitch_init = 0;
			double roll_init = 0;
			
			virtual_yaw = yaw_init;
			virtual_pitch = pitch_init;
			virtual_roll = roll_init;
			
			temp_rotation3x3.setEulerYPR(yaw_init,pitch_init,roll_init);

			vir_R.at<double>(0,0) = temp_rotation3x3[0][0];
			vir_R.at<double>(1,0) = temp_rotation3x3[1][0];
			vir_R.at<double>(2,0) = temp_rotation3x3[2][0];
			vir_R.at<double>(0,1) = temp_rotation3x3[0][1];
			vir_R.at<double>(1,1) = temp_rotation3x3[1][1];
			vir_R.at<double>(2,1) = temp_rotation3x3[2][1];
			vir_R.at<double>(0,2) = temp_rotation3x3[0][2];
			vir_R.at<double>(1,2) = temp_rotation3x3[1][2];
			vir_R.at<double>(2,2) = temp_rotation3x3[2][2];
			
			vir_T = cv::Mat::zeros(3,1,CV_64F);
			///////////// rotation and translation from desired to the reference virtual camera /////////
			vir_T.at<double>(0,0) = -0.5;
			vir_T.at<double>(1,0) = 0.0;
			vir_T.at<double>(2,0) = 2;
			//vir_T.at<double>(0,0) = 0;
			//vir_T.at<double>(1,0) = 0;
			//vir_T.at<double>(2,0) = 0;
			using_virtual_reference = true;
			
			
			// new position is mi_vir_bar = vir_T + vir_R*mi_ref_bar
			
			// real coordinate values for the circles in the virtual
			mr_vir_bar = cv::Mat::zeros(3,1,CV_64F);
			mg_vir_bar = cv::Mat::zeros(3,1,CV_64F);
			mc_vir_bar = cv::Mat::zeros(3,1,CV_64F);
			mp_vir_bar = cv::Mat::zeros(3,1,CV_64F);
			
			// rotating and translating the reference to some pose
			mr_vir_bar = vir_T + vir_R*mr_ref_bar;
			mg_vir_bar = vir_T + vir_R*mg_ref_bar;
			mc_vir_bar = vir_T + vir_R*mc_ref_bar;
			mp_vir_bar = vir_T + vir_R*mp_ref_bar;
			
			std::cout << "reference red:\n" << mr_ref_bar << std::endl;
			std::cout << "reference green:\n" << mg_ref_bar << std::endl;
			std::cout << "reference cyan:\n" << mc_ref_bar << std::endl;
			std::cout << "reference purple:\n" << mp_ref_bar << std::endl;
			
			std::cout << "virtual translation:\n" << vir_T << std::endl;
			std::cout << "virtual rotation:\n" << vir_R << std::endl;
			std::cout << "virtual red bar:\n" << mr_vir_bar << std::endl;
			std::cout << "virtual green bar:\n" << mg_vir_bar << std::endl;
			std::cout << "virtual cyan bar:\n" << mc_vir_bar << std::endl;
			std::cout << "virtual purple bar:\n" << mp_vir_bar << std::endl;
			
			// normalizing
			mr_vir_norm = (1.0/mr_vir_bar.at<double>(2,0))*mr_vir_bar;
			mg_vir_norm = (1.0/mg_vir_bar.at<double>(2,0))*mg_vir_bar;
			mc_vir_norm = (1.0/mc_vir_bar.at<double>(2,0))*mc_vir_bar;
			mp_vir_norm = (1.0/mp_vir_bar.at<double>(2,0))*mp_vir_bar;
			
			std::cout << "virtual red norm:\n" << mr_vir_norm << std::endl;
			std::cout << "virtual green norm:\n" << mg_vir_norm << std::endl;
			std::cout << "virtual cyan norm:\n" << mc_vir_norm << std::endl;
			std::cout << "virtual purple norm:\n" << mp_vir_norm << std::endl;
			
			// as pixels
			pr_vir = K*mr_vir_norm;
			pg_vir = K*mg_vir_norm;
			pc_vir = K*mc_vir_norm;
			pp_vir = K*mp_vir_norm;
			
			std::cout << "virtual red pixels:\n" << pr_vir << std::endl;
			std::cout << "virtual green pixels:\n" << pg_vir << std::endl;
			std::cout << "virtual cyan pixels:\n" << pc_vir << std::endl;
			std::cout << "virtual purple pixels:\n" << pp_vir << std::endl;
			
			// red
			homog_circles_virtual.red_circle.x = pr_vir.at<double>(0,0);
			homog_circles_virtual.red_circle.y = pr_vir.at<double>(1,0);
			homog_circles_virtual.red_circle.z = pr_vir.at<double>(2,0);
			
			// green
			homog_circles_virtual.green_circle.x = pg_vir.at<double>(0,0);
			homog_circles_virtual.green_circle.y = pg_vir.at<double>(1,0);
			homog_circles_virtual.green_circle.z = pg_vir.at<double>(2,0);
			
			// cyan
			homog_circles_virtual.cyan_circle.x = pc_vir.at<double>(0,0);
			homog_circles_virtual.cyan_circle.y = pc_vir.at<double>(1,0);
			homog_circles_virtual.cyan_circle.z = pc_vir.at<double>(2,0);
			
			// purple
			homog_circles_virtual.purple_circle.x = pp_vir.at<double>(0,0);
			homog_circles_virtual.purple_circle.y = pp_vir.at<double>(1,0);
			homog_circles_virtual.purple_circle.z = pp_vir.at<double>(2,0);
			
			//// calculating what the euclidean homography is
			//n_ref = cv::Mat::zeros(3,1,CV_64F);
			//n_ref.at<double>(2,0) = 1;
			//std::cout << "n_ref:\n" << n_ref << std::endl;
			
			//d_ref = (n_ref).dot(mr_ref_bar);
			//std::cout << "d_ref:\t" << d_ref << std::endl;
			
			//H = vir_R + ((1.0/d_ref)*vir_T)*(n_ref.t());
			//std::cout << "euclidean homography:\n" << H << std::endl;
			
			//// finding the alphas
			//std::cout << "red alpha x:\t" << mr_vir_norm.at<double>(0,0)/((H.row(0)).dot(mr_ref_norm.t())) << std::endl;
			//std::cout << "red alpha y:\t" << mr_vir_norm.at<double>(1,0)/((H.row(1)).dot(mr_ref_norm.t())) << std::endl;
			//std::cout << "red alpha z:\t" << mr_vir_norm.at<double>(2,0)/((H.row(2)).dot(mr_ref_norm.t())) << std::endl;
			
			//std::cout << "green alpha x:\t" << mg_vir_norm.at<double>(0,0)/((H.row(0)).dot(mg_ref_norm.t())) << std::endl;
			//std::cout << "green alpha y:\t" << mg_vir_norm.at<double>(1,0)/((H.row(1)).dot(mg_ref_norm.t())) << std::endl;
			//std::cout << "green alpha z:\t" << mg_vir_norm.at<double>(2,0)/((H.row(2)).dot(mg_ref_norm.t())) << std::endl;
			
			//std::cout << "cyan alpha x:\t" << mc_vir_norm.at<double>(0,0)/((H.row(0)).dot(mc_ref_norm.t())) << std::endl;
			//std::cout << "cyan alpha y:\t" << mc_vir_norm.at<double>(1,0)/((H.row(1)).dot(mc_ref_norm.t())) << std::endl;
			//std::cout << "cyan alpha z:\t" << mc_vir_norm.at<double>(2,0)/((H.row(2)).dot(mc_ref_norm.t())) << std::endl;
			
			//std::cout << "purple alpha x:\t" << mp_vir_norm.at<double>(0,0)/((H.row(0)).dot(mp_ref_norm.t())) << std::endl;
			//std::cout << "purple alpha y:\t" << mp_vir_norm.at<double>(1,0)/((H.row(1)).dot(mp_ref_norm.t())) << std::endl;
			//std::cout << "purple alpha z:\t" << mp_vir_norm.at<double>(2,0)/((H.row(2)).dot(mp_ref_norm.t())) << std::endl;
			
			//G = K*H*K.inv(cv::DECOMP_LU);
			
			//std::cout << "projective homography:\n" << G << std::endl;
			
			//// finding the alphas
			//std::cout << "red alpha x:\t" << pr_vir.at<double>(0,0)/((G.row(0)).dot(pr_ref.t())) << std::endl;
			//std::cout << "red alpha y:\t" << pr_vir.at<double>(1,0)/((G.row(1)).dot(pr_ref.t())) << std::endl;
			//std::cout << "red alpha z:\t" << pr_vir.at<double>(2,0)/((G.row(2)).dot(pr_ref.t())) << std::endl;
			
			//std::cout << "green alpha x:\t" << pg_vir.at<double>(0,0)/((G.row(0)).dot(pg_ref.t())) << std::endl;
			//std::cout << "green alpha y:\t" << pg_vir.at<double>(1,0)/((G.row(1)).dot(pg_ref.t())) << std::endl;
			//std::cout << "green alpha z:\t" << pg_vir.at<double>(2,0)/((G.row(2)).dot(pg_ref.t())) << std::endl;
			
			//std::cout << "cyan alpha x:\t" << pc_vir.at<double>(0,0)/((G.row(0)).dot(pc_ref.t())) << std::endl;
			//std::cout << "cyan alpha y:\t" << pc_vir.at<double>(1,0)/((G.row(1)).dot(pc_ref.t())) << std::endl;
			//std::cout << "cyan alpha z:\t" << pc_vir.at<double>(2,0)/((G.row(2)).dot(pc_ref.t())) << std::endl;
			
			//std::cout << "purple alpha x:\t" << pp_vir.at<double>(0,0)/((G.row(0)).dot(pp_ref.t())) << std::endl;
			//std::cout << "purple alpha y:\t" << pp_vir.at<double>(1,0)/((G.row(1)).dot(pp_ref.t())) << std::endl;
			//std::cout << "purple alpha z:\t" << pp_vir.at<double>(2,0)/((G.row(2)).dot(pp_ref.t())) << std::endl;
			
			//// finding the projective homography from findHomography
			//curr_points_p.push_back(cv::Point2f(pr_vir.at<double>(0,0),pr_vir.at<double>(1,0)));
			//curr_points_p.push_back(cv::Point2f(pg_vir.at<double>(0,0),pg_vir.at<double>(1,0)));
			//curr_points_p.push_back(cv::Point2f(pc_vir.at<double>(0,0),pc_vir.at<double>(1,0)));
			//curr_points_p.push_back(cv::Point2f(pp_vir.at<double>(0,0),pp_vir.at<double>(1,0)));
			
			//ref_points_p.push_back(cv::Point2f(pr_ref.at<double>(0,0),pr_ref.at<double>(1,0)));
			//ref_points_p.push_back(cv::Point2f(pg_ref.at<double>(0,0),pg_ref.at<double>(1,0)));
			//ref_points_p.push_back(cv::Point2f(pc_ref.at<double>(0,0),pc_ref.at<double>(1,0)));
			//ref_points_p.push_back(cv::Point2f(pp_ref.at<double>(0,0),pp_ref.at<double>(1,0)));
			
			//G_fH = cv::findHomography(ref_points_p,curr_points_p,0);
			
			//std::cout << "projective from find homography:\n" << G_fH << std::endl;
			
			//// finding the alphas
			//std::cout << "red alpha x:\t" << pr_vir.at<double>(0,0)/((G_fH.row(0)).dot(pr_ref.t())) << std::endl;
			//std::cout << "red alpha y:\t" << pr_vir.at<double>(1,0)/((G_fH.row(1)).dot(pr_ref.t())) << std::endl;
			//std::cout << "red alpha z:\t" << pr_vir.at<double>(2,0)/((G_fH.row(2)).dot(pr_ref.t())) << std::endl;
			
			//std::cout << "green alpha x:\t" << pg_vir.at<double>(0,0)/((G_fH.row(0)).dot(pg_ref.t())) << std::endl;
			//std::cout << "green alpha y:\t" << pg_vir.at<double>(1,0)/((G_fH.row(1)).dot(pg_ref.t())) << std::endl;
			//std::cout << "green alpha z:\t" << pg_vir.at<double>(2,0)/((G_fH.row(2)).dot(pg_ref.t())) << std::endl;
			
			//std::cout << "cyan alpha x:\t" << pc_vir.at<double>(0,0)/((G_fH.row(0)).dot(pc_ref.t())) << std::endl;
			//std::cout << "cyan alpha y:\t" << pc_vir.at<double>(1,0)/((G_fH.row(1)).dot(pc_ref.t())) << std::endl;
			//std::cout << "cyan alpha z:\t" << pc_vir.at<double>(2,0)/((G_fH.row(2)).dot(pc_ref.t())) << std::endl;
			
			//std::cout << "purple alpha x:\t" << pp_vir.at<double>(0,0)/((G_fH.row(0)).dot(pp_ref.t())) << std::endl;
			//std::cout << "purple alpha y:\t" << pp_vir.at<double>(1,0)/((G_fH.row(1)).dot(pp_ref.t())) << std::endl;
			//std::cout << "purple alpha z:\t" << pp_vir.at<double>(2,0)/((G_fH.row(2)).dot(pp_ref.t())) << std::endl;
			
			
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
			
			// initialize the transforms
			world_to_reference.setOrigin(tf::Vector3(0,0,z_ref));
			tf::Quaternion q_temp;
			q_temp.setEuler(0.0,M_PIl,-1.0*M_PIl/2.0);
			world_to_reference.setRotation(q_temp);
			
			// initialize the virtual transform
			reference_to_current.setOrigin(tf::Vector3(vir_T.at<double>(0,0), vir_T.at<double>(0,1), vir_T.at<double>(0,2)));
			vir_R_3x3[0][0] = vir_R.at<double>(0,0);
			vir_R_3x3[0][1] = vir_R.at<double>(0,1);
			vir_R_3x3[0][2] = vir_R.at<double>(0,2);
			vir_R_3x3[1][0] = vir_R.at<double>(1,0);
			vir_R_3x3[1][1] = vir_R.at<double>(1,1);
			vir_R_3x3[1][2] = vir_R.at<double>(1,2);
			vir_R_3x3[2][0] = vir_R.at<double>(2,0);
			vir_R_3x3[2][1] = vir_R.at<double>(2,1);
			vir_R_3x3[2][2] = vir_R.at<double>(2,2);
			vir_R_3x3.getRotation(vir_Q);
			reference_to_current.setRotation(vir_Q);
			
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
				first_run = true;
				last_virtual_update_time = ros::Time::now();
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
					first_run = true;
					last_virtual_update_time = ros::Time::now();
				}
			}
			else
			{
				res.success = false;
			}
			
			return true;
		}
		
		// get the depth estimate
		
		// update the virtual camera
		void virtual_camera_callback(const geometry_msgs::Twist& msg)
		{
			if (using_virtual_reference && reference_set)
			{
				// converting the command velocity to update the position
				///////////// rotation and translation from desired to the reference virtual camera /////////
				// update current time and get the time difference
				double current_time = ros::Time::now().toSec();
				double time_diff = current_time - last_virtual_update_time.toSec();
				std::cout << "time diff:\t" << time_diff << std::endl;
				
				// The coordinate frame of the body of the quad relative to the camera is xb = -yc, yb = -xc, and zb = -zc
				// angular zb is angular zc, that is angular zb is positive about zc, strange
				
				
				// update the virtual rotation matrix
				virtual_yaw = virtual_yaw + msg.angular.z*time_diff;
				
				//virtual_theta = 0;
				std::cout << "virtual yaw:\t" << virtual_yaw << std::endl;
				
				vir_R = cv::Mat::zeros(3,3,CV_64F);
				vir_R.at<double>(0,0) = cos(virtual_yaw);
				vir_R.at<double>(1,0) = sin(virtual_yaw);
				vir_R.at<double>(0,1) = -1.0*sin(virtual_yaw);
				vir_R.at<double>(1,1) = cos(virtual_yaw);
				vir_R.at<double>(2,2) = 1.0;
				std::cout << "virtual rotation:\n" << vir_R << std::endl;
								
				// update the virtual x, y, z
				//vir_T.at<double>(0,0) = -0.5;
				//vir_T.at<double>(1,0) = 0;
				//vir_T.at<double>(2,0) = 0;
				vir_T.at<double>(0,0) = vir_T.at<double>(0,0) + msg.linear.y*time_diff;
				vir_T.at<double>(1,0) = vir_T.at<double>(1,0) + msg.linear.x*time_diff;
				vir_T.at<double>(2,0) = vir_T.at<double>(2,0) + msg.linear.z*time_diff;
				std::cout << "virtual translation:\n" << vir_T << std::endl;
				
				// update the transform
				reference_to_current.setOrigin(tf::Vector3(vir_T.at<double>(0,0), vir_T.at<double>(0,1), vir_T.at<double>(0,2)));
				vir_R_3x3[0][0] = vir_R.at<double>(0,0);
				vir_R_3x3[0][1] = vir_R.at<double>(0,1);
				vir_R_3x3[0][2] = vir_R.at<double>(0,2);
				vir_R_3x3[1][0] = vir_R.at<double>(1,0);
				vir_R_3x3[1][1] = vir_R.at<double>(1,1);
				vir_R_3x3[1][2] = vir_R.at<double>(1,2);
				vir_R_3x3[2][0] = vir_R.at<double>(2,0);
				vir_R_3x3[2][1] = vir_R.at<double>(2,1);
				vir_R_3x3[2][2] = vir_R.at<double>(2,2);
				vir_R_3x3.getRotation(vir_Q);
				reference_to_current.setRotation(vir_Q);

				// new position is mi_vir_bar = vir_T + vir_R*mi_ref_bar
				// real coordinate values for the circles in the virtual
				mr_vir_bar = cv::Mat::zeros(3,1,CV_64F);
				mg_vir_bar = cv::Mat::zeros(3,1,CV_64F);
				mc_vir_bar = cv::Mat::zeros(3,1,CV_64F);
				mp_vir_bar = cv::Mat::zeros(3,1,CV_64F);
				
				// rotating and translating the reference to some pose
				mr_vir_bar = vir_T + vir_R*mr_ref_bar;
				mg_vir_bar = vir_T + vir_R*mg_ref_bar;
				mc_vir_bar = vir_T + vir_R*mc_ref_bar;
				mp_vir_bar = vir_T + vir_R*mp_ref_bar;
				
				//std::cout << "reference red:\n" << mr_ref_bar << std::endl;
				//std::cout << "reference green:\n" << mg_ref_bar << std::endl;
				//std::cout << "reference cyan:\n" << mc_ref_bar << std::endl;
				//std::cout << "reference purple:\n" << mp_ref_bar << std::endl;
				
				//std::cout << "virtual translation:\n" << vir_T << std::endl;
				//std::cout << "virtual rotation:\n" << vir_R << std::endl;
				//std::cout << "virtual red bar:\n" << mr_vir_bar << std::endl;
				//std::cout << "virtual green bar:\n" << mg_vir_bar << std::endl;
				//std::cout << "virtual cyan bar:\n" << mc_vir_bar << std::endl;
				//std::cout << "virtual purple bar:\n" << mp_vir_bar << std::endl;
				
				// normalizing
				mr_vir_norm = (1.0/mr_vir_bar.at<double>(2,0))*mr_vir_bar;
				mg_vir_norm = (1.0/mg_vir_bar.at<double>(2,0))*mg_vir_bar;
				mc_vir_norm = (1.0/mc_vir_bar.at<double>(2,0))*mc_vir_bar;
				mp_vir_norm = (1.0/mp_vir_bar.at<double>(2,0))*mp_vir_bar;
				
				//std::cout << "virtual red norm:\n" << mr_vir_norm << std::endl;
				//std::cout << "virtual green norm:\n" << mg_vir_norm << std::endl;
				//std::cout << "virtual cyan norm:\n" << mc_vir_norm << std::endl;
				//std::cout << "virtual purple norm:\n" << mp_vir_norm << std::endl;
				
				// as pixels
				pr_vir = K*mr_vir_norm;
				pg_vir = K*mg_vir_norm;
				pc_vir = K*mc_vir_norm;
				pp_vir = K*mp_vir_norm;
				
				//std::cout << "virtual red pixels:\n" << pr_vir << std::endl;
				//std::cout << "virtual green pixels:\n" << pg_vir << std::endl;
				//std::cout << "virtual cyan pixels:\n" << pc_vir << std::endl;
				//std::cout << "virtual purple pixels:\n" << pp_vir << std::endl;
				
				// red
				homog_circles_virtual.red_circle.x = pr_vir.at<double>(0,0);
				homog_circles_virtual.red_circle.y = pr_vir.at<double>(1,0);
				homog_circles_virtual.red_circle.z = pr_vir.at<double>(2,0);
				
				// green
				homog_circles_virtual.green_circle.x = pg_vir.at<double>(0,0);
				homog_circles_virtual.green_circle.y = pg_vir.at<double>(1,0);
				homog_circles_virtual.green_circle.z = pg_vir.at<double>(2,0);
				
				// cyan
				homog_circles_virtual.cyan_circle.x = pc_vir.at<double>(0,0);
				homog_circles_virtual.cyan_circle.y = pc_vir.at<double>(1,0);
				homog_circles_virtual.cyan_circle.z = pc_vir.at<double>(2,0);
				
				// purple
				homog_circles_virtual.purple_circle.x = pp_vir.at<double>(0,0);
				homog_circles_virtual.purple_circle.y = pp_vir.at<double>(1,0);
				homog_circles_virtual.purple_circle.z = pp_vir.at<double>(2,0);
				
				last_virtual_update_time = ros::Time::now();
			}
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
								red_circle_p_curr.x = final_circles[ii][0].x;
								red_circle_p_curr.y = final_circles[ii][0].y;
								red_circle_p_curr.z = 1;
								break;
								
							// green
							case 1:
								//std::cout << "green found" << std::endl;
								color_con = new cv::Scalar(0, 255, 0);
								// assigning the values of the circle to the circle
								green_circle_p_curr.x = final_circles[ii][0].x;
								green_circle_p_curr.y = final_circles[ii][0].y;
								green_circle_p_curr.z = 1;
								break;
								
							// cyan
							case 2:
								//std::cout << "cyan found" << std::endl;
								color_con = new cv::Scalar(255, 255, 0);
								// assigning the values of the circle to the circle
								cyan_circle_p_curr.x = final_circles[ii][0].x;
								cyan_circle_p_curr.y = final_circles[ii][0].y;
								cyan_circle_p_curr.z = 1;
								break;
								
							// purple
							case 3:
								//std::cout << "purple found" << std::endl;
								color_con = new cv::Scalar(255, 0, 255);
								// assigning the values of the circle to the circle
								purple_circle_p_curr.x = final_circles[ii][0].x;
								purple_circle_p_curr.y = final_circles[ii][0].y;
								purple_circle_p_curr.z = 1;
								break;
						}
						// if it is the first run will just use the value brought in and set up the last value for next time
						if (first_run)
						{
							first_run = false;

							// assigning the values of the circle to the circle
							red_circle_p = red_circle_p_curr;
							green_circle_p = green_circle_p_curr;
							cyan_circle_p = cyan_circle_p_curr;
							purple_circle_p = purple_circle_p_curr;
							
							// updating the last
							red_circle_p_last = red_circle_p;
							green_circle_p_last = green_circle_p;
							cyan_circle_p_last = cyan_circle_p;
							purple_circle_p_last = purple_circle_p;
								
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
							
							// updating the last
							
						}
												
						// draw all the contours
						cv::drawContours(blur_frame, contours[ii], -1, *color_con, 1);
						
						cv::circle( blur_frame, 
									cv::Point(final_circles[ii][0].x, final_circles[ii][0].y ),
									final_circles[ii][0].radius,
									*color_con,
									2
									);
					
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

	if (image_converter.using_virtual_reference)
	{
		ros::Rate loop_rate(100);
		while (ros::ok())
		{
			// giving the virtual circles to the output message
			image_converter.homog_circles_virtual.header.stamp = image_converter.last_virtual_update_time;
			image_converter.homog_circles_ref.header.stamp = image_converter.homog_circles_curr.header.stamp;
			image_converter.complete_msg.header.stamp = image_converter.homog_circles_curr.header.stamp;
			image_converter.complete_msg.reference_set = image_converter.reference_set;
			image_converter.complete_msg.current_points = image_converter.homog_circles_virtual;
			image_converter.complete_msg.reference_points = image_converter.homog_circles_ref;
			
			// publish the circle points
			image_converter.circle_pub.publish(image_converter.complete_msg);
			
			image_converter.br.sendTransform(tf::StampedTransform(image_converter.world_to_reference, image_converter.last_virtual_update_time
											 ,"world", "reference_image"));
			image_converter.br.sendTransform(tf::StampedTransform(image_converter.reference_to_current.inverse(), image_converter.last_virtual_update_time
											 ,"reference_image","current_image"));
			
			//reference_to_current;
			//std::cout << "complete message\n" << image_converter.complete_msg << std::endl << std::endl;
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else
	{
		// release and sleep until next time image is available
		ros::spin();
	}
    
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
