#include <iostream>
#include <string>
#include <vector>

// ros and opencv includes for using opencv and ros
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>


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
        
        // Constructor for when passed values are known
        Circle(float self_radius, float self_x, float self_y);
        
        // Constructor for when the passed in values are unknown
        Circle();

        // function to set circle
        void setCircle(float self_radius, float self_x, float self_y);

        // function to print circle
        void printCircle();
        
        // deconstructor, called automatically
        ~Circle();
};

// first constructor callback
Circle::Circle(float self_radius, float self_x, float self_y)
{
    radius = self_radius;
    x = self_x;
    y = self_y;
}

// second constructor callback
Circle::Circle()
{
    radius = 0;
    x = 0;
    y = 0;
}

// set callback
void Circle::setCircle(float self_radius, float self_x, float self_y)
{
    radius = self_radius;
    x = self_x;
    y = self_y;
}

// function to print circle
void Circle::printCircle()
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
Circle::~Circle()
{
    // std::cout << "\n\tCircle Destroyed\n" << std::endl;
}

// class to convert images
class ImageConverter
{

  
public:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	// image pointer for the converted images
	cv_bridge::CvImagePtr cv_ptr;
	
	// constructor for the image converter
	ImageConverter() : it_(nh_)
	{
		//// Subscribe to input video feed from bluefox
		//image_sub_ = it_.subscribe( "/mv_BF001066/image_raw",
									//1, 
									//&ImageConverter::image_callback,
									//this);
									
		// subscribe to ardrone front camera
		image_sub_ = it_.subscribe( "/ardrone/front/image_raw",
									1, 
									&ImageConverter::image_callback,
									this);
									
		// publish processed image from homography
		image_pub_ = it_.advertise("homog_image", 1);
		
	}
	
	// destructor for the image converter
	~ImageConverter()
	{

	}

	// callback for the image subscriber
	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		// trying to initialize the ptr to the image passed in and
		// converting it to a bgr8 image so opencv can use it
		// if it fails will through the error
		try
		{
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		  std::cout << "image converted" << std::endl;
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		//// Draw an example circle on the video stream
		//if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		  //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

		//// Update GUI Window
		//cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		//cv::waitKey(3);

		//// Output modified video stream
		//image_pub_.publish(cv_ptr->toImageMsg());
	}
};

// argc is the number of things passed in
// argv is the actual things passed in
// to access the thing in argv use the [] indexing style
int main(int argc, char** argv)
{   
	/********** Inialize Node **********/    
	ros::init(argc,argv,"homog_node");
	
	/********** Inialize Image Converter **********/    
	ImageConverter image_converter;

    /********** Begin Matrix Declarations **********/    
    const int height = 640;
    const int width = 480;
    
    // frame will be a 3 channel 8 bit image
    cv::Mat frame(height,width,CV_8UC3);
    
    // blur frame is the same as frame
    cv::Mat blur_frame(height,width,CV_8UC3);
    
    // hsv is the same as frame
    cv::Mat hsv_frame(height,width,CV_8UC3);
    
    // primary binary frame is a 1 channel 8 bit image
    cv::Mat p_binary_frame(height,width,CV_8UC1);
    
    // secondary binary frame is same as primary
    cv::Mat s_binary_frame(height,width,CV_8UC1);

    // putting the binary frames into a vector
    std::vector<cv::Mat> binary_frames{p_binary_frame, s_binary_frame};
    
    /********** End Matrix Declarations **********/    
    

    /********** Begin Kernel Declarations **********/
    // kernel size for blurring
    int blur_kernel_size = 7;
    
    // kernel size for erode
    int erode_kernel_size = 17;

    // kernel size for dilate
    int dilate_kernel_size = 15;

    // getting the structuring element for the erode
    cv::Mat erode_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                       cv::Size(  erode_kernel_size,
                                                                  erode_kernel_size ),
                                                       cv::Point( -1, -1) 
                                                     );
    
    // getting the structuring element for the dilate 
    cv::Mat dilate_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                        cv::Size( dilate_kernel_size,
                                                                  dilate_kernel_size ),
                                                        cv::Point( -1, -1) 
                                                      );

    /********** End Kernel Declarations **********/


    /********** Begin Primary and secondary threshold declarations and initializations **********/

    // number of threshold values
    const int thresh_length = 6;

    // starting values for the primary color threshold values: hue, staturation, and value
    // order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
    int p_thresh_start[thresh_length] = {165, 6, 138, 255, 145, 255};

    // current values for the secondary trackbars, initially set to the threshold start values
    // order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
    int p_thresh_current[thresh_length];

    // starting values for the secondary color threshold values: hue, staturation, and value
    // order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
    int s_thresh_start[thresh_length] = {30, 66, 85, 255, 0, 255};

    // current values for the secondary trackbars, initially set to the threshold start values
    // order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
    int s_thresh_current[thresh_length];

    // putting the start arrays into a vector
    std::vector<int*> thresh_start{ p_thresh_start, s_thresh_start };

    // putting the current arrays into a vector
    std::vector<int*> thresh_current{ p_thresh_current, s_thresh_current };

    // initializing the current values to be the starting value
    // need to do it twice, once for primary and once for secondary
    for (int jj = 0; jj < 2; jj++)
    {
        for (int ii = 0; ii < thresh_length; ii++)
        {
            thresh_current[jj][ii] = thresh_start[jj][ii];
        }        
    }

    // max values for the primary colorthreshold value: hue, saturation, and value
    // order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
    int thresh_max[thresh_length] = {180, 180, 255, 255, 255, 255};

    /********** End Primary and secondary threshold declarations and initializations **********/


    /********** Begin Decarations for the contours and the shapes **********/

    // primary contour
    std::vector< std::vector<cv::Point>> p_contours;
    
    // secondary contour
    std::vector< std::vector<cv::Point>> s_contours;
    
    // putting the contours into a vector
    std::vector< std::vector< std::vector<cv::Point> > > contours{p_contours, s_contours};

    // primary vector of areas
    std::vector<float> p_areas;

    // secondary vector of areas
    std::vector<float> s_areas;

    // putting the areas into a vector
    std::vector< std::vector<float> > areas{p_areas, s_areas };

    // names for the contours
    std::vector<std::string> contour_names{ "P Contour", "S Contour" };

    // enclosing circle radius and center for the contours
    // order is (radius, center x, center y)
    Circle *temp_circle = new Circle;

    // temp_circle->printCircle();

    // primary circles
    std::vector<Circle> p_circles1;
    
    // secondary circles
    std::vector<Circle> s_circles1;

    // primary circles
    std::vector<Circle> p_circles2;
    
    // secondary circles
    std::vector<Circle> s_circles2;

    // // print out radius
    // std::cout << temp_circle->radius;

    // putting them in a vector
    std::vector< std::vector<Circle> > temp_circles{p_circles1, s_circles1};

    // final circles
    std::vector< std::vector<Circle> > final_circles{p_circles2, s_circles2};

    // tells how many circles desired for the final circle iteration
    int desired_number;

    // tells how many circles have been found
    int current_number;

    // tells which index is currently the greatest
    int current_index;

    // tells which index is the compare index
    int current_compare;

    // tells when the current compare has exceeded the length of the vector
    bool end_reached;

    /********** End Decarations for the contours and the shapes **********/

	ros::Rate loop_rate(30);

    /********** Begin Main Loop **********/
    while (ros::ok())
    {
		// checking to see if an image has come in
		if (!image_converter.cv_ptr)
		{
			std::cout << "no image" << std::endl;
			ros::spinOnce();
	        loop_rate.sleep();
			continue;
		}
		
		// getting the new frame
        frame = image_converter.cv_ptr->image;
        
        // median blur on the current frame
        cv::medianBlur( frame, blur_frame, blur_kernel_size );
        
        // converting the frame to hsv
        cv::cvtColor(blur_frame,hsv_frame,CV_BGR2HSV);

        // filtering the image for the  different colors
        // need to do it twice, once for primary and once for secondary
        for (int ii = 0; ii < 2; ii++)
        {
            filter_image_for_color( hsv_frame,
                                    binary_frames[ii],
                                    thresh_current[ii],
                                    thresh_max,
                                    erode_element,
                                    dilate_element 
                                  );
        }
        
        /********** finding all the contours then the max 1 or 3 radius circles **********/

        for (int ii = 0; ii < 2; ii++)
        {

            cv::findContours(binary_frames[ii].clone(), contours[ii], CV_RETR_LIST, CV_CHAIN_APPROX_NONE);    
            
            // centers of all the contours circles
            std::vector<cv::Point2f> center( contours[ii].size() );

            // radius of all the contour circles
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

            /********** getting the max 1 or 3 circles depending on if it primary or secondary**********/
            
            // resetting the search values
            current_number = 0;
            current_index = 0;
            current_compare = 0;
            end_reached = false;
            desired_number = 0;

            // determing if it is primary or secondary if it is not empty
            if (!temp_circles[ii].empty())
            {
                if (ii == 0)
                {
                    desired_number = 1;
                }
                
                else
                {
                    desired_number = 3;
                }
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

                    // resetting end reached
                    end_reached = false;
                }
            }

            // erasing all the temp circles
            temp_circles[ii].erase(temp_circles[ii].begin(),temp_circles[ii].end());
        }
             
        // color for the drawing
        cv::Scalar* color_con = new cv::Scalar();

        /********** ordering the secondary circles clockwise from the primary **********/
        

        
        /********** drawing the circles and the contours **********/
        for (int ii = 0; ii < 2; ii++)
        {
            cv::drawContours(frame, contours[ii], -1, cv::Scalar( 0, 0, 0), 1);

            for (int jj = 0; jj < final_circles[ii].size(); jj++)
            {
                if (ii == 0)
                {
                    color_con = new cv::Scalar(0, 0, 255);
                }
                else if ( (jj==0) && (ii==1) )
                {
                    color_con = new cv::Scalar(255, 255, 0);
                }
                else if ( (jj==1) && (ii==1) )
                {
                    color_con = new cv::Scalar(0, 255, 0);
                }
                else if ( (jj==2) && (ii==1) )
                {
                    color_con = new cv::Scalar(255, 0, 0);
                }

                cv::circle(frame, 
                           cv::Point(final_circles[ii][jj].x, final_circles[ii][jj].y ),
                           final_circles[ii][jj].radius,
                           *color_con,
                           3
                          );

                final_circles[ii][jj].printCircle();
            }
        }
        std::cout << "\n";
        
        // erasing all the circles
        final_circles[0].erase(final_circles[0].begin(),final_circles[0].end());
        final_circles[1].erase(final_circles[1].begin(),final_circles[1].end());
        
        
        image_converter.image_pub_.publish(image_converter.cv_ptr->toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    
    // killing all the windows
    cv::destroyAllWindows();
    
    return 0;
}

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

        // cv::imshow("show inside",bin_src);
}
