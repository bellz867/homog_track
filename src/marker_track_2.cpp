#include <iostream>
#include <string>
#include <vector>

// ros and opencv includes for using opencv and ros
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
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
	unsigned int new_image_recieved;

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
		
		// tells if recieved a new image#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
		new_image_recieved = 0;
		
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
		  new_image_recieved = 1;
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
	ros::NodeHandle n;
	
	/********** Inialize the publisher for the 4 circles **********/
	// the order goes red, green, cyan, purple
	ros::Publisher circles_xy_pub = n.advertise<std_msgs::Float32MultiArray>("homog_circles_xy",1);
	
	// vector for the publisher initializing to 0
	std_msgs::Float32MultiArray circles_xy;
	for (int i = 0; i < 8; i++)
	{
		circles_xy.data.push_back(0);
	}
	
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
    
    // red binary frame is a 1 channel 8 bit image
    cv::Mat r_binary_frame(height,width,CV_8UC1);
    
    // green binary frame is same as red
    cv::Mat g_binary_frame(height,width,CV_8UC1);

	// cyan binary frame is same as red
    cv::Mat c_binary_frame(height,width,CV_8UC1);
    
    // purple binary frame is same as red
    cv::Mat p_binary_frame(height,width,CV_8UC1);

    // putting the binary frames into a vector
    std::vector<cv::Mat> binary_frames{r_binary_frame, g_binary_frame, c_binary_frame, p_binary_frame};
    
    /********** End Matrix Declarations **********/    
    

    /********** Begin Kernel Declarations **********/
    // kernel size for blurring
    int blur_kernel_size = 7;
    
    // kernel size for erode
    int erode_kernel_size = 3;

    // kernel size for dilate
    int dilate_kernel_size = 3;

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

	// order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
    // values for the red color threshold values: hue, staturation, and value
    int r_thresh[thresh_length] = {175, 5, 100, 255, 50, 255};

    // values for the green color threshold values: hue, staturation, and value
    int g_thresh[thresh_length] = {70, 85, 100, 255, 0, 255};

	// values for the cyan color threshold values: hue, staturation, and value
    int c_thresh[thresh_length] = {90, 110, 50, 255, 0, 255};
    
    // values for the violet color threshold values: hue, staturation, and value
    int p_thresh[thresh_length] = {125, 140, 50, 255, 0, 255};

    // putting the start arrays into a vector
    std::vector<int*> thresh_current{ r_thresh, g_thresh, c_thresh, p_thresh };

    // max values for the primary colorthreshold value: hue, saturation, and value
    // order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
    int thresh_max[thresh_length] = {180, 180, 255, 255, 255, 255};

    /********** End Primary and secondary threshold declarations and initializations **********/


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

	// desired number of circles to find
	int desired_number;

    // tells when the current compare has exceeded the length of the vector
    bool end_reached;

    /********** End Decarations for the contours and the shapes **********/

    /********** Begin Decarations for circle locations publisher **********/
    
    
    /********** End Decarations for circle locations publisher **********/


	ros::Rate loop_rate(30);

    /********** Begin Main Loop **********/
    while (ros::ok())
    {
		// checking to see if an image has come in
		if (!image_converter.cv_ptr || image_converter.new_image_recieved == 0)
		{
			std::cout << "no image" << std::endl;
			ros::spinOnce();
	        loop_rate.sleep();
			continue;
		}
		
		// restting the new image boolean if it was equal 50to 1
		image_converter.new_image_recieved = 0;
		
		// getting the new frame
        frame = image_converter.cv_ptr->image;
        
        // median blur on the current frame
        cv::medianBlur( frame, blur_frame, blur_kernel_size );
        
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
			// draw all the contours
			cv::drawContours(frame, contours[ii], -1, cv::Scalar( 0, 0, 0), 1);
			
			// drawing the circles if there is one
			if (!final_circles[ii].empty())
			{
				// color for the drawing
				cv::Scalar* color_con = new cv::Scalar();
				
				// choosing which color to print based on the circle
				switch (ii)
				{
					// red
					case 0:
						std::cout << "red" << std::endl;
						color_con = new cv::Scalar(0, 0, 255);
						break;
					// green
					case 1:
						std::cout << "greapien" << std::endl;
						color_con = new cv::Scalar(0, 255, 0);
						break;
					// cyan
					case 2:
						std::cout << "cyan" << std::endl;
						color_con = new cv::Scalar(255, 255, 0);
						break;
					// purple
					case 3:
						std::cout << "purple" << std::endl;
						color_con = new cv::Scalar(255, 0, 255);
						break;
				}
			

				cv::circle( frame, 
							cv::Point(final_circles[ii][0].x, final_circles[ii][0].y ),
							final_circles[ii][0].radius,
							*color_con,
							3
							);
			
				// printing out the circles values
				final_circles[ii][0].printCircle();
				
				// putting the circle into its corresponding place xy
				// red is 2*(ii=0),2*(ii=0)+1
				// green is 2*(ii=1),2*(ii=1)+1
				// cyan is 2*(ii=2),2*(ii=2)+1
				// purple is 2*(ii=3),2*(ii=3)+1
				circles_xy.data[2*ii] = final_circles[ii][0].x;
				circles_xy.data[2*ii+1] = final_circles[ii][0].y;
			}
			// if the circle is missing will make its location -1 to indicate
			else
			{
				// putting the circle into its corresponding place xy
				// red is 2*(ii=0),2*(ii=0)+1
				// green is 2*(ii=1),2*(ii=1)+1
				// cyan is 2*(ii=2),2*(ii=2)+1
				// purple is 2*(ii=3),2*(ii=3)+1
				circles_xy.data[2*ii] = -1;
				circles_xy.data[2*ii+1] = -1;
			}
                
        }
        std::cout << "\n";
        
        // publish the processed image
        image_converter.image_pub_.publish(image_converter.cv_ptr->toImageMsg());
        circles_xy_pub.publish(circles_xy);
        
        // erasing all the circles and restting the circles_xy to 0
        for (int ii = 0; ii < 4; ii++)
        {
			final_circles[ii].erase(final_circles[ii].begin(),final_circles[ii].end());
		}
		
		// clearing the circles_xy
		circles_xy.data.clear();
		
		// initializing circles_xy to 0 again
		for (int i = 0; i < 8; i++)
		{
			circles_xy.data.push_back(0);
		}
        
        // release and sleep
        ros::spinOnce();
        loop_rate.sleep();   
    }
    
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

}
