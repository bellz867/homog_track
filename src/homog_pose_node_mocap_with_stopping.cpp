#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

// Namespaces
using namespace std;
using namespace cv;

// Prototypes

void groundTruthCB(const geometry_msgs::PoseStamped::ConstPtr&,const geometry_msgs::PoseStamped::ConstPtr&,const geometry_msgs::PoseStamped::ConstPtr&);
void poseEstimate(const sensor_msgs::ImageConstPtr&);
int choose_sol(vector<Mat>, vector<Mat>, vector<Mat>);
void joy_callback(const sensor_msgs::Joy&);
double joy_deadband(double);
void command_coerce(double, double,double,double,double,double,double, double&, double&, double&, double&);
vector<vector<Point2f> > findInliers(vector<Point2f>, vector<Point2f>, vector<uchar>, vector<float>);
void reset();

// Variables

// camera matrix
Mat cam_calib_mat = cv::Mat::zeros(3,3,CV_64F);
	
//  distortion matrix for the ardrone
std::vector<double> dC;

Mat n_c = (Mat_<double>(3, 1) << 0, 0, 1),//normal vector
init_pose = Mat::eye(4,4,CV_64F), //initial pose of cam
k_pose = Mat::eye(4,4,CV_64F),//key pose of cam
f_pose = Mat::eye(4, 4, CV_64F), //current frame pose of cam
ground_truth = Mat::eye(4,4,CV_64F), //ground truth pose of cam
frame, distorted_frame,c_gray, p_gray, h, c_pose;
vector<Mat> rot,trans,normals;
vector<Point2f> c_points,p_points;
vector<Mat> past_pose,past_pose_gt;
vector<uchar> status;
vector<float> err;
vector<int> good_idx;
TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 200, 0.03);
Size subPixWinSize(5, 5), winSize(5, 5);
double f_dist = -1,up_x,low_x,up_y,low_y;
vector<double> distance_travelled = {0,0,0};
bool start = true;
bool start_cv = false;
bool reset_b = false;
cv_bridge::CvImagePtr cv_ptr;
tf::StampedTransform cam_transform,ugv0_transform,ugv1_transform,cam_gt;

/********** Begin xbox controller message **********/
double a_button_land_b0 = 0;
double b_button_reset_b1 = 0;
double y_button_takeoff_b3 = 0;
double lb_button_teleop_b4 = 0;
double rb_button_teleop_b5 = 0;
double rt_stick_ud_x_a3 = 0;
double rt_stick_lr_y_a2 = 0;
double lt_stick_ud_z_a1 = 0;
double lt_stick_lr_th_a0 = 0;
geometry_msgs::Twist command_from_xbox;

//subscribe and publish joystick
ros::Subscriber joy_sub;
ros::Publisher takeoff_pub;
ros::Publisher land_pub;
ros::Publisher reset_pub;
ros::Publisher cmd_vel_pub;


// Start main program
int main(int argc, char** argv)
{
  cout << "Start of program..." << endl;
  // Initialize ROS
  ros::init(argc, argv, "homog_pose_mocap");
  ros::NodeHandle n;
  ros::NodeHandle pose_n;
  tf::TransformListener listener;

  cout << "ROS handles initialized..." << endl;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub;
  
  cv::namedWindow("View");
  cv::startWindowThread();
  ros::Rate loop_rate(100);
  
  // setup camera info
  	cam_calib_mat.at<double>(0,0) = 567.79;
	cam_calib_mat.at<double>(0,2) = 337.35;
	cam_calib_mat.at<double>(1,1) = 564.52;
	cam_calib_mat.at<double>(1,2) = 169.54;
	cam_calib_mat.at<double>(2,2) = 1;
	dC.push_back(-0.5122398601387984);
	dC.push_back(0.2625218940944695);
	dC.push_back(-0.0009892579344331395);
	dC.push_back(0.002480111502028633);
	dC.push_back(0.0);
  
  //subscribe image
  std::string image_topic = "/ardrone/front/image_raw";
  image_sub = it.subscribe(image_topic, 1, poseEstimate);
  cout << "Subscribing to image topic: " << image_topic << endl;

joy_sub = n.subscribe("joy",1,&joy_callback);

// publishers to the takeoff message, land message, reset message, and velocity message
takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff",1);
land_pub = n.advertise<std_msgs::Empty>("ardrone/land",1);
reset_pub = n.advertise<std_msgs::Empty>("ardrone/reset",1);
cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);


// initially sending it a desired command of 0
cmd_vel_pub.publish(geometry_msgs::Twist());
   
past_pose_gt.push_back(Mat::eye(4,4,CV_64F));
  
  
   
  
    // transform broadcaster
    tf::Transform transform;
    tf::Matrix3x3 rotm;
    tf::TransformBroadcaster br;
    double last_time = ros::Time::now().toSec();
    while(ros::ok())
    {
		Mat rvec,tvec;
        Mat g_pose = init_pose*f_pose;
		
		tf::Vector3 g_trans(g_pose.at<double>(0,3),g_pose.at<double>(1,3),g_pose.at<double>(2,3));
		tf::Matrix3x3 g_rot(g_pose.at<double>(0,0),g_pose.at<double>(0,1),g_pose.at<double>(0,2),
							g_pose.at<double>(1,0),g_pose.at<double>(1,1),g_pose.at<double>(1,2),
							g_pose.at<double>(2,0),g_pose.at<double>(2,1),g_pose.at<double>(2,3));
		tf::Quaternion g_q;
		g_rot.getRotation(g_q);
		tf::Transform transform;
		transform.setOrigin(g_trans);
		transform.setRotation(g_q);
		
		ros::Time g_time = ros::Time::now();
		
        br.sendTransform(tf::StampedTransform(transform, g_time,"world","cam"));
       //br.sendTransform(tf::StampedTransform(cam_transform, g_time,"world","drone_gt"));

		command_coerce(low_x, up_x, low_y , up_y, g_pose.at<double>(0,3),g_pose.at<double>(1,3), g_time.toSec()-last_time,
						command_from_xbox.linear.x ,command_from_xbox.linear.y, command_from_xbox.linear.x ,command_from_xbox.linear.y);

		// initially sending it a desired command of 0
		cmd_vel_pub.publish(command_from_xbox);

		last_time = g_time.toSec();
        ros::spinOnce();
        loop_rate.sleep();
        
    }

    ros::spin();
    return 0;
}

// Ground truth callback
void groundTruthCB(const geometry_msgs::PoseStamped::ConstPtr& ardrone_msg, const geometry_msgs::PoseStamped::ConstPtr& c1_msg, const geometry_msgs::PoseStamped::ConstPtr& c2_msg)
{
}


// Visual estimation callback
void poseEstimate(const sensor_msgs::ImageConstPtr& msg)
{
	if(!start_cv) 
	{

		return;
	}
	if(reset_b)
	{
		reset_b = false;
		p_gray.release();
	}
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    distorted_frame = cv_ptr->image.clone();
	cv::undistort(distorted_frame,frame,cam_calib_mat,dC);
    if (frame.empty())
    {
        cout << "End of stream." << endl;
        return;
    }
    cvtColor(frame, c_gray, CV_RGB2GRAY);
    if (countNonZero(c_gray) < 1) return;


  tf::TransformListener listener;
		listener.waitForTransform("/world", "/image", ros::Time::now(), ros::Duration(0.02));
		listener.lookupTransform("/world", "/image", ros::Time(0), cam_gt);

	
	cout << cam_gt.getOrigin().getX() << endl;
    //First time startup.
    if (p_gray.empty())
    {
        cout << "p_gray is empty" << endl;
        goodFeaturesToTrack(c_gray, c_points, 100, 0.1, 5, noArray(),11,false);
        cornerSubPix(c_gray, c_points, subPixWinSize, Size(-1, -1), termcrit);
        p_gray = c_gray.clone();
        p_points = c_points;
        return;
    }

    //Perform optical flow.
    calcOpticalFlowPyrLK(p_gray, c_gray, p_points, c_points, status, err, winSize, 3, termcrit, OPTFLOW_LK_GET_MIN_EIGENVALS, 0.001);

    //Take out lost points from p_points in order to perform homography.
    vector<vector<Point2f> > inliers = findInliers(p_points,c_points,status,err);
    p_points = inliers[0];
    c_points = inliers[1];
    vector<Point2f> p_inliers = p_points;
    vector<Point2f> c_inliers = c_points;

    if (p_points.size() > 4) //Make sure there are enough points for calculating homography.
    {
        //Find homography that takes x'(pixel coord of current set) to x(pixel coord of past set), such that x = H * x'.
        h = findHomography(c_inliers, p_inliers, RANSAC, 0.5);
        decomposeHomographyMat(h, cam_calib_mat, rot, trans, normals);

        if (rot.size() < 4) return;

        // choose the correct soln base on distance.
        good_idx.clear();

        // take out negative normals (z-axis < 0).
        for (int i = 0; i < 4; i++)
        {
            Mat m1 = cam_calib_mat.inv()*(Mat_<double>(3,1) << p_points[1].x, p_points[1].y, 1);
            Mat Rn = rot[i] * normals[i];
            if (m1.dot(Rn) > 0) //Rn.at<double>(2) > 0 //normals[i].at<double>(2) > 0
            {
                good_idx.push_back(i);
            }
        }

        // Choose closest solution to previous solution
        Mat t1, t2;
        hconcat(rot[good_idx[0]], trans[good_idx[0]]*f_dist, t1);
        vconcat(t1, vector < double > {0, 0, 0, 1}, t1);

        hconcat(rot[good_idx[1]], trans[good_idx[1]]*f_dist, t2);
        vconcat(t2, vector < double > {0, 0, 0, 1}, t2);


        if (normals[good_idx[0]].dot(n_c) > normals[good_idx[1]].dot(n_c)) // trans[0].dot(normals[0]) - f_dist < trans[1].dot(normals[1]) - f_dist 
        //if (norm(k_pose*t1 - k_pose, 2) < norm(k_pose*t2 - k_pose, 2))
        {
            good_idx.erase(good_idx.begin() + 1); //good_idx.erase(1);
            f_pose = k_pose*t1;
        }   
        else
        {
            good_idx.erase(good_idx.begin());
            f_pose = k_pose*t2;
        }
    }
    else
    {
        cout << "Lost track... Reinitializing!" << endl;
    }
    //Repopulate features if too few left or off center
    
    double cx=0,cy=0;
    for(int i = 0; i < c_points.size(); i++)
    {
        cx += c_points[i].x;
        cy += c_points[i].y;
    }
    cx = cx/c_points.size();
    cy = cy/c_points.size();
    
    Point2f center(cx,cy);
    
    if (c_points.size() < (frame.size().height / 100)*(frame.size().width / 100) 
        || (abs(center.x-frame.size().width/2) > frame.size().width /4 || abs(center.y-frame.size().height/2) > frame.size().height /4) || start == true)
    {
		start = false;
        //cout << "Too few features... Reinitializing" << endl;
        goodFeaturesToTrack(c_gray, c_points, 100, 0.1, 5, noArray(),11,false);
        cornerSubPix(c_gray, c_points, subPixWinSize, Size(-1, -1), termcrit);
        k_pose = f_pose;
        n_c = normals[good_idx[0]];
        Mat trans2 = trans[good_idx[0]]*f_dist;
        f_dist = f_dist + trans2.dot(normals[good_idx[0]]);
        past_pose.push_back(k_pose);
        
          tf::Matrix3x3 gt_rot(cam_gt.getRotation());

    Mat past_pose_gt_cur = Mat::eye(4,4,CV_64F);
  past_pose_gt_cur.at<double>(0,0) = gt_rot.getRow(0).getX();
  past_pose_gt_cur.at<double>(0,1) = gt_rot.getRow(0).getY();
  past_pose_gt_cur.at<double>(0,2) = gt_rot.getRow(0).getZ();
  past_pose_gt_cur.at<double>(0,3) = cam_gt.getOrigin().getX();
  past_pose_gt_cur.at<double>(1,0) = gt_rot.getRow(1).getX();
  past_pose_gt_cur.at<double>(1,1) = gt_rot.getRow(1).getY();
  past_pose_gt_cur.at<double>(1,2) = gt_rot.getRow(1).getZ();
  past_pose_gt_cur.at<double>(1,3) = cam_gt.getOrigin().getY();
  past_pose_gt_cur.at<double>(2,0) = gt_rot.getRow(2).getX();
  past_pose_gt_cur.at<double>(2,1) = gt_rot.getRow(2).getY();
  past_pose_gt_cur.at<double>(2,2) = gt_rot.getRow(2).getZ();
  past_pose_gt_cur.at<double>(2,3) = cam_gt.getOrigin().getZ();
  past_pose_gt_cur.at<double>(3,0) = 0;
  past_pose_gt_cur.at<double>(3,1) = 0;
  past_pose_gt_cur.at<double>(3,2) = 0;
  past_pose_gt_cur.at<double>(3,3) = 1;
  past_pose_gt.push_back(past_pose_gt_cur);

    }

    // Drawing stuffs
    Mat display = frame.clone();

    if (!rot.empty() && !good_idx.empty())
    {
        // Construct pose
        Mat rvec,tvec;
        Mat g_pose = init_pose*f_pose;
        Rodrigues(g_pose(Range(0,3),Range(0,3)), rvec);
        tvec = g_pose(Range(0, 3), Range(3, 4));
        Affine3f cam_pose(Vec3f(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), Vec3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

		/*
        cout << "X trans: "<<f_pose.at<double>(0,3) << endl;
		cout << "Y trans: "<<f_pose.at<double>(1,3) << endl;
		cout << "Z trans: "<<f_pose.at<double>(2,3) << endl;
		*/
		
		/*
        cout << "X: " << tvec.at<double>(0) << endl;
        cout << "Y: " << tvec.at<double>(1) << endl;
        cout << "Z: " << tvec.at<double>(2) << endl;
        cout << "d: " << f_dist << endl;
		*/
		
		

	
		// Calculate error
		double total_distance = 0;
		for(int i = 1; i < past_pose_gt.size();i++)
		{
			total_distance += sqrt(pow((past_pose_gt[i].at<double>(0,3)-past_pose_gt[i-1].at<double>(0,3)),2.0)
									+pow((past_pose_gt[i].at<double>(1,3)-past_pose_gt[i-1].at<double>(1,3)),2.0)
									+pow((past_pose_gt[i].at<double>(2,3)-past_pose_gt[i-1].at<double>(2,3)),2.0));
		}
		total_distance += sqrt(pow((cam_gt.getOrigin().getX()-past_pose_gt.back().at<double>(0,3)),2.0)
									+pow((cam_gt.getOrigin().getY()-past_pose_gt.back().at<double>(1,3)),2.0)
									+pow((cam_gt.getOrigin().getZ()-past_pose_gt.back().at<double>(2,3)),2.0));
									
								
						cout << cam_gt.getOrigin().getX()<< endl;			
		double ex = (g_pose.at<double>(0,3)-cam_gt.getOrigin().getX())/total_distance;
		double ey = (g_pose.at<double>(1,3)-cam_gt.getOrigin().getY())/total_distance;
		double ez = (g_pose.at<double>(2,3)-cam_gt.getOrigin().getZ())/total_distance;
		
		cout << "X error: " << ex*100 << " %" << endl;
		cout << "Y error: " << ey*100 << " %" << endl;
		cout << "Z error: " << ez*100 << " %" << endl;
		
		if(abs(ex) > 0.2 || abs(ey) > 0.2 || abs(ez) > 0.2)
		{
			reset();
			}
		


        //if(tvec.at<double>(0)>up_x || tvec.at<double>(0)<low_x || tvec.at<double>(1)>up_y || tvec.at<double>(1)<low_y) //box design
        if((tvec.at<double>(0) < up_x && tvec.at<double>(1) < low_y) || (tvec.at<double>(0) < up_x && tvec.at<double>(1)> up_y) )//launch zone + passage
        {
            //cout << "Out of bounds..." << endl;
            cv::line(display,Point(0,0),Point(display.size().width,display.size().height),Scalar(0,0,255),5,8,0);
            cv::line(display,Point(0,display.size().height),Point(display.size().width,0),Scalar(0,0,255),5,8,0);
        }

    }

    for (int i = 0; i < c_points.size(); i++)
    {
        circle(display, c_points[i], 3, Scalar(0, 255, 0), -1, 8);
    }
    imshow("View", display);
    char action = cv::waitKey(10);

    switch (action)
    {
        case 'i': //initialize
            break;
        case 'r': //reinitialize
            break;
        default:
            break;
      
    }


    // Update data 
    p_gray = c_gray.clone();
    p_points = c_points;


    return;
}

int choose_sol(vector<Mat>rot,vector<Mat> trans, vector<Mat> normals)
{
    int idx;
    for (int i = 0; i < 4; i++)
    {
        if (normals[i].at<double>(0,2) < 0) continue;
        else
        {

        }
    }
    return idx;
}
vector<vector<Point2f> > findInliers(vector<Point2f> p_points, vector<Point2f> c_points, vector<uchar> status, vector<float> err)
{
    vector<vector<Point2f> > inliers;
    vector<Point2f> p_inliers, c_inliers;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] == 1 && err[i] < 5)
        {
            p_inliers.push_back(p_points[i]);
            c_inliers.push_back(c_points[i]);
        }
    }
    inliers.push_back(p_inliers);
    inliers.push_back(c_inliers);
    return inliers;
}


// joystick callback
void joy_callback(const sensor_msgs::Joy& msg)
{

	// cleaning the xbox twist message
	command_from_xbox = geometry_msgs::Twist();

	// breaking out the values
	a_button_land_b0 = msg.buttons[0];
	if (a_button_land_b0 > 0)
		land_pub.publish(std_msgs::Empty());
	
	b_button_reset_b1 = msg.buttons[1];
	if (b_button_reset_b1 > 0)
		reset_pub.publish(std_msgs::Empty());
	
	y_button_takeoff_b3 = msg.buttons[3];
	if (y_button_takeoff_b3 > 0)
		takeoff_pub.publish(std_msgs::Empty());
		

	rb_button_teleop_b5 = msg.buttons[5];
	
	if(start_cv && rb_button_teleop_b5)
	{
		reset();
	}
	
	lb_button_teleop_b4 = msg.buttons[4];
	
	if(!start_cv && lb_button_teleop_b4 == 1)
	{
		start_cv = true;
		
		
		
  if(f_dist = -1)
  {
	  
	    // Initialize transform listener
  tf::TransformListener listener;
  listener.waitForTransform("/world", "/image", ros::Time::now(), ros::Duration(1.0));
  listener.lookupTransform("/world", "/image", ros::Time(0), cam_transform);
  listener.lookupTransform("/world", "/ugv0", ros::Time(0), ugv0_transform);
  listener.lookupTransform("/world", "/ugv1", ros::Time(0), ugv1_transform);
  
  tf::Matrix3x3 init_rot(cam_transform.getRotation());
  //initialize d
  f_dist = cam_transform.getOrigin().getZ();
  
  //initialize pose
    
  init_pose.at<double>(0,0) = init_rot.getRow(0).getX();
  init_pose.at<double>(0,1) = init_rot.getRow(0).getY();
  init_pose.at<double>(0,2) = init_rot.getRow(0).getZ();
  init_pose.at<double>(0,3) = cam_transform.getOrigin().getX();
  init_pose.at<double>(1,0) = init_rot.getRow(1).getX();
  init_pose.at<double>(1,1) = init_rot.getRow(1).getY();
  init_pose.at<double>(1,2) = init_rot.getRow(1).getZ();
  init_pose.at<double>(1,3) = cam_transform.getOrigin().getY();
  init_pose.at<double>(2,0) = init_rot.getRow(2).getX();
  init_pose.at<double>(2,1) = init_rot.getRow(2).getY();
  init_pose.at<double>(2,2) = init_rot.getRow(2).getZ();
  init_pose.at<double>(2,3) = cam_transform.getOrigin().getZ();
  init_pose.at<double>(3,0) = 0;
  init_pose.at<double>(3,1) = 0;
  init_pose.at<double>(3,2) = 0;
  init_pose.at<double>(3,3) = 1;
  
  Mat init_rot_mat = init_pose(Range(0,3),Range(0,3));
  Mat z_w = (Mat_<double>(3, 1) << 0, 0, -1); // -Z of the world frame
  n_c = init_rot_mat.inv()*z_w; // define n in camera frame for finding correct homography decomposition.
  cout << n_c << endl;
  
  //set bounds
  if(ugv0_transform.getOrigin().getX()>ugv1_transform.getOrigin().getX())
  {
    up_x = ugv0_transform.getOrigin().getX();
    low_x = ugv1_transform.getOrigin().getX();
  }
  else
  {
    up_x = ugv1_transform.getOrigin().getX();
    low_x = ugv0_transform.getOrigin().getX();
  }
   if(ugv0_transform.getOrigin().getY()>ugv1_transform.getOrigin().getY())
  {
    low_y = ugv1_transform.getOrigin().getY();
    up_y = ugv0_transform.getOrigin().getY();
  }
  else
  {
    low_y = ugv0_transform.getOrigin().getY();
    up_y = ugv1_transform.getOrigin().getY();
  }
  
  cout << "Initial d: " << f_dist << endl;
  cout << "Initial pose: " << init_pose << endl;
  cout << "Upper X: " << up_x << endl;
  cout << "Lower X: " << low_x << endl;
  cout << "Upper Y: " << up_y << endl;
  cout << "Lower Y: " << low_y << endl; 
  }
	} 
	
	rt_stick_ud_x_a3 = joy_deadband(msg.axes[3]);
	command_from_xbox.linear.x = 0.1*rt_stick_ud_x_a3;
	
	rt_stick_lr_y_a2 = joy_deadband(msg.axes[2]);
	command_from_xbox.linear.y = 0.1*rt_stick_lr_y_a2;
	
	lt_stick_ud_z_a1 = joy_deadband(msg.axes[1]);
	command_from_xbox.linear.z = 0.1*lt_stick_ud_z_a1;
	
	lt_stick_lr_th_a0 = joy_deadband(msg.axes[0]);
	command_from_xbox.angular.z = 0.1*lt_stick_lr_th_a0;
	
}

// joystick deadband
double joy_deadband(double input_value)
{
	double filtered_value = 0;
	//std::cout << " input " << input_value << std::endl;
	//std::cout << " abs input " << std::abs(input_value) << std::endl;
	if (std::abs(input_value) > 0.11)
	{
		filtered_value = input_value;
	}
	//std::cout << " output " << filtered_value << std::endl;
	return filtered_value;
}

// zone force
void command_coerce(double x_min_w, double x_max_w,double y_min_w,double y_max_w,double x_curr_w,double y_curr_w,double last_time_diff, double& x_command_current, double& y_command_current, double& x_command_out, double& y_command_out)
{
	// gains for the x and y controller to velocity
	double kxp = 1;
	double kyp = 1;
	
	// getting the future x and y values from the current locations and the current commands
	double x_des_w = x_curr_w + kxp*x_command_current*last_time_diff;
	double y_des_w = y_curr_w + kyp*y_command_current*last_time_diff;
	
	/*
	cout << "x_des_w: " << x_des_w << endl;
	cout << "y_des_w: " << y_des_w << endl;
	cout << "x_curr: " << x_curr_w << endl;
	cout << "y_curr: " << y_curr_w << endl;
	* */
		
	// checking if the commands exceed the upper or lower bounds and if it does sending out a command of 0 otherwise sending out the command
	// checking the x command
	if (x_des_w >= x_max_w || x_des_w <= x_min_w)
	{
		//cout << "x command should be zero" << endl;
		//x_command_out = 0;
	}
	else
	{
		x_command_out = x_command_current;
	}
	
	// checking the y command
	if (y_des_w >= y_max_w || y_des_w <= y_min_w)
	{
		//cout << "y command should be zero" << endl;
		//y_command_out = 0;
	}
	else
	{
		y_command_out = y_command_current;
	}
}
void reset()
{
	reset_b = true;
	n_c = (Mat_<double>(3, 1) << 0, 0, 1),//normal vector
	init_pose = Mat::eye(4,4,CV_64F), //initial pose of cam
	k_pose = Mat::eye(4,4,CV_64F),//key pose of cam
	f_pose = Mat::eye(4, 4, CV_64F), //current frame pose of cam
	ground_truth = Mat::eye(4,4,CV_64F), //ground truth pose of cam	
	distance_travelled = {0,0,0};
	rot.clear(),trans.clear(),normals.clear();
	c_points.clear(),p_points.clear();
past_pose.clear(),past_pose_gt.clear();
status.clear();
err.clear();
good_idx.clear();
past_pose_gt.push_back(Mat::eye(4,4,CV_64F));


	    // Initialize transform listener
  tf::TransformListener listener;
  listener.waitForTransform("/world", "/image", ros::Time::now(), ros::Duration(0.1));
  listener.lookupTransform("/world", "/image", ros::Time(0), cam_transform);
  listener.lookupTransform("/world", "/ugv0", ros::Time(0), ugv0_transform);
  listener.lookupTransform("/world", "/ugv1", ros::Time(0), ugv1_transform);
  
  tf::Matrix3x3 init_rot(cam_transform.getRotation());
  //initialize d
  f_dist = cam_transform.getOrigin().getZ();
  
  //initialize pose
    
  init_pose.at<double>(0,0) = init_rot.getRow(0).getX();
  init_pose.at<double>(0,1) = init_rot.getRow(0).getY();
  init_pose.at<double>(0,2) = init_rot.getRow(0).getZ();
  init_pose.at<double>(0,3) = cam_transform.getOrigin().getX();
  init_pose.at<double>(1,0) = init_rot.getRow(1).getX();
  init_pose.at<double>(1,1) = init_rot.getRow(1).getY();
  init_pose.at<double>(1,2) = init_rot.getRow(1).getZ();
  init_pose.at<double>(1,3) = cam_transform.getOrigin().getY();
  init_pose.at<double>(2,0) = init_rot.getRow(2).getX();
  init_pose.at<double>(2,1) = init_rot.getRow(2).getY();
  init_pose.at<double>(2,2) = init_rot.getRow(2).getZ();
  init_pose.at<double>(2,3) = cam_transform.getOrigin().getZ();
  init_pose.at<double>(3,0) = 0;
  init_pose.at<double>(3,1) = 0;
  init_pose.at<double>(3,2) = 0;
  init_pose.at<double>(3,3) = 1;
  
  Mat init_rot_mat = init_pose(Range(0,3),Range(0,3));
  Mat z_w = (Mat_<double>(3, 1) << 0, 0, -1); // -Z of the world frame
  n_c = init_rot_mat.inv()*z_w; // define n in camera frame for finding correct homography decomposition.
  cout << n_c << endl;

}

