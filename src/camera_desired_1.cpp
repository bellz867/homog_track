
#include <iostream>
#include <string>
#include <vector>

// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <homog_track/HomogMarker.h>
#include <homog_track/HomogComplete.h>
#include <homog_track/HomogDecomposed.h>
#include <homog_track/HomogDesired.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

class DesiredCamera
{
	public:	
		// node handle
		ros::NodeHandle nh;
		
		// bag for the desired location in regulation tracking
		rosbag::Bag bag;
		
		// current sum of the pose
		std::vector<geometry_msgs::Pose> desired_poses;
		
		// number of pose messages
		int number_poses;
		
		// desired pose
		geometry_msgs::Pose desired_pose;
		
		// pose averages
		std::vector<double> pose_avgs;
		
		// current sum of the red pixel location
		std::vector<geometry_msgs::Point32> desired_red_circles;
		
		// number of samples for the red circle
		int number_red_circles;
		
		// circle averages
		std::vector<double> circle_avgs;
		
		// point message for the desired reference
		geometry_msgs::Point32 desired_red_circle;
		
		// desired message
		homog_track::HomogDesired desired_camera;
		
		// publisher
		ros::Publisher desired_camera_pub;
		
		DesiredCamera()
		{
			// opening the bag and getting the topics
			bag.open("/home/zack/bag_files/move_nxny.bag", rosbag::bagmode::Read);
			std::vector<std::string> topics;
			topics.push_back(std::string("/decomposed_homography")); 
			topics.push_back(std::string("/complete_homog_set"));
			rosbag::View view(bag, rosbag::TopicQuery(topics));
			
			// running through the bag and putting the pose and red circle position into a container
			foreach(rosbag::MessageInstance const m, view)
			{
				homog_track::HomogDecomposed::ConstPtr s = m.instantiate<homog_track::HomogDecomposed>();
				if (s != NULL)
				{
					desired_poses.push_back(s->pose);
					std::cout << "pose\n" << s->pose << std::endl << std::endl;
				}
					
				homog_track::HomogComplete::ConstPtr i = m.instantiate<homog_track::HomogComplete>();
				if (i != NULL)
				{
					desired_red_circles.push_back(i->current_points.red_circle);
					std::cout << "red marker\n" << i->current_points.red_circle << std::endl << std::endl;
				}
			}
			bag.close();
			
			// getting the average for the x and y values for the circle
			number_red_circles = desired_red_circles.size();
			
			// initialze the circle averages
			circle_avgs.push_back(0);
			circle_avgs.push_back(0);
			
			// getting the sum
			for (geometry_msgs::Point32 ii : desired_red_circles)
			{
				circle_avgs[0] += ii.x;
				circle_avgs[1] += ii.y;
			}
			std::cout << "x sum\t" << circle_avgs[0] << std::endl;
			std::cout << "y sum\t" << circle_avgs[1] << std::endl;
			std::cout << "number circles\t" << number_red_circles << std::endl;
			
			// averaging
			circle_avgs[0] /= number_red_circles;
			circle_avgs[1] /= number_red_circles;
			
			// giving it to the desired circle
			desired_red_circle.x = circle_avgs[0];
			desired_red_circle.y = circle_avgs[1];
			desired_red_circle.z = 1;
			std::cout << "desired red circle\n" << desired_red_circle << std::endl;
			
			// getting the averages for the pose
			number_poses = desired_poses.size();
			
			// initialize to zero
			for (int ii = 0; ii < 7; ii++)
				pose_avgs.push_back(0);
			
			// getting the sum
			for (geometry_msgs::Pose ii : desired_poses)
			{
				pose_avgs[0] += ii.position.x;
				pose_avgs[1] += ii.position.y;
				pose_avgs[2] += ii.position.z;
				pose_avgs[3] += ii.orientation.x;
				pose_avgs[4] += ii.orientation.y;
				pose_avgs[5] += ii.orientation.z;
				pose_avgs[6] += ii.orientation.w;
			}
			std::cout << "x position sum\t" << pose_avgs[0] << std::endl;
			std::cout << "y position sum\t" << pose_avgs[1] << std::endl;
			std::cout << "z position sum\t" << pose_avgs[2] << std::endl;
			std::cout << "x orient sum\t" << pose_avgs[3] << std::endl;
			std::cout << "y orient sum\t" << pose_avgs[4] << std::endl;
			std::cout << "z orient sum\t" << pose_avgs[5] << std::endl;
			std::cout << "w orient sum\t" << pose_avgs[6] << std::endl;
			std::cout << "number poses\t" << number_poses << std::endl;
			
			// averaging
			for (int ii = 0; ii < pose_avgs.size(); ii++)
				pose_avgs[ii] /= number_poses;
			
			// giving it to the desired pose
			desired_pose.position.x = pose_avgs[0];
			desired_pose.position.y = pose_avgs[1];
			desired_pose.position.z = pose_avgs[2];
			desired_pose.orientation.x = pose_avgs[3];
			desired_pose.orientation.y = pose_avgs[4];
			desired_pose.orientation.z = pose_avgs[5];
			desired_pose.orientation.w = pose_avgs[6];
			
			// giving the pose and the circle to the message
			desired_camera.header.stamp = ros::Time::now();
			desired_camera.pose = desired_pose;
			desired_camera.red_circle = desired_red_circle;
			
			std::cout << "desired camera\n" << desired_camera << std::endl;
			
			// publish the value
			desired_camera_pub = nh.advertise<homog_track::HomogDesired>("desired_camera",1);
			
		}
};

// main
int main(int argc, char** argv)
{   
	// initialize node
	ros::init(argc,argv,"desired_camera_node");
	
	// initialize the controller    
	DesiredCamera desired_camera_;

	ros::Rate loop_rate(30);
    
    while(ros::ok())
    {
		desired_camera_.desired_camera_pub.publish(desired_camera_.desired_camera);
		
		// release 
		ros::spinOnce();
		loop_rate.sleep();
	}

    
    return 0;
}
