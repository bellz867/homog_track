#include <iostream>
#include <cmath>
#include <ros/ros.h>

int main(int argc, char** argv)
{   
	ros::init(argc,argv,"nodetastic");
	
	//ros::Rate loop_rate(1);
	
	//while (ros::ok())
	//{

		
		//ros::spinOnce();
		//loop_rate.sleep();
	//}
	
	for (int ii = 0; ii < 9; ii++)
	{
		std::cout << "column: " << ii%3 << " row: " << ii/3 << std::endl;
	}
	
	ros::spin();
    return 0;
}
