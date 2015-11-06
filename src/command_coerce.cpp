ros::Subscriber joy_sub;
ros::Publisher takeoff_pub;
ros::Publisher land_pub;
ros::Publisher reset_pub;
ros::Publisher cmd_vel_pub;
joy_sub = nh.subscribe("joy",1,&QuadController::joy_callback,this);

// publishers to the takeoff message, land message, reset message, and velocity message
takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff",1);
land_pub = nh.advertise<std_msgs::Empty>("ardrone/land",1);
reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset",1);
cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
z_ref_pub = nh.advertise<std_msgs::Float64>("z_ref_hat",1);

// initially sending it a desired command of 0
cmd_vel_pub.publish(geometry_msgs::Twist());

/********** Begin xbox controller message **********/
double a_button_land_b0 = 0;
double b_button_reset_b1 = 0;
double y_button_takeoff_b3 = 0;
double lb_button_teleop_b4 = 0;
double rt_stick_ud_x_a3 = 0;
double rt_stick_lr_y_a2 = 0;
double lt_stick_ud_z_a1 = 0;
double lt_stick_lr_th_a0 = 0;
geometry_msgs::Twist command_from_xbox;

/********** callback for the controller **********/
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
		
	lb_button_teleop_b4 = msg.buttons[4];
	
	
	
	rt_stick_ud_x_a3 = joy_deadband(msg.axes[3]);
	command_from_xbox.linear.x = rt_stick_ud_x_a3;
	
	rt_stick_lr_y_a2 = joy_deadband(msg.axes[2]);
	command_from_xbox.linear.y = rt_stick_lr_y_a2;
	
	lt_stick_ud_z_a1 = joy_deadband(msg.axes[1]);
	command_from_xbox.linear.z = lt_stick_ud_z_a1;
	
	lt_stick_lr_th_a0 = joy_deadband(msg.axes[0]);
	command_from_xbox.angular.z = lt_stick_lr_th_a0;
	
	
	
}

/********** filters out the controller values **********/
double joy_deadband(double input_value)
{
	double filtered_value = 0;
	std::cout << " input " << input_value << std::endl;
	std::cout << " abs input " << std::abs(input_value) << std::endl;
	if (std::abs(input_value) > 0.11)
	{
		filtered_value = input_value;
	}
	std::cout << " output " << filtered_value << std::endl;
	return filtered_value;
}

/********** checks the command from the controller to see if it will send the quad out of bounds **********/
void command_coerce(double& x_min_w, double& x_max_w,double& y_min_w,double& y_max_w,double& x_curr_w, double& y_curr_w,double& last_time_diff, double& rt_stick_ud_x_a3_curr, double& rt_stick_lr_y_a2_curr, double& rt_stick_ud_x_a3_out, double& rt_stick_lr_y_a2_out)
{
	// gains for the x and y controller to velocity
	double kxp = 1;
	double kyp = 1;
	
	// getting the future x and y values from the current locations and the current commands
	double x_des_w = x_curr_w + kxp*rt_stick_ud_x_a3_curr*last_time_diff;
	double y_des_w = y_curr_w + kyp*rt_stick_lr_y_a2_curr*last_time_diff;
	
	// checking if the commands exceed the upper or lower bounds and if it does sending out a command of 0 otherwise sending out the command
	// checking the x command
	if (x_des_w >= x_max || x_des_w <= x_min)
	{
		rt_stick_ud_x_a3_out = 0;
	}
	else
	{
		rt_stick_ud_x_a3_out = rt_stick_ud_x_a3_curr;
	}
	
	// checking the y command
	if (y_des_w >= y_max || y_des_w <= y_min)
	{
		rt_stick_lr_y_a2_out = 0;
	}
	else
	{
		rt_stick_lr_y_a2_out = rt_stick_lr_y_a2_curr;
	}
}
