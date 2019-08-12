#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

float L_Slope;
float R_Slope;
bool IsRed;
bool IsYellow;
bool IsGreen;
int L_Loc;
int R_Loc;

int Servo_Value;
bool IsGo;

void bool_cb1(const std_msgs::Bool::ConstPtr& msg)
{
	IsRed=msg->data;
	ROS_INFO("yes_or_no: [%s]", msg->data ? "true" : "false");
}
void bool_cb2(const std_msgs::Bool::ConstPtr& msg)
{
	IsYellow=msg->data;
	ROS_INFO("yes_or_no: [%s]", msg->data ? "true" : "false");
}
void bool_cb3(const std_msgs::Bool::ConstPtr& msg)
{
	IsGreen=msg->data;
	ROS_INFO("yes_or_no: [%s]", msg->data ? "true" : "false");
}

void float_cb1(const std_msgs::Float32::ConstPtr& msg)
{
	L_Slope=msg->data;
	ROS_INFO("value: [%f]", msg->data);
}
void float_cb2(const std_msgs::Float32::ConstPtr& msg)
{
	R_Slope=msg->data;
	ROS_INFO("value: [%f]", msg->data);
}

void int_cb1(const std_msgs::Int32::ConstPtr& msg)
{
	L_Loc=msg->data;
	ROS_INFO("value: [%d]", msg->data);
}
void int_cb2(const std_msgs::Int32::ConstPtr& msg)
{
	R_Loc=msg->data;
	ROS_INFO("value: [%d]", msg->data);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Decision");

	ros::NodeHandle n;

	ros::Subscriber left_slope = n.subscribe("l_slope", 10, float_cb1);
	ros::Subscriber right_slope = n.subscribe("r_slope", 10, float_cb2);
	ros::Subscriber red_right = n.subscribe("isred", 10, bool_cb1);
	ros::Subscriber yellow_right = n.subscribe("isyellow", 10, bool_cb2);	
	ros::Subscriber green_right = n.subscribe("isgreen", 10, bool_cb3);	
	ros::Subscriber left_location = n.subscribe("l_location", 10, int_cb1);	
	ros::Subscriber right_location = n.subscribe("r_location", 10, int_cb2);	
	
	

	ros::Publisher servo_pub=n.advertise<std_msgs::Int32>("servo",10);
	ros::Publisher tl_pub=n.advertise<std_msgs::Bool>("traffic_light",10);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		std_msgs::Int32 servo;
		std_msgs::Bool tl;
		
		//right 10~14 center 15 left 16~20
	if(L_Slope < 0 && R_Slope > 9900)
	{
		if(L_Loc >= 0 && L_Loc <= 30)
				Servo_Value=15;
		else if(L_Loc >= 31 && L_Loc <= 99)
				Servo_Value=16;
		else if(L_Loc >= 100 && L_Loc <= 159)
				Servo_Value=17;
		else if(L_Loc >= 160 && L_Loc <= 210)
				Servo_Value=18;
	}
	if(R_Slope < 0)
	{
		if(R_Loc >= 321 && R_Loc <= 480)
				Servo_Value=19;
		else if(R_Loc >= 481 && R_Loc <= 640)
				Servo_Value=20;
	}

	if(R_Slope > 0 && R_Slope <9990 && L_Slope > 9900)
	{

		if(R_Loc >= 610 && R_Loc <= 640)
				Servo_Value=15;
		else if(R_Loc >= 541 && R_Loc <= 609)
				Servo_Value=14;
		else if(R_Loc >= 480 && R_Loc <= 540)
				Servo_Value=13;
		else if(R_Loc >= 430 && R_Loc <= 479)
				Servo_Value=12;
	}
	if(L_Slope > 0 && L_Slope <9990)
	{
		if(L_Loc >= 160 && L_Loc <= 319)
				Servo_Value=11;
		else if(L_Loc >= 0 && L_Loc <= 159)
				Servo_Value=10;
	}
	if(R_Slope > 9900 && L_Slope > 9900)
		Servo_Value=15;
	

	//red or yellow = stop, green or none = go

	if(IsRed == true || IsYellow == true)
		IsGo = false;
	else if(IsGreen == true)
		IsGo = true;
	else
		IsGo = true;

		servo.data=Servo_Value;
		tl.data=IsGo;

		servo_pub.publish(servo);
		tl_pub.publish(tl);
		std::printf("servo_value : %d\n",Servo_Value);

		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();



	return 0;
}
