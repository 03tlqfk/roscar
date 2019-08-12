#include <stdio.h>
#include "softPwm.h"
#include "wiringPi.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

bool tl;

void tl_CB(const std_msgs::Bool::ConstPtr& msg)
{
	tl=msg->data;
}

int main(int argc, char **argv)
{
	const int INA=24;
	const int IN1=27;
	const int IN2=23;

	//traffic_light

	
	wiringPiSetup();

	ros::init(argc,argv, "dc_node");
	ros::NodeHandle sn;
	

	pinMode(INA, PWM_OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);

	ros::Rate rate(10);
	ros::Subscriber val=sn.subscribe("traffic_light",10,tl_CB);


	while(ros::ok())
	{
		

		if(tl == true)
		{
			printf("go\n");
			digitalWrite(IN1, HIGH);
			digitalWrite(IN2, LOW);
			pwmWrite(INA, 360);
		}
		else if(tl == false)
		{
			printf("stop\n");
			digitalWrite(IN1, LOW);
			digitalWrite(IN2, LOW);
			pwmWrite(INA, 0);
		}
		ros::spinOnce();
		rate.sleep();
	}

	return 0;

}
