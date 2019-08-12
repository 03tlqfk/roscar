#include <stdio.h>
#include "softPwm.h"
#include "wiringPi.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"


int servo_val;

void servoCB(const std_msgs::Int32::ConstPtr& msg)
{
	servo_val=msg->data;
}

int main(int argc, char **argv)
{
	const int servo_pin=1;
	
	char input;
	
	wiringPiSetup();

	pinMode(servo_pin, PWM_OUTPUT);

	softPwmCreate(servo_pin,0,300);

	ros::init(argc,argv, "servo_node");
	ros::NodeHandle sn;
	ros::Rate rate(10);

	ros::Subscriber val=sn.subscribe("servo",10,servoCB);

	//digitalWrite(IN1, HIGH);
	//digitalWrite(IN2, LOW);

	while(ros::ok())
	{
		softPwmWrite(servo_pin, servo_val);
		if(servo_val<15)
		{
			printf("right %d\n", servo_val);
		}
		else if(servo_val>15)
		{
			printf("left %d\n", servo_val);
		}
		else if(servo_val==15)
		{
			printf("straight %d\n", servo_val);
		}
		
		ros::spinOnce();
		rate.sleep();
	}



	return 0;
}
