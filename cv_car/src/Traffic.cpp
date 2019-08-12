#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <time.h>
#include <std_msgs/Bool.h>

#define PI 3.1415926

static const std::string OPENCV_WINDOW = "Lane Image window";

bool isred=false, isyellow=false, isgreen=false;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera_raw", 1, &ImageConverter::imageCb, this);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	cv::Mat raw_src;
	cv::Mat src;
	cv::Mat colorsrc;
	cv::Mat roi;
	cv::Mat coloroi;
	cv::Mat roi_g; //gaussian
	cv::Mat result_g; //gaussian
	cv::Mat result_c; //canny
	cv::Mat result_c1; //canny
	cv::Mat result_c2;
	int blue=0, green=0, red=0;

	cv::cvtColor(cv_ptr->image, raw_src, CV_BGR2GRAY);
	cv::GaussianBlur(raw_src, src, cv::Size(7,7), 0.0);
	//cv::cvtColor(src, colorsrc, CV_GRAY2RGB);
	//cv::cvtColor(colorsrc1, colorsrc2, CV_RGB2BGR);
	colorsrc=cv_ptr->image.clone();

	//src=cv_ptr->image;
	//src=cv_ptr->image.clone();
	
	//cv::GaussianBlur(src, src_g, cv::Size(5,5), 2);
	//cv::GaussianBlur(src, src_g, cv::Size(5,5), 2);

	roi = src(cv::Rect(0, 0, 640,src.rows/2));
	coloroi = colorsrc(cv::Rect(0, 0, 640, colorsrc.rows/2));

	//cv::GaussianBlur(roi, roi_g, cv::Size(5,5), 2);

	cv::Canny(roi, result_c, 80, 140);

	//cv::cvtColor(roi, roi, CV_RGB2GRAY);

	std::vector<cv::Vec3f> circle;
	//cv::HoughCircles(roi, circle, CV_HOUGH_GRADIENT, 2, 50, 200, 100, 25, 100);
	cv::HoughCircles(roi, circle, CV_HOUGH_GRADIENT, 1, 50);
	cv::Mat result_0(src.size(), CV_8UC3);
	//cv::cvtColor(src, result_0, CV_GRAY2BGR);

	cv::Vec3f params;
	int cx,cy,r;
	for(int k=0; k<circle.size(); k++)
	{
		params=circle[k];
		cx=cvRound(params[0]);
		cy=cvRound(params[1]);
		r=cvRound(params[2]);

		//printf("circle[%2d]:(cx,cy)=(%d,%d),r=%d\n",k,cx,cy,r);
		//printf("red=%d, green=%d, blue=%d\n",red,green,blue);
		std::cout<<"circle["<<k<<"]:"<<"("<<cx<<","<<cy<<")"<<" r="<<r<<std::endl;

		blue = coloroi.at<cv::Vec3b> (cv::Point(cx,cy))[0];
		green =  coloroi.at<cv::Vec3b> (cv::Point(cx,cy))[1];	
		red =  coloroi.at<cv::Vec3b> (cv::Point(cx,cy))[2];

		std::cout<<"red:"<<red<<" green:"<<green<<" blue:"<<blue<<std::endl;

		if(red > 200 && green < 91 && blue < 91)
		{
			isred=true;
			isyellow=false;
			isgreen=false;
		}
		if(red > 180 && green > 180 && blue < 51)
		{
			isred=false;
			isyellow=true;
			isgreen=false;
		}		
		if(red < 101 && green > 150 && blue < 51)
		{
			isred=false;
			isyellow=false;
			isgreen=true;
		}

		cv::Point center(cx, cy);
		cv::circle(coloroi, center, r, cv::Scalar(0,0,255), 2);
	}
	//default(no signal light)
	if(circle.size() == 0)
	{
			isred=false;
			isyellow=false;
			isgreen=false;
	}
/*cv_ptr->image*/ 

	//cv::cvtColor(result_c, result_0, CV_GRAY2BGR);

	cv::imshow(OPENCV_WINDOW, coloroi /*result_0*/);
	cv::waitKey(3);

  }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Traffic");
	ros::NodeHandle s;
	ros::Publisher red_light_pub=s.advertise<std_msgs::Bool>("isred",10);
	ros::Publisher yellow_light_pub=s.advertise<std_msgs::Bool>("isyellow",10);
	ros::Publisher green_light_pub=s.advertise<std_msgs::Bool>("isgreen",10);
	ros::Rate loop_rate(5);
	
	ImageConverter ic;
	while(ros::ok())
	{
		std_msgs::Bool r_light;
		std_msgs::Bool y_light;
		std_msgs::Bool g_light;
		r_light.data=isred;
		y_light.data=isyellow;
		g_light.data=isgreen;
		red_light_pub.publish(r_light);
		yellow_light_pub.publish(y_light);
		green_light_pub.publish(g_light);
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}
