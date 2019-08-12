#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#define PI 3.1415926

static const std::string OPENCV_WINDOW = "Lane Image window";

int servo_value=13;

float left_slope=0, right_slope=0; //slope
int l_location=0, r_location=0; //location

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed 
    image_sub_ = it_.subscribe("/camera_raw", 1, &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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


	cv::Mat src;
	cv::Mat roi;
	cv::Mat roi_l;
	cv::Mat roi_r;

	cv::Mat result_c;//canny
	cv::Mat result_c1; 
	cv::Mat result_c2;

	cv::cvtColor(cv_ptr->image, src, CV_BGR2GRAY);
	roi = src(cv::Rect(0, (src.rows/5)*4,640,src.rows/5));
	//roi_l = src(cv::Rect(0, (src.rows/5)*4, 250, src.rows/5));
	//roi_r = src(cv::Rect(390, (src.rows/5)*4, 250, src.rows/5));


	cv::Canny(roi, result_c, 80, 120);
	//cv::Canny(roi_l, result_c1, (src.rows+src.cols)/4, (src.rows+src.cols)/2);
	//cv::Canny(roi_r, result_c2, (src.rows+src.cols)/4, (src.rows+src.cols)/2);


	std::vector<cv::Vec4f> lines;
	std::vector<cv::Vec4f> lines1;
	std::vector<cv::Vec4f> lines2;

	cv::HoughLinesP(result_c, lines, 1, CV_PI/180.0, 20, 10, 20);
	//cv::HoughLinesP(result_c1, lines1, 1, CV_PI/180.0, 5,10,5);
	//cv::HoughLinesP(result_c2, lines2, 1, CV_PI/180.0, 5,10,5);

	cv::Mat result(result_c.size(), CV_8UC3);
	//cv::Mat result_1(result_c1.size(), CV_8UC3);
	//cv::Mat result_2(result_c2.size(), CV_8UC3);

	cv::cvtColor(result_c, result, CV_GRAY2BGR);
	//cv::cvtColor(result_c1, result_1, CV_GRAY2BGR);
	//cv::cvtColor(result_c2, result_2, CV_GRAY2BGR);

	//std::cout<<"lines.size()="<<lines1.size()+lines2.size()<<std::endl;

	cv::Vec4f params;
	cv::Vec4f params1;
	cv::Vec4f params2;
	int x1_1, x2_1, y1_1, y2_1; //left lane
	int x1_1s=0, x2_1s=0, y1_1s=0, y2_1s=0; //left lane sum
	int x1_1a=0, x2_1a=0, y1_1a=0, y2_1a=0; //left lane avg
	int x1_2, x2_2, y1_2, y2_2; //right lane
	int x1_2s=0, x2_2s=0, y1_2s=0, y2_2s=0; //right lane sum
	int x1_2a=0, x2_2a=0, y1_2a=0, y2_2a=0; //right lane avg
	int left_num=0;	//number of left line
	int right_num=0; //number of right line
	
	for(int k=0; k<lines.size(); k++)
	{
		params=lines[k];

		if(params[0] < 320 && params[2] < 320)	//left
		{
			x1_1=params[0];
			y1_1=params[1];
			x2_1=params[2];
			y2_1=params[3];
			x1_1s=x1_1s+x1_1;
			y1_1s=y1_1s+y1_1;
			x2_1s=x2_1s+x2_1;
			y2_1s=y2_1s+y2_1;
			left_num++;
		}

		else if(params[2] > 320 && params[0] >320)	//right
		{
			x1_2=params[0];
			y1_2=params[1];
			x2_2=params[2];
			y2_2=params[3];
			x1_2s=x1_2s+x1_2;
			y1_2s=y1_2s+y1_2;
			x2_2s=x2_2s+x2_2;
			y2_2s=y2_2s+y2_2;
			right_num++;
		}

	}
	if(left_num!=0)
	{
		x1_1a=x1_1s/left_num; 
		x2_1a=x2_1s/left_num; 
		y1_1a=y1_1s/left_num;
		y2_1a=y2_1s/left_num;
	}
	if(right_num!=0)
	{
		x1_2a=x1_2s/right_num;
		x2_2a=x2_2s/right_num;
		y1_2a=y1_2s/right_num;
		y2_2a=y2_2s/right_num;
	}
	if(left_num == 0)
	{
		x1_1a=0; 
		x2_1a=0; 
		y1_1a=0;
		y2_1a=0;
	}
	if(right_num == 0)
	{
		x1_2a=0;
		x2_2a=0;
		y1_2a=0;
		y2_2a=0;
	}
	cv::Point pt1_l(x1_1a, y1_1a), pt2_l(x2_1a,y2_1a);
	cv::Point pt1_r(x1_2a, y1_2a), pt2_r(x2_2a,y2_2a);

	cv::line(result, pt1_l, pt2_l, cv::Scalar(0,0,255),2);
	cv::line(result, pt1_r, pt2_r, cv::Scalar(0,255,0),2);

	if((x2_1a-x1_1a) != 0)
	{
		left_slope= (((float)y2_1a-(float)y1_1a)/((float)x2_1a-(float)x1_1a));
	}
	if((x2_2a-x1_2a) != 0)
	{
		right_slope=(((float)y2_2a-(float)y1_2a)/((float)x2_2a-(float)x1_2a));
	}
	if((x2_1a-x1_1a) == 0)
		left_slope = 9999;

	if((x2_2a-x1_2a) == 0)
		right_slope = 9999;

	l_location = x1_1a;
	r_location = x2_2a;

	std::printf("left slope=%f\n", left_slope);
	std::printf("right slope=%f\n", right_slope);
	std::printf("location:%d %d\n",l_location, r_location);
	std::printf("num=%d %d %d\n", left_num, right_num, lines.size());


	cv::imshow(OPENCV_WINDOW, result);
	cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Lane");
  ros::NodeHandle s;
  ros::Publisher l_slope_pub=s.advertise<std_msgs::Float32>("l_slope",10);
  ros::Publisher r_slope_pub=s.advertise<std_msgs::Float32>("r_slope",10);
  ros::Publisher l_loc_pub=s.advertise<std_msgs::Int32>("l_location",10);
  ros::Publisher r_loc_pub=s.advertise<std_msgs::Int32>("r_location",10);
  ros::Rate loop_rate(10);
  ImageConverter ic;
  while(ros::ok())
  {
	std_msgs::Float32 l_msg;
	std_msgs::Float32 r_msg;
	std_msgs::Int32 l_loc;
	std_msgs::Int32 r_loc;

	l_msg.data=left_slope;
	r_msg.data=right_slope;
	l_loc.data=l_location;
	r_loc.data=r_location;

	l_slope_pub.publish(l_msg);
	r_slope_pub.publish(r_msg);
	l_loc_pub.publish(l_loc);
	r_loc_pub.publish(r_loc);

	ros::spinOnce();
	loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
