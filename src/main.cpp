/************************************************************************
* Copyright(c) 2013  Yang Xian
* All rights reserved.
*
* File:	main.cpp
* Brief: marker检测识别
* Version: 1.0
* Author: Yang Xian
* Email: yang_xian521@163.com
* Date:	2013/1/29 12:17
* History:
************************************************************************/
#include "MarkerDetector.h"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <bitset>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

using namespace cv;

static const char WINDOW[]="RGB Image";
static const char WINDOW2[]="Gray Image";

Mat img_gray;
Mat img_rgb;

bool flag_getnewimage = false;

geometry_msgs::Twist twist_msg;
ros::Publisher pub_empty_takeoff;
std_msgs::Empty emp_msg;
geometry_msgs::Twist twist_msg_hover;

void process(const sensor_msgs::ImageConstPtr& cam_image)
{
    cv_bridge::CvImagePtr cv_ptr;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(cam_image,sensor_msgs::image_encodings::BGR8);
	}

	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception:%s",e.what());
	  return;
	}

	img_rgb = cv_ptr->image;


	cvtColor(img_rgb,img_gray,CV_RGB2GRAY);

	flag_getnewimage = true;
}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"droneTest");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    ros::Publisher pub_twist;
    image_transport::Subscriber image_sub = it.subscribe("/quadrotor/ardrone/bottom/ardrone/bottom/image_raw",1,process);
    pub_twist = n.advertise<geometry_msgs::Twist>("/quadrotor/cmd_vel", 1);
    pub_empty_takeoff = n.advertise<std_msgs::Empty>("/quadrotor/ardrone/takeoff", 1);
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Vector3>("chatter", 1000);
    geometry_msgs::Vector3 msg_marker;
    geometry_msgs::Vector3 msg_marker_angle;
	MarkerDetector markCapture;
	Mat frame;

	ros::Rate loop_rate(30);//频率很重要，不能大于30Hz

	double start_time;
	start_time =(double)ros::Time::now().toSec();

	twist_msg_hover.linear.x=0.0; 
	twist_msg_hover.linear.y=0.0;
	twist_msg_hover.linear.z=0.0;
	twist_msg_hover.angular.x=0.0; 
	twist_msg_hover.angular.y=0.0;
	twist_msg_hover.angular.z=0.0;  

	// 起飞
	while ((double)ros::Time::now().toSec()< start_time + 20.0)
	{ //takeoff

		pub_empty_takeoff.publish(emp_msg); 
		pub_twist.publish(twist_msg_hover); 

		ros::spinOnce();
		loop_rate.sleep();
	}//while takeoff

	while (ros::ok())
	{

	 	if (flag_getnewimage == true)
	 	{
	  		flag_getnewimage = false;

	  		frame = img_rgb.clone();

	 		// step1： 读取视频帧，传递给markCapture帧处理接口
	 		//camCapture >> frame;
	 		markCapture.processFrame(frame);

	 		//show threshold 显示阀值
	 		//imshow("thresholdImg", markCapture.m_imgThreshold);

	 		//show contours
	 		//显示轮廓
			vector<Vec4i> hierarchy;
			Mat contourImg = Mat::zeros(frame.size(), CV_8UC3);

	 		// step2：画出检测到二维码的轮廓（drawContours为Opencv的函数）
			for(unsigned int i=0, end = markCapture.m_contours.size(); i<end; i++)
			{
				drawContours(contourImg, markCapture.m_contours, i, Scalar(255,255,255), 2, 8, hierarchy, 0, Point());
			}
	 		//imshow("contours", contourImg);

			// show marker
	 		// 显示标记
			for(unsigned int i=0, end = markCapture.m_markers.size(); i<end; i++)
			{
				unsigned int sizeNum = markCapture.m_markers[i].m_points.size();
				for (unsigned int j=0; j<sizeNum; j++)
				{
					line(frame, markCapture.m_markers[i].m_points[j], markCapture.m_markers[i].m_points[(j+1)%sizeNum], Scalar(0,0,255), 2, 8);
				}
				circle(frame, markCapture.m_markers[i].m_points[0], 3, Scalar(0,255,255), 2, 8);
			}

	 		// 显示追踪结果 
	 		imshow("markerDetector", frame);
	 		cvWaitKey(10);
			
	 		// print markers information
	 		// 得到二维码的数量
			int numMarker = markCapture.m_markers.size();
		 	std::cout << "new frame---------------" << std::endl;
			if (numMarker > 0 )
			{
				std::cout << "detect marker number: " << numMarker << std::endl;
				for (int i=0; i<numMarker; i++)
				{
					std::cout << "marker index: " << i << "   " << "marker ID: " << std::bitset<10>(markCapture.m_markers[i].m_id) << std::endl;
				}
			}
			else
			{
				std::cout << "no marker!" << std::endl;
			}

	 		std::cout << markCapture.getTransformations() << std::endl;

	 		// 欧拉角
	 		msg_marker.x = markCapture.getTransformations()[3];
	 		msg_marker.y = markCapture.getTransformations()[4];
	 		msg_marker.z = markCapture.getTransformations()[5];

	 		msg_marker_angle.z = markCapture.getTransformations()[2];

	 		twist_msg.linear.x = 0.5 * (0 - msg_marker.x);
	 		twist_msg.linear.y = 0.5 * (0 - msg_marker.y);
	 		// twist_msg.linear.z = 0.5 * (1 + msg_marker.z);
			twist_msg.linear.z = 0;

			// twist_msg.angular.z = 0.5 * (0 - msg_marker_angle.z); 

	 		if (msg_marker.z == 0) twist_msg.linear.z = 0;
	 		if (msg_marker.x == 0) twist_msg.linear.x = 0;
	 		if (msg_marker.y == 0) twist_msg.linear.y = 0;
	 		// ROS publish 
	 		chatter_pub.publish(msg_marker);
	 		pub_twist.publish(twist_msg);
	 	}

 		//在循环中，必须要有，通过这个才能到cb函数中
  		ros::spinOnce();
  		loop_rate.sleep();

		cvWaitKey(1);

		// keyboard UI
		// opencv等待按键输入函数，等待退出
		int c = cv::waitKey(10);
		if((char)c == 'q')
		{
			 break;
		}
		if(c >= 0 && c!= 'q')
		{
			cv::waitKey();
		}


	}

		

	 

	return 0;

}
