/*
 *  Node for increasing the contrast of an grayscale image
 *  Wim Lemkens <wim.lemkens@gmail.com>
 *  http://rosmultirgbd.wordpress.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

/* Simple contrast control */
string strContrastValue = "contrast_value";
/* Simple brightness control */
string strBrightnessValue = "brightness_value";


class ConstrastAugmenter{

private:
	ros::NodeHandle	nh;
	ros::Publisher	image_pub;
	ros::Publisher	rect_pub;
	ros::Publisher	info_pub;
	ros::Subscriber image_sub;
	ros::Subscriber rect_sub;
	ros::Subscriber info_sub;

public:
    ConstrastAugmenter(int argc, char **argv);
    ~ConstrastAugmenter();
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void rectCallback(const sensor_msgs::Image::ConstPtr& msg);
	void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
	bool isNumeric( const char* pszInput, int nNumberBase );
};

/**
 * Constructor
 */
ConstrastAugmenter::ConstrastAugmenter(int argc, char **argv)
{
	// set initialaize param
	nh.setParam(strContrastValue, 256.0 );
	nh.setParam(strBrightnessValue, 10000 );

	if (argc > 1){
		if (isNumeric(argv[argc-1],10)){
		  	nh.setParam(strContrastValue, atoi(argv[argc-1]) );
		} else {
	  		ROS_ERROR("Expected last parameter to be an integer (alpha). It was '%s'",argv[argc-1]);
	  		return;
		}
	}

	// The default topic (preferbly raw)
	string image = "/camera/ir/image";
	if (ros::names::remap("image") != "image") {
		image = ros::names::remap("image");
	}
	// The rectified topic
	string rect = ros::names::parentNamespace(image)+"/image_rect";
	// The camera info
	string info = ros::names::parentNamespace(image)+"/camera_info";

	string parentns = ros::names::parentNamespace(ros::names::parentNamespace(image));

	string new_image = ros::names::parentNamespace(image)+"_augmented/image";
	string new_rect  = ros::names::parentNamespace(image)+"_augmented/image_rect";
	string new_info  = ros::names::parentNamespace(image)+"_augmented/camera_info";

	ROS_INFO("listening on topic '%s', publishing on '%s'",image.c_str(),new_image.c_str() );
	ROS_INFO("listening on topic '%s', publishing on '%s'",rect.c_str(),new_rect.c_str() );
	ROS_INFO("listening on topic '%s', publishing on '%s'",info.c_str(),new_info.c_str() );
	image_sub = nh.subscribe(image, 3, &ConstrastAugmenter::imageCallback, this);
	rect_sub  = nh.subscribe(rect, 3, &ConstrastAugmenter::rectCallback, this);
	info_sub  = nh.subscribe(info, 3, &ConstrastAugmenter::infoCallback, this);
	image_pub = nh.advertise<sensor_msgs::Image>(new_image, 3);
	rect_pub  = nh.advertise<sensor_msgs::Image>(new_rect, 3);
	info_pub  = nh.advertise<sensor_msgs::CameraInfo>(new_info, 3);
}

/**
 * Destructor
 */
ConstrastAugmenter::~ConstrastAugmenter()
{
}

/**
 * The callback for the image
 */
void ConstrastAugmenter::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	double alpha;
	int beta;
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		nh.getParam(strContrastValue, alpha);
		nh.getParam(strBrightnessValue, beta);
		cv_ptr = cv_bridge::toCvCopy(msg);
		cv_ptr->image.convertTo(cv_ptr->image, -1, alpha, beta);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	image_pub.publish(cv_ptr->toImageMsg());  
}

/**
 * The callback for the rectified image
 */
void ConstrastAugmenter::rectCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	double alpha;
	int beta;
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		nh.getParam(strContrastValue, alpha);
		nh.getParam(strBrightnessValue, beta);
		cv_ptr = cv_bridge::toCvCopy(msg);
		cv_ptr->image.convertTo(cv_ptr->image, -1, alpha, beta);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	rect_pub.publish(cv_ptr->toImageMsg());  
}

/**
 * The callback for the info topic
 */
void ConstrastAugmenter::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	// Just republish it for the new images
	info_pub.publish(msg);  
}

bool ConstrastAugmenter::isNumeric( const char* pszInput, int nNumberBase )
{
	istringstream iss( pszInput );
 
	if ( nNumberBase == 10 )
	{
		double dTestSink;
		iss >> dTestSink;
	}
	else if ( nNumberBase == 8 || nNumberBase == 16 )
	{
		int nTestSink;
		iss >> ( ( nNumberBase == 8 ) ? oct : hex ) >> nTestSink;
	}
	else
		return false;
 
	// was any input successfully consumed/converted?
	if ( ! iss )
		return false;
 
	// was all the input successfully consumed/converted?
	return ( iss.rdbuf()->in_avail() == 0 );
}

int main(int argc, char **argv)
{
	ROS_INFO("contrast_augmenter image:=<grayscale image topic> [alpha]");

	// Initialisation
	ros::init(argc, argv, "contrast_augmenter");

	ConstrastAugmenter aConstrastAugmenter(argc, argv);

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
	ros::spin();

	return 0;
}
