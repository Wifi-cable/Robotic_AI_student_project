#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
	ROS_INFO("created send_image_node");
  image_transport::Publisher pub = it.advertise("camera/image", 1); //coment on publisher 
  // open the first webcam plugged in the computer
  cv::VideoCapture camera(0);

  if (!camera.isOpened()) {	//end the main if there is no camera
    ROS_DEBUG("ERROR: Could not open camera");
    return 1;
  }

  cv::Mat3b frame0;	//Mat are c++ objects for storing images without memory management
  camera >> frame0;


  //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame0).toImageMsg();
	 //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame0)toCompressedImgMsg(frame0, 'png');
//  const cv_bridge::Format dst_format = cv_bridge::Format PNG;
//	sensor_msgs::ImagePtr makerImgMessage = cv_bridge::CvImage::toCompressedImageMsg(frame0, cv_bridge::CvImage PNG); 

// Works:
sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame0).toImageMsg();

// Don't Work :-((
// cv_bridge::Format dst_format33 = cv_bridge::PNG;
// sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame0).toCompressedImgMsg(dst_format33);

	//toCompressedImageMsg (sensor_msgs::CompressedImage &ros_image, const Format dst_format=JPG) const 
	//sensor_msgs::CompressedImagePtr cv_bridge::CvImage::toCompressedImageMsg 	( 	const Format  	dst_format = JPG	) 	const
  ros::Rate loop_rate(2);	//publish how many times per second? 
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
		ROS_INFO("publised image");
  }
}

