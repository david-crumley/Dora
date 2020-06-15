
/*
Will be used for original depth perception algorithm using the rectified images from the
left and right camera seperately
*/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// creating dummy callbacks for now
void imageRightRectCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	ROS_INFO("Right Rectified image size: %dx%d", msg->width, msg->height);
}

void imageLeftRectCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	ROS_INFO("Left Rectified image size: %dx%d", msg->width, msg->height);
}

// main
int main(int argc, char** argv)
{
	ros::init(argc, argv, "left_right_rect_subscriber");

	ros::NodeHandle n;

	ros::Subscriber subRightRectified = n.subscribe("/zed/zed_node/right/image_rect_color", 10,			
													 imageRightRectCallback);
	ros::Subscriber subLeftRectified  = n.subscribe("/zed/zed_node/left/image_rect_color", 10,
													 imageLeftRectCallback);

	ros::spin();

	return 0;
}
