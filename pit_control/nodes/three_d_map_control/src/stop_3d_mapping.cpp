#include <ros/ros.h>
#include <zed_interfaces/stop_3d_mapping.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stop_3d_mapping");

	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<zed_interfaces::stop_3d_mapping>("/zed/zed_node/stop_3d_mapping");

	zed_interfaces::stop_3d_mapping start;

	if (client.call(start))
	{
		ROS_INFO("The map has stopped");
	}

	else 
	{
		client.waitForExistence();
		client.call(start);	
	}

	return 0;
}
