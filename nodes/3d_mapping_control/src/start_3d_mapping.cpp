#include <ros/ros.h>
#include <zed_interfaces/start_3d_mapping.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "start_3d_mapping");

	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<zed_interfaces::start_3d_mapping>("/zed/zed_node/start_3d_mapping");

	zed_interfaces::start_3d_mapping start;

	// parameters for the map
	start.request.resolution = 0.01;
	start.request.max_mapping_range = 4.0;
	start.request.fused_pointcloud_freq = 100.0;

	if (client.call(start))
	{
		ROS_INFO("The map has started");
	}

	else 
	{
		client.waitForExistence();
		client.call(start);	
	}

	return 0;
}
