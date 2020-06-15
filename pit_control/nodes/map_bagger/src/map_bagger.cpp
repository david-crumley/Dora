#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>


void bagIt(const sensor_msgs::PointCloud2::ConstPtr& map)
{	
	rosbag::Bag bag;
	bag.open("3d_map.bag", rosbag::bagmode::Write);
	bag.write("fused_cloud", ros::Time::now(), *map);
	
	ROS_INFO("Bagged the 3d_map in file: 3d_map.bag");

	bag.close();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_bagger");

	ros::NodeHandle n;	

	ros::Subscriber map_bagger = n.subscribe("/zed/zed_node/mapping/fused_cloud", 2, bagIt);

	ros::spin();

	return 0;
}



