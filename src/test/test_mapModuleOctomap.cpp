#include<ros/ros.h>
#include<trajectory_optimization/mapModuleOctomap.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "test_mapModuleOctomap_node");
	ros::NodeHandle nh;
	mapModule m (nh);

	return 0;
}