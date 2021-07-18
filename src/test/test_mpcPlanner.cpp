#include <ros/ros.h>
#include <trajectory_optimization/mpcPlanner.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "test_mpcPlanner_node");
	ros::NodeHandle nh;

	mpcPlanner mp ();
	return 0;
}