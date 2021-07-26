#include <ros/ros.h>
#include <trajectory_optimization/mavrosTest.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "test_mavros_node");
	ros::NodeHandle nh;
	mavrosTest mtest (nh);
}
