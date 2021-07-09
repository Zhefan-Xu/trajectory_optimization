#include<ros/ros.h>
#include<trajectory_optimization/mapModuleOctomap.h>
#include<trajectory_optimization/polyTrajGen.h>
#include<trajectory_optimization/readfile.h>
#include<trajectory_optimization/vis_utils.h>
#include<chrono> 
using namespace std::chrono;

int main(int argc, char** argv){
	ros::init(argc, argv, "test_timeOptimizer_node");
	ros::NodeHandle nh;
	return 0;
}