#include <ros/ros.h>
#include <trajectory_optimization/mavrosTest.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "test_mavros_node");
	ros::NodeHandle nh;
	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	int idx = 30;
	mavrosTest mtest (nh);
	mtest.loadPath(filename, idx);
	mtest.setInitialPosition();
}
