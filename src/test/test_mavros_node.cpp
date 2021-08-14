#include <ros/ros.h>
#include <trajectory_optimization/mavrosTest.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "test_mavros_node");
	ros::NodeHandle nh;
	// std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt"; int idx = 37; 
	// std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_box.txt"; int idx = 0;
	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze.txt"; int idx = 0;
	double delT = 0.1;
	mavrosTest mtest (nh, delT);
	mtest.loadPath(filename, idx);
	mtest.setInitialPosition();
	mtest.run();
}
