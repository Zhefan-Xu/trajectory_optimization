#include <ros/ros.h>
#include <trajectory_optimization/readfile.h>
#include <trajectory_optimization/polyTrajGen.h>


int main(int argc, char **argv){
	// Read waypoint path

	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);

	int test_path_index = 30;	// use index 30 for test
	std::vector<pose> path = paths[test_path_index];

	// Conduct optimization
	polyTraj polytraj_optimizer;
	polytraj_optimizer.loadWaypointPath(path);
	polytraj_optimizer.printWaypointPath();
}
