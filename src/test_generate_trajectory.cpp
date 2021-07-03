#include <ros/ros.h>
#include <trajectory_optimization/readfile.h>
#include <trajectory_optimization/polyTrajGen.h>
#include <chrono> 

using namespace std::chrono;


int main(int argc, char **argv){
	// Read waypoint path

	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);

	int test_path_index = 32;	// use index 30 for test
	std::vector<pose> path = paths[test_path_index];

	// Conduct optimization
	polyTraj polytraj_optimizer (6);
	polytraj_optimizer.loadWaypointPath(path);
	polytraj_optimizer.printWaypointPath();
	auto start_time = high_resolution_clock::now();
	polytraj_optimizer.optimize();
	auto end_time = high_resolution_clock::now();
	auto duration_total = duration_cast<microseconds>(end_time - start_time);
	cout << "Total: "<< duration_total.count()/1e6 << " seconds. " << endl;

	std::vector<pose> trajectory = polytraj_optimizer.getTrajectory(0.1);
	polytraj_optimizer.printTrajectory();

}
