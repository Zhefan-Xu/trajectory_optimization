#include <ros/ros.h>
#include <trajectory_optimization/readfile.h>
#include <trajectory_optimization/polyTrajGen.h>
#include <trajectory_optimization/vis_utils.h>
#include <chrono> 

using namespace std::chrono;


int main(int argc, char **argv){
	// Read waypoint path

	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);

	int test_path_index = 30;	// use index 30 for test
	std::vector<pose> path = paths[test_path_index];

	// Conduct optimization
	polyTraj polytraj_optimizer (6, 1.5, 4);
	polytraj_optimizer.loadWaypointPath(path);
	polytraj_optimizer.printWaypointPath();
	auto start_time = high_resolution_clock::now();
	polytraj_optimizer.optimize();
	auto end_time = high_resolution_clock::now();
	auto duration_total = duration_cast<microseconds>(end_time - start_time);
	cout << "Total: "<< duration_total.count()/1e6 << " seconds. " << endl;

	std::vector<pose> trajectory = polytraj_optimizer.getTrajectory(0.1);
	// polytraj_optimizer.printTrajectory();
	visualization_msgs::MarkerArray path_msg = wrapVisMsg(path);
	visualization_msgs::MarkerArray trajectory_msg = wrapVisMsg(trajectory);

	ros::init(argc, argv, "test_generate_trajectory");
	ros::NodeHandle nh;
	ros::Publisher path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_path", 0);
	ros::Publisher trajectory_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 0);

	ros::Rate loop_rate(2);
	while (ros::ok()){
		path_vis_pub.publish(path_msg);
		trajectory_vis_pub.publish(trajectory_msg);
		loop_rate.sleep();
	}

}
