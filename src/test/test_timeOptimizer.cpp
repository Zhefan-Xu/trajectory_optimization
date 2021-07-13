#include<ros/ros.h>
#include<trajectory_optimization/mapModuleOctomap.h>
#include<trajectory_optimization/polyTrajGen.h>
#include<trajectory_optimization/readfile.h>
#include<trajectory_optimization/vis_utils.h>
#include<trajectory_optimization/timeOptimizer.h>
#include<chrono> 
using namespace std::chrono;

int main(int argc, char** argv){
	ros::init(argc, argv, "test_timeOptimizer_node");
	ros::NodeHandle nh;
	mapModule m (nh, 0.1, 0.2, 0.2, 0.1);

	auto start_time0 = high_resolution_clock::now();
	m.updateMap();
	auto end_time0 = high_resolution_clock::now();
	auto duration_total0 = duration_cast<microseconds>(end_time0 - start_time0);
	cout << "Total map update: "<< duration_total0.count()/1e6 << " seconds. " << endl;

	cout << "Map Resolution: " <<  m.getMapResolution() << endl;



	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);

	int test_path_index = 37;	// use index 30 for test
	std::vector<pose> path = paths[test_path_index];

	std::vector<pose> path_sc = m.shortcutWaypointPath(path);

	int max_waypoint = 5;
	std::vector<pose> path_new;
	if (path_sc.size() > max_waypoint){
		for (int i=0; i<max_waypoint; ++i){
			path_new.push_back(path_sc[i]);
		}
	}
	else{
		path_new = path_sc;		
	}


	// Conduct optimization
	int degree = 7; double velocityd = 1.5; int diff_degree = 4; double perturb = 1;
	polyTraj polytraj_optimizer (degree, velocityd, diff_degree, perturb);
	polytraj_optimizer.loadWaypointPath(path_new);
	polytraj_optimizer.printWaypointPath();
	auto start_time = high_resolution_clock::now();
	polytraj_optimizer.optimize();
	auto end_time = high_resolution_clock::now();
	auto duration_total = duration_cast<microseconds>(end_time - start_time);
	cout << "Total: "<< duration_total.count()/1e6 << " seconds. " << endl;

	// Time optimizer:
	std::vector<double> timed = polytraj_optimizer.getTimed();


	std::vector<pose> trajectory = polytraj_optimizer.getTrajectory(0.05);
	// polytraj_optimizer.printTrajectory();
	visualization_msgs::MarkerArray path_msg = wrapVisMsg(path_new);
	visualization_msgs::MarkerArray trajectory_msg = wrapVisMsg(trajectory);


	ros::Publisher path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_path", 0);
	ros::Publisher trajectory_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 0);

	ros::Rate loop_rate(2);
	while (ros::ok()){
		path_vis_pub.publish(path_msg);
		trajectory_vis_pub.publish(trajectory_msg);
		loop_rate.sleep();
	}

	return 0;
}