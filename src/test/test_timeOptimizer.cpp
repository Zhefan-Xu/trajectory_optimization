#include<ros/ros.h>
#include<trajectory_optimization/mapModuleOctomap.h>
#include<trajectory_optimization/polyTrajGen.h>
#include<trajectory_optimization/readfile.h>
#include<trajectory_optimization/vis_utils.h>
#include<trajectory_optimization/timeOptimizer.h>
#include<chrono> 
using namespace std::chrono;

int main(int argc, char** argv){


	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);

	int test_path_index = 30;	// use index 30 for test
	std::vector<pose> path = paths[test_path_index];

	// Conduct optimization
	int degree = 7; double velocityd = 1; int diff_degree = 4; double perturb = 1;
	polyTraj polytraj_optimizer (degree, velocityd, diff_degree, perturb);
	polytraj_optimizer.loadWaypointPath(path);
	polytraj_optimizer.printWaypointPath();
	auto start_time = high_resolution_clock::now();
	polytraj_optimizer.optimize();
	auto end_time = high_resolution_clock::now();
	auto duration_total = duration_cast<microseconds>(end_time - start_time);
	cout << "Total: "<< duration_total.count()/1e6 << " seconds. " << endl;

	// Time optimizer:
	quadprogpp::Vector<double> x_param_sol, y_param_sol, z_param_sol;
	polytraj_optimizer.getSol(x_param_sol, y_param_sol, z_param_sol);
	std::vector<double> timed = polytraj_optimizer.getTimed();
	double dh = 0.01;
	timeOptimizer time_optimizer (timed, x_param_sol, y_param_sol, z_param_sol, dh, degree, diff_degree, perturb);
	double lr = 0.00001; int max_iteration = 100;
	std::vector<double> optimized_t = time_optimizer.optimize(lr, max_iteration);

	int count_timed = 0;
	for (double ot: optimized_t){
		cout << ot <<" " << timed[count_timed+1] - timed[count_timed] << endl;
		++count_timed;
	}

	// polytraj_optimizer.adjustTimedSegment(optimized_t);
	// auto start_time1 = high_resolution_clock::now();
	// polytraj_optimizer.optimize();
	// auto end_time1 = high_resolution_clock::now();
	// auto duration_total1 = duration_cast<microseconds>(end_time1 - start_time1);
	// cout << "Total: "<< duration_total1.count()/1e6 << " seconds. " << endl;



	ros::init(argc, argv, "test_timeOptimizer_node");
	ros::NodeHandle nh;

	std::vector<pose> trajectory = polytraj_optimizer.getTrajectory(0.1);
	// polytraj_optimizer.printTrajectory();
	visualization_msgs::MarkerArray path_msg = wrapVisMsg(path);
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