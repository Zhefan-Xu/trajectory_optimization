#include <ros/ros.h>
#include <trajectory_optimization/readfile.h>
#include <trajectory_optimization/staticPlanner.h>
#include <trajectory_optimization/dynamicPlanner.h>
#include <trajectory_optimization/vis_utils.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "test_dynamicPlanner_node");
	ros::NodeHandle nh;

	// read data and load data
	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);
	std::vector<pose> path = paths[30];

	double res = 0.1; // map resolution
	double xsize = 0.2; double ysize = 0.2; double zsize = 0.1; // Robot collision box size
	mapModule* mapModuleOctomap = new mapModule (nh, res, xsize, ysize, zsize);
	mapModuleOctomap->updateMap();

	// parameters
	int degree = 7; // polynomial degree
	double velocityd = 1; // desired average velocity
	int diff_degree = 4; // Minimum snap (4), minimum jerk (3)
	double perturb = 1; // Regularization term and also make PSD -> PD
	bool shortcut = true; // shortcut waypoints
	double delT = 0.1; // resolution of the final trajectory
	std::vector<pose> loadPath;

	// solve trajectory:
	std::vector<pose> trajectory = staticPlanner(mapModuleOctomap, degree, velocityd, diff_degree, perturb, path, shortcut, delT, loadPath);
	

	// MPC
	int horizon = 40; // MPC horizon
	double mass = 1.0; double k_roll = 1.0; double tau_roll = 1.0; double k_pitch = 1.0; double tau_pitch = 1.0; 
	double T_max = 3 * 9.8; double roll_max = PI_const/6; double pitch_max = PI_const/6; 



	// Visualization:
	visualization_msgs::MarkerArray path_msg = wrapVisMsg(loadPath, 0, 0, 0);
	visualization_msgs::MarkerArray trajectory_msg = wrapVisMsg(trajectory, 0, 1, 0);

	ros::Publisher path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_path", 0);
	ros::Publisher trajectory_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 0);
	ros::Publisher mpc_trajectory_vis_pub = nh.advertise<nav_msgs::Path>("mpc_trajectory", 0);

	ros::Rate loop_rate(1/delT);
	DVector currentStates(8); currentStates.setAll(0.0);
	currentStates(0) = trajectory[0].x; currentStates(1) = trajectory[0].y; currentStates(2) = trajectory[0].z; 
	DVector nextStates; std::vector<pose> mpc_trajectory;
	double currentYaw = 0;
	while (ros::ok()){
		VariablesGrid xd;
		std::vector<obstacle> obstacles;
		auto start_time = high_resolution_clock::now();
		mpc_trajectory = dynamicPlanner(mapModuleOctomap, trajectory, obstacles, horizon, mass, k_roll, tau_roll, k_pitch, tau_pitch, T_max, roll_max, pitch_max, delT, currentStates, currentYaw, nextStates, xd);
		currentStates = nextStates; 
		auto end_time = high_resolution_clock::now();
		auto duration_total = duration_cast<microseconds>(end_time - start_time);
		cout << "Total: "<< duration_total.count()/1e6 << " seconds. " << endl;		
		nav_msgs::Path mpc_trajectory_msg = wrapPathMsg(mpc_trajectory);
		path_vis_pub.publish(path_msg);
		trajectory_vis_pub.publish(trajectory_msg);
		mpc_trajectory_vis_pub.publish(mpc_trajectory_msg);		
		loop_rate.sleep();
		break;
	}

	return 0;
}