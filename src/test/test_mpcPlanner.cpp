#include <ros/ros.h>
#include <trajectory_optimization/readfile.h>
#include <trajectory_optimization/mpcPlanner.h>
#include <trajectory_optimization/staticPlanner.h>
#include <chrono> 

using namespace std::chrono;

int main(int argc, char** argv){
	ros::init(argc, argv, "test_mpcPlanner_node");
	ros::NodeHandle nh;

	// read data and load data
	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);
	std::vector<pose> path = paths[30];

	// parameters
	double res = 0.1; // map resolution
	double xsize = 0.2; double ysize = 0.2; double zsize = 0.1; // Robot collision box size
	int degree = 7; // polynomial degree
	double velocityd = 1; // desired average velocity
	int diff_degree = 4; // Minimum snap (4), minimum jerk (3)
	double perturb = 1; // Regularization term and also make PSD -> PD
	bool shortcut = true; // shortcut waypoints
	double delT = 0.2; // resolution of the final trajectory
	std::vector<pose> loadPath;

	// solve trajectory:
	std::vector<pose> trajectory = staticPlanner(nh, res, xsize, ysize, zsize, degree, velocityd, diff_degree, perturb, path, shortcut, delT, loadPath);


	int horizon = 20; // MPC horizon

	double mass = 1.0; double k_roll = 1.0; double tau_roll = 1.0; double k_pitch = 1.0; double tau_pitch = 1.0; 
	double T_max = 3 * mass * 9.8; double roll_max = PI_const/6; double pitch_max = PI_const/6; double yawdot_max = PI_const/3;
	mpcPlanner mp (horizon);
	mp.loadParameters(mass, k_roll, tau_roll, k_pitch, tau_pitch);
	mp.loadControlLimits(T_max, roll_max, pitch_max, yawdot_max);
	mp.loadRefTrajectory(trajectory, delT);

	int start_index = 0;
	auto start_time = high_resolution_clock::now();
	DVector currentStates(9); currentStates.setAll(0.0);
	currentStates(0) = trajectory[0].x; currentStates(1) = trajectory[0].y; currentStates(2) = trajectory[0].z; currentStates(8) = trajectory[0].yaw;  
	std::vector<pose> mpc_trajectory = mp.optimize(currentStates);
	auto end_time = high_resolution_clock::now();
	auto duration_total = duration_cast<microseconds>(end_time - start_time);
	cout << "Total: "<< duration_total.count()/1e6 << " seconds. " << endl;


	return 0;
}