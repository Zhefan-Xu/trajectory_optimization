#include <ros/ros.h>
#include <trajectory_optimization/staticPlanner.h>
#include <trajectory_optimization/readfile.h>
#include <trajectory_optimization/vis_utils.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "test_staticPlanner_node");
	ros::NodeHandle nh;

	// read data and load data
	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);
	std::vector<pose> path = paths[30];

	// parameters
	double res = 0.1; // map resolution
	double xsize = 0.2; double ysize = 0.2; double zsize = 0.1; // Robot collision box size
	int degree = 7; // polynomial degree
	double velocityd = 2; // desired average velocity
	int diff_degree = 4; // Minimum snap (4), minimum jerk (3)
	double perturb = 1; // Regularization term and also make PSD -> PD
	bool shortcut = true; // shortcut waypoints
	double delT = 0.1; // resolution of the final trajectory
	std::vector<pose> loadPath;

	// solve trajectory:
	std::vector<pose> trajectory = staticPlanner(nh, res, xsize, ysize, zsize, degree, velocityd, diff_degree, perturb, path, shortcut, delT, loadPath);


	// Visualization:
	visualization_msgs::MarkerArray path_msg = wrapVisMsg(loadPath);
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