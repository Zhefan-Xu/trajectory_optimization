#include<ros/ros.h>
#include<trajectory_optimization/mapModuleOctomap.h>
#include<trajectory_optimization/polyTrajGen.h>
#include<trajectory_optimization/readfile.h>
#include<trajectory_optimization/vis_utils.h>
#include<chrono> 
using namespace std::chrono;


int main(int argc, char** argv){
	ros::init(argc, argv, "test_mapModuleOctomap_node");
	ros::NodeHandle nh;
	mapModule m (nh, 0.1, 0.3, 0.3, 0.1);

	auto start_time = high_resolution_clock::now();
	m.updateMap();
	auto end_time = high_resolution_clock::now();
	auto duration_total = duration_cast<microseconds>(end_time - start_time);
	cout << "Total map update: "<< duration_total.count()/1e6 << " seconds. " << endl;

	cout << "Map Resolution: " <<  m.getMapResolution() << endl;

	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);
	std::vector<pose> path = paths[37];

	std::vector<pose> path_sc = m.shortcutWaypointPath(path);
	polyTraj polytraj_optimizer (6, 1, 3);
	polytraj_optimizer.loadWaypointPath(path_sc);
	polytraj_optimizer.printWaypointPath();
	auto start_time1 = high_resolution_clock::now();
	polytraj_optimizer.optimize();
	auto end_time1 = high_resolution_clock::now();
	auto duration_total1 = duration_cast<microseconds>(end_time1 - start_time1);
	cout << "Total Trajectory Opimtization: "<< duration_total1.count()/1e6 << " seconds. " << endl;

	std::vector<pose> trajectory = polytraj_optimizer.getTrajectory(0.1);
	// polytraj_optimizer.printTrajectory();
	visualization_msgs::MarkerArray path_msg = wrapVisMsg(path_sc);
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