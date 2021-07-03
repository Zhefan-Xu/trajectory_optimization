#include <ros/ros.h>
#include <trajectory_optimization/readfile.h>
#include <trajectory_optimization/vis_utils.h>



int main(int argc, char **argv){
	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt";
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);
	
	ros::init(argc, argv, "waypoint_path_vis_node");
	ros::NodeHandle nh;
	ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_path", 0);


	int publish_path_index = 30;
	std::vector<pose> path = paths[publish_path_index];

	visualization_msgs::MarkerArray msg = wrapVisMsg(path);

	

	ros::Rate loop_rate(2);
	while (ros::ok()){
		vis_pub.publish(msg);
		loop_rate.sleep();
	}

}