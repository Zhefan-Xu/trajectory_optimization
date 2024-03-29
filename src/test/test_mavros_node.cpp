#include <ros/ros.h>
#include <trajectory_optimization/mavrosTest.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "test_mavros_node");
	ros::NodeHandle nh;
	// std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze_complete.txt"; int idx = 37; std::vector<std::string> obstaclesType {"person_walking", "person_walking_0"};
	std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_box.txt"; int idx = 0; std::vector<std::string> obstaclesType {"person_walking", "person_walking_0"};
	// std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_maze.txt"; 
	// // int idx = 0;  
	// int idx = 1; 
	// std::vector<std::string> obstaclesType {"person_walking", "person_walking_0", "person_walking_1", "person_walking_2", "person_walking_3",
	// 										"person_walking_4", "person_walking_5", "person_walking_6", "warehouse_robot", "warehouse_robot_0",
	// 										"warehouse_robot_1", "warehouse_robot_2", "warehouse_robot_3"};
	

	// std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_tunnel.txt"; 
	// int idx = 0;  
	// std::vector<std::string> obstaclesType {"person_walking", "person_walking_0", "person_walking_1", "person_walking_2", 
	// 										"warehouse_robot", "warehouse_robot_0", "warehouse_robot_1", "warehouse_robot_2","warehouse_robot_3"};


	// std::string filename = "/home/zhefan/catkin_ws/src/trajectory_optimization/path/waypoint_tree.txt"; 
	// // int idx = 0;  
	// // int idx = 1;  
	// int idx = 2;  
	// std::vector<std::string> obstaclesType {"person_walking", "person_walking_0", "person_walking_1", "person_walking_2", "person_walking_3",
	// 										"person_walking_4", "person_walking_5", "person_walking_6", "person_walking_7", "person_walking_8",
	// 									    "warehouse_robot", "warehouse_robot_0"};
	double delT = 0.05;
	mavrosTest mtest (nh, delT);
	mtest.loadPath(filename, idx);
	mtest.setInitialPosition();
	mtest.loadObstacleType(obstaclesType);
	mtest.run();
}
