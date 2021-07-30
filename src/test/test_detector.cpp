#include <ros/ros.h>
#include <trajectory_optimization/fakeDetector.h>
#include <trajectory_optimization/vis_utils.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "test_detector_node");
	ros::NodeHandle nh;
	ros::Publisher obstacle_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/obstacles", 0);


	std::vector<std::string> obstaclesType {"person_walking", "person_walking_0"};
	fakeDetector d (nh);
	d.loadObstacleType(obstaclesType);
	
	ros::Rate rate(2);
	while (ros::ok()){
		std::vector<obstacle> obstacles;
		d.detect(obstacles);
		visualization_msgs::MarkerArray obstacle_msg = wrapObstacleMsg(obstacles);
		obstacle_vis_pub.publish(obstacle_msg);
		rate.sleep();
	}

	return 0;
}