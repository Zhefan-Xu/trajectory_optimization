#include <ros/ros.h>
#include <trajectory_optimization/fakeDetector.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "test_detector_node");
	ros::NodeHandle nh;


	std::vector<std::string> obstaclesType {"person_walking", "person_walking_0"};
	fakeDetector d (nh);
	d.loadObstacleType(obstaclesType);
	std::vector<obstacle> obstacles;
	d.detect(obstacles);

	return 0;
}