#ifndef FAKEDETECTOR_H
#define FAKEDETECTOR_H
#include <ros/ros.h>
#include <iostream>
#include <gazebo_msgs/GetModelState.h>
#include <trajectory_optimization/utils.h>

using std::cout; using std::endl;




// this class is for simulation 
class fakeDetector{
private:
	ros::NodeHandle nh;
	ros::ServiceClient gazebo_getModel_client;
	std::vector<std::string> obstaclesType;

public:
	fakeDetector(const ros::NodeHandle& _nh);
	void detect(std::vector<obstacle> &obstacles); 
	void loadObstacleType(const std::vector<std::string> &_obstaclesType);
};

#endif