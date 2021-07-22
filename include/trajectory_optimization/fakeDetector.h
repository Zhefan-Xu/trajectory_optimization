#ifndef FAKEDETECTOR_H
#define FAKEDETECTOR_H
#include <trajectory_optimization/fakeDetector.h>

// this class is for simulation 
class fakeDetector{
private:

public:
	fakeDetector(const ros::NodeHandle& _nh, std::string topicName);
};

#endif