#ifndef MAVROSTEST_H
#define MAVROSTEST_H
#include <ros/ros.h>

class mavrosTest{
private:
	ros::NodeHandle nh;
public:
	mavrosTest(const ros::NodeHandle &_nh);
};



#endif