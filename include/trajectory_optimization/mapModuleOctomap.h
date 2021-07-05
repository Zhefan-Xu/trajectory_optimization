#ifndef MAPMODULEOCTOMAP_H
#define MAPMODULEOCTOMAP_H

#include<ros/ros.h>
#include <octomap/OcTree.h>
#include<octomap_msgs/GetOctomap.h>
#include<octomap_msgs/Octomap.h>
#include<octomap_msgs/conversions.h>


using namespace octomap;

class mapModule{
private:
	ros::NodeHandle nh;
	ros::ServiceClient octomap_client;
	OcTree* tree_ptr;

public:
	mapModule(const ros::NodeHandle& _nh);
	void updateMap();

};


#endif