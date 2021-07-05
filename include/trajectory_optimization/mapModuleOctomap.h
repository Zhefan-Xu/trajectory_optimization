#ifndef MAPMODULEOCTOMAP_H
#define MAPMODULEOCTOMAP_H

#include<ros/ros.h>
#include <octomap/OcTree.h>
#include<octomap_msgs/GetOctomap.h>
#include<octomap_msgs/Octomap.h>
#include<octomap_msgs/conversions.h>


using namespace octomap;
using namespace std;

class mapModule{
private:
	ros::NodeHandle nh;
	ros::ServiceClient octomap_client;
	OcTree* tree_ptr;
	AbstractOcTree* abtree;
	double res;

public:
	mapModule(const ros::NodeHandle& _nh);
	mapModule(const ros::NodeHandle& _nh, double _res);
	void updateMap(); // update tree_ptr
	double getMapResolution();

};


#endif