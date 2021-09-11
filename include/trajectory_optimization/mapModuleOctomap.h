#ifndef MAPMODULEOCTOMAP_H
#define MAPMODULEOCTOMAP_H

#include<ros/ros.h>
#include<trajectory_optimization/utils.h>
#include<octomap/OcTree.h>
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

	// Robot paramters
	double xsize, ysize, zsize; // robot size

public:
	mapModule(const ros::NodeHandle& _nh);
	mapModule(const ros::NodeHandle& _nh, double _res);
	mapModule(const ros::NodeHandle& _nh, double _res, double _xsize, double _ysize, double _zsize);
	void updateMap(); // update tree_ptr
	bool checkCollision(point3d p, bool ignore_unknown=false); // check point collision
	bool checkCollision(pose p, bool ignore_unknown=false); //check point collision
	bool checkCollisionRobot(point3d p, bool ignore_unknown=false); // collision check for robot
	bool checkCollisionRobot(pose p, bool ignore_unknown=false); // collision check for robot
	bool checkCollisionTrajectory(const std::vector<pose>& trajectory, std::vector<int>& collision_idx, bool ignore_unknown=false);
	bool checkCollisionLine(point3d p1, point3d p2, bool ignore_unknown=false);
	bool checkCollisionLine(pose p1, pose p2, bool ignore_unknown=false);
	std::vector<pose> shortcutWaypointPath(std::vector<pose> path, bool ignore_unknown=false);
	double getMapResolution();
};


#endif