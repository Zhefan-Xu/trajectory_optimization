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
	bool checkCollision(point3d p); // check point collision
	bool checkCollision(pose p); //check point collision
	bool checkCollisionRobot(point3d p); // collision check for robot
	bool checkCollisionRobot(pose p); // collision check for robot
	bool checkCollisionTrajectory(std::vector<pose> trajectory, std::vector<pose>& collision_poses, int& collision_idx);
	bool checkCollisionLine(point3d p1, point3d p2);
	bool checkCollisionLine(pose p1, pose p2);
	std::vector<pose> shortcutWaypointPath(std::vector<pose> path);
	double getMapResolution();
};


#endif