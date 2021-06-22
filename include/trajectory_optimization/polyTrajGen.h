#ifndef POLYTRAJGEN_H
#define POLYTRAJGEN_H
#include <trajectory_optimization/utils.h>
#include <nlopt.h>



struct pose{
	double x;
	double y;
	double z;
	double yaw;
};

class polyTraj{
private:
	std::vector<pose> path;
public:
	polyTraj();
	void loadWaypointPath(std::vector<pose> _path);
	void optimize();
	void printWaypointPath();
};



#endif