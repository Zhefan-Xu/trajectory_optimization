#ifndef POLYTRAJGEN_H
#define POLYTRAJGEN_H
#include <trajectory_optimization/utils.h>



class polyTraj{
private:
	int degree;
	int diff_degree;
	double velocityd;
	std::vector<pose> path;
	std::vector<double> timed; // desired time for path
	std::vector<double> Q_vec; // array to represent diagonal matrix


public:
	polyTraj();
	polyTraj(int _degree);
	polyTraj(int _degree, double _velocityd, double _diff_degree);
	void loadWaypointPath(std::vector<pose> _path);
	void constructQ();
	void optimize();
	void printWaypointPath();


};



#endif