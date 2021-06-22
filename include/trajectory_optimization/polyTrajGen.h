#ifndef POLYTRAJGEN_H
#define POLYTRAJGEN_H
#include <trajectory_optimization/utils.h>
#include <nlopt.hpp>





class polyTraj{
private:
	int degree;
	double velocityd;
	std::vector<pose> path;
	std::vector<double> timed; // desired time for path
public:
	polyTraj();
	polyTraj(int _degree);
	polyTraj(int _degree, double _velocityd);
	void loadWaypointPath(std::vector<pose> _path);
	std::vector<double> minJerkFuncCoeff(double start_t, double end_t); // for x, y, z
	std::vector<double> minSecDiffCoeff(double start_t, double end_t); // for yaw
	double objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
	void optimize();
	void printWaypointPath();

};



#endif