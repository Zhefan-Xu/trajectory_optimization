#ifndef POLYTRAJGEN_H
#define POLYTRAJGEN_H
#include <trajectory_optimization/utils.h>
#include <QuadProg++.hh>


class polyTraj{
private:
	double degree;
	double diff_degree;
	double velocityd;
	std::vector<pose> path;
	std::vector<double> timed; // desired time for path


	// the following attributes are based on Quadprog++
	quadprogpp::Matrix<double> Q; // Hessian
	quadprogpp::Matrix<double> A; // Linear Equality Constraint Matrix
	quadprogpp::Vector<double> b; // Linear Equality Constraint constant vector


public:
	polyTraj();
	polyTraj(double _degree);
	polyTraj(double _degree, double _velocityd, double _diff_degree);
	void loadWaypointPath(const std::vector<pose> &_path);
	void constructQ(); // Hessian Matrix
	void constructAb(); // Equality constraint
	void optimize();
	void printWaypointPath();


};



#endif