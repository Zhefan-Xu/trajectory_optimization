#ifndef POLYTRAJGEN_H
#define POLYTRAJGEN_H
#include <trajectory_optimization/utils.h>
#include <Eigen/Eigen>
#include <nlopt.hpp>


class polyTraj{
private:
	int degree;
	int diff_degree;
	double velocityd;
	std::vector<pose> path;
	std::vector<double> timed; // desired time for path
	Eigen::MatrixXd Q;

	// following attributes are for optimization constraints:
	int waypoint_index;
	int segment;
	int variable_index;
	int diff;

public:
	polyTraj();
	polyTraj(int _degree);
	polyTraj(int _degree, double _velocityd, double _diff_degree);
	void loadWaypointPath(std::vector<pose> _path);
	Eigen::MatrixXd constructQ();
	double constraint(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
	double objective(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
	void optimize();
	void printWaypointPath();
	static double objectiveWrap(const std::vector<double> &x, std::vector<double> &grad, void *data);
	static double constraintWrap(const std::vector<double> &x, std::vector<double> &grad, void *data);

};



#endif