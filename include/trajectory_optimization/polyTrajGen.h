#ifndef POLYTRAJGEN_H
#define POLYTRAJGEN_H
#include <trajectory_optimization/utils.h>
#include <nlopt.hpp>
#include <Eigen/Eigen>

typedef struct {
    int waypoint_index; // which waypoint is current waypoint
    int segment; // for position 0 is previous, 1 is after
    int variable_index; // x: 0, y:1, z:2, yaw: 3
    int diff; // derivative No.
} constraint_data;

class polyTraj{
private:
	int degree;
	int diff_degree;
	double velocityd;
	std::vector<pose> path;
	std::vector<double> timed; // desired time for path
	Eigen::MatrixXd Q;
public:
	polyTraj();
	polyTraj(int _degree);
	polyTraj(int _degree, double _velocityd, double _diff_degree);
	void loadWaypointPath(std::vector<pose> _path);
	Eigen::MatrixXd constructQ();
	double constraint(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
	double objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
	void optimize();
	void printWaypointPath();

};



#endif