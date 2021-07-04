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
	std::vector<pose> trajectory;
	std::vector<double> timed; // desired time for path


	// the following attributes are based on Quadprog++
	quadprogpp::Matrix<double> Qx, Qy, Qz; // Hessian
	quadprogpp::Vector<double> px, py, pz; // linear term in objective function
	quadprogpp::Matrix<double> Ax, Ay, Az; // Linear Equality Constraint Matrix
	quadprogpp::Vector<double> bx, by, bz; // Linear Equality Constraint constant vector
	quadprogpp::Matrix<double> Cx, Cy, Cz; // Linear Inequality Constraint Matrix
	quadprogpp::Vector<double> dx, dy, dz; // LInear Inequality Constraint Vector
	quadprogpp::Vector<double> x_param_sol, y_param_sol, z_param_sol; // Solution



public:
	polyTraj();
	polyTraj(double _degree);
	polyTraj(double _degree, double _velocityd, double _diff_degree);
	void loadWaypointPath(const std::vector<pose> &_path);
	void constructQp(); // Hessian Matrix and linear vector
	void constructAb(); // Equality constraint
	void constructCd(); // Inequality constraint
	void optimize();
	pose getPose(double t);
	std::vector<pose> getTrajectory(double delT);
	void printWaypointPath();
	void printTrajectory();


};



#endif