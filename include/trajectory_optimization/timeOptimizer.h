#ifndef TIMEOPTIMIZER_H
#define TIMEOPTIMIZER_H
#include <trajectory_optimization/utils.h>
#include <QuadProg++.hh>

// Implement using gradient descent method


class timeOptimizer{
private:
	double dh; // delta h (incremental) for computing gradient
	quadprogpp::Vector<double> t0; // this is the time for each segment
	quadprogpp::Vector<double> x_param_sol, y_param_sol, z_param_sol;

	// parameters from polynomial trajectory optimizer:
	double degree;
	double diff_degree;
	double perturb;	

public:


	timeOptimizer(const quadprogpp::Vector<double>& _t0, 
				  const quadprogpp::Vector<double>& _x_param_sol, 
				  const quadprogpp::Vector<double>& _y_param_sol,
				  const quadprogpp::Vector<double>& _z_param_sol,
				  double _degree, double _diff_degree, double _perturb);

	timeOptimizer(const quadprogpp::Vector<double>& _t0, 
			  	  const quadprogpp::Vector<double>& _x_param_sol, 
			      const quadprogpp::Vector<double>& _y_param_sol,
			      const quadprogpp::Vector<double>& _z_param_sol,
			      double _dh, double _degree, double _diff_degree, double _perturb);

	timeOptimizer(const std::vector<double>& timed, 
				  const quadprogpp::Vector<double>& _x_param_sol, 
				  const quadprogpp::Vector<double>& _y_param_sol,
				  const quadprogpp::Vector<double>& _z_param_sol,
				  double _degree, double _diff_degree, double _perturb);

	timeOptimizer(const std::vector<double>& timed, 
				  const quadprogpp::Vector<double>& _x_param_sol, 
				  const quadprogpp::Vector<double>& _y_param_sol,
				  const quadprogpp::Vector<double>& _z_param_sol,
				  double _dh, double _degree, double _diff_degree, double _perturb);



	static void timed2Duration(const std::vector<double> timed, quadprogpp::Vector<double>& t);
	static void duration2Timed(const quadprogpp::Vector<double>& t, std::vector<double>& timed);
	void computeGradient(const quadprogpp::Vector<double>& t, quadprogpp::Vector<double>& grad);
	double objectiveFunc(const quadprogpp::Vector<double>& t);
	void projection(quadprogpp::Vector<double>& t);
	std::vector<double> optimize(double lr, int max_iteration);

};

#endif