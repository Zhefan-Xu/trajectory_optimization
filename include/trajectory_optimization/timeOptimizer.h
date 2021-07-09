#ifndef TIMEOPTIMIZER_H
#define TIMEOPTIMIZER_H
#include <trajectory_optimization/utils.h>
#include <QuadProg++.hh>

// Implement using gradient descent method
struct objectiveFunc_data{
	int degree;
	int diff_degree;
	double perturb;
	
	objectiveFunc_data(int _degree, int _diff_degree, double _perturb){
		this->degree = _degree; this->diff_degree = _diff_degree; this->perturb = _perturb;
	}
};


class timeOptimizer{
private:
	double dt; // delta t for computing gradient
	quadprogpp::Vector<double> t0; // this is the time for each segment
	quadprogpp::Vector<double> x_param_sol, y_param_sol, z_param_sol;



public:


	timeOptimizer(const quadprogpp::Vector<double>& _t0, 
				  const quadprogpp::Vector<double>& _x_param_sol, 
				  const quadprogpp::Vector<double>& _y_param_sol,
				  const quadprogpp::Vector<double>& _z_param_sol);

	timeOptimizer(const quadprogpp::Vector<double>& _t0, 
			  	  const quadprogpp::Vector<double>& _x_param_sol, 
			      const quadprogpp::Vector<double>& _y_param_sol,
			      const quadprogpp::Vector<double>& _z_param_sol,
			      double _dt);

	timeOptimizer(const std::vector<double>& timed, 
				  const quadprogpp::Vector<double>& _x_param_sol, 
				  const quadprogpp::Vector<double>& _y_param_sol,
				  const quadprogpp::Vector<double>& _z_param_sol);

	timeOptimizer(const std::vector<double>& timed, 
				  const quadprogpp::Vector<double>& _x_param_sol, 
				  const quadprogpp::Vector<double>& _y_param_sol,
				  const quadprogpp::Vector<double>& _z_param_sol,
				  double _dt);



	void timed2Duration(const std::vector<double> timed, quadprogpp::Vector<double>& t);
	void duration2Timed(const quadprogpp::Vector<double>& t, std::vector<double>& timed);
	void computeGradient(const quadprogpp::Vector<double>& t, const objectiveFunc_data& data, quadprogpp::Vector<double>& grad);
	double objectiveFunc(const quadprogpp::Vector<double>& t, const objectiveFunc_data& data);

};

#endif