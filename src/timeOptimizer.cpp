#include<trajectory_optimization/timeOptimizer.h>

timeOptimizer::timeOptimizer(const quadprogpp::Vector<double>& _t0, 
							 const quadprogpp::Vector<double>& _x_param_sol, 
				  			 const quadprogpp::Vector<double>& _y_param_sol,
				  			 const quadprogpp::Vector<double>& _z_param_sol){
	this->t0 = _t0;
	this->x_param_sol = _x_param_sol;
	this->y_param_sol = _y_param_sol;
	this->z_param_sol = _z_param_sol;
	this->dt = 0.1;
}

timeOptimizer::timeOptimizer(const quadprogpp::Vector<double>& _t0, 
							 const quadprogpp::Vector<double>& _x_param_sol, 
				  			 const quadprogpp::Vector<double>& _y_param_sol,
				  			 const quadprogpp::Vector<double>& _z_param_sol,
				  			 double _dt){
	this->t0 = _t0;
	this->x_param_sol = _x_param_sol;
	this->y_param_sol = _y_param_sol;
	this->z_param_sol = _z_param_sol;
	this->dt = _dt;
}

timeOptimizer::timeOptimizer(const std::vector<double>& timed, 
				             const quadprogpp::Vector<double>& _x_param_sol, 
				             const quadprogpp::Vector<double>& _y_param_sol,
				             const quadprogpp::Vector<double>& _z_param_sol){
	quadprogpp::Vector<double> _t0;
	this->convertTimed(timed, _t0);
	this->t0 = _t0;
	this->x_param_sol = _x_param_sol;
	this->y_param_sol = _y_param_sol;
	this->z_param_sol = _z_param_sol;
	this->dt = 0.1;
}

timeOptimizer::timeOptimizer(const std::vector<double>& timed, 
				             const quadprogpp::Vector<double>& _x_param_sol, 
				             const quadprogpp::Vector<double>& _y_param_sol,
				             const quadprogpp::Vector<double>& _z_param_sol,
				             double _dt){
	quadprogpp::Vector<double> _t0;
	this->convertTimed(timed, _t0);
	this->t0 = _t0;
	this->x_param_sol = _x_param_sol;
	this->y_param_sol = _y_param_sol;
	this->z_param_sol = _z_param_sol;
	this->dt = _dt;
}

void timeOptimizer::convertTimed(const std::vector<double> timed, quadprogpp::Vector<double> _t0){
	int size = timed.size() - 1;
	_t0.resize(size);
	for (int i=0; i<size; ++i){
		_t0[i] = timed[i+1] - timed[i];
	}
}


void timeOptimizer::computeGradient(const quadprogpp::Vector<double>& t, const objectiveFunc_data& data, quadprogpp::Vector<double>& grad){
	// TODO: implement computing gradient
	int num_variable = t.size();
	grad.resize(num_variable);
	double f0 = this->objectiveFunc(t, data);
	for (int i=0; i<num_variable; ++i){ // each direction of gradient
		quadprogpp::Vector<double> t_plus = t; t_plus[i] += this->dt;
		double f_plus = this->objectiveFunc(t_plus, data);
		grad[i] = (f_plus - f0)/this->dt;
	}

}

double timeOptimizer::objectiveFunc(const quadprogpp::Vector<double>& t, const objectiveFunc_data& data){
	// TODO: evaluate each t
	int num_variable = t.size(); int num_path_segment = num_variable;
	quadprogpp::Matrix<double> Qx, Qy, Qz;
	Qx.resize(num_variable, num_variable); Qy.resize(num_variable, num_variable); Qz.resize(num_variable, num_variable);
	int degree = data.degree; int diff_degree = data.diff_degree; double perturb = data.perturb;

	// for (int )

	return 0;
}

