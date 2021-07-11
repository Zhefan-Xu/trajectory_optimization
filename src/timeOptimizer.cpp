#include<trajectory_optimization/timeOptimizer.h>

timeOptimizer::timeOptimizer(const quadprogpp::Vector<double>& _t0, 
							 const quadprogpp::Vector<double>& _x_param_sol, 
				  			 const quadprogpp::Vector<double>& _y_param_sol,
				  			 const quadprogpp::Vector<double>& _z_param_sol,
				  			 double _degree, double _diff_degree, double _perturb){
	this->t0 = _t0;
	this->x_param_sol = _x_param_sol;
	this->y_param_sol = _y_param_sol;
	this->z_param_sol = _z_param_sol;
	this->dh = 0.1;

	this->degree = _degree;
	this->diff_degree = _diff_degree;
	this->perturb = _perturb;
}

timeOptimizer::timeOptimizer(const quadprogpp::Vector<double>& _t0, 
							 const quadprogpp::Vector<double>& _x_param_sol, 
				  			 const quadprogpp::Vector<double>& _y_param_sol,
				  			 const quadprogpp::Vector<double>& _z_param_sol,
				  			 double _dh, double _degree, double _diff_degree, double _perturb){
	this->t0 = _t0;
	this->x_param_sol = _x_param_sol;
	this->y_param_sol = _y_param_sol;
	this->z_param_sol = _z_param_sol;
	this->dh = _dh;

	this->degree = _degree;
	this->diff_degree = _diff_degree;
	this->perturb = _perturb;
}

timeOptimizer::timeOptimizer(const std::vector<double>& timed, 
				             const quadprogpp::Vector<double>& _x_param_sol, 
				             const quadprogpp::Vector<double>& _y_param_sol,
				             const quadprogpp::Vector<double>& _z_param_sol,
				             double _degree, double _diff_degree, double _perturb){
	quadprogpp::Vector<double> _t0;
	this->timed2Duration(timed, _t0);
	this->t0 = _t0;
	this->x_param_sol = _x_param_sol;
	this->y_param_sol = _y_param_sol;
	this->z_param_sol = _z_param_sol;
	this->dh = 0.1;

	this->degree = _degree;
	this->diff_degree = _diff_degree;
	this->perturb = _perturb;
}

timeOptimizer::timeOptimizer(const std::vector<double>& timed, 
				             const quadprogpp::Vector<double>& _x_param_sol, 
				             const quadprogpp::Vector<double>& _y_param_sol,
				             const quadprogpp::Vector<double>& _z_param_sol,
				             double _dh, double _degree, double _diff_degree, double _perturb){
	quadprogpp::Vector<double> _t0;
	this->timed2Duration(timed, _t0);
	this->t0 = _t0;
	this->x_param_sol = _x_param_sol;
	this->y_param_sol = _y_param_sol;
	this->z_param_sol = _z_param_sol;
	this->dh = _dh;

	this->degree = _degree;
	this->diff_degree = _diff_degree;
	this->perturb = _perturb;
}

void timeOptimizer::timed2Duration(const std::vector<double> timed, quadprogpp::Vector<double>& t){
	int size = timed.size() - 1;
	t.resize(size);
	for (int i=0; i<size; ++i){
		t[i] = timed[i+1] - timed[i];
	}
}

void timeOptimizer::duration2Timed(const quadprogpp::Vector<double>& t, std::vector<double>& timed){
	timed.push_back(0);
	int size = t.size();
	double time_acc = 0;
	for (int i=0; i<size; ++i){
		time_acc += t[i];
		timed.push_back(time_acc);
	}
}


void timeOptimizer::computeGradient(const quadprogpp::Vector<double>& t, quadprogpp::Vector<double>& grad){
	int num_variable = t.size();
	grad.resize(num_variable);
	double f0 = this->objectiveFunc(t);
	for (int i=0; i<num_variable; ++i){ // each direction of gradient
		quadprogpp::Vector<double> t_plus = t; t_plus[i] += this->dh;
		double f_plus = this->objectiveFunc(t_plus);
		grad[i] = (f_plus - f0)/this->dh;
		cout << "i: " << i << ", f0: " << f0 << ", fplus: " << f_plus << endl;
	}

}

double timeOptimizer::objectiveFunc(const quadprogpp::Vector<double>& t){
	// TODO: evaluate each t
	int num_path_segment = t.size();
	int degree = this->degree; int diff_degree = this->diff_degree; double perturb = this->perturb;
	int num_each_coeff = degree + 1; 
	int dimension = num_each_coeff * num_path_segment;

	quadprogpp::Matrix<double> Qx, Qy, Qz;
	Qx.resize(dimension, dimension); Qy.resize(dimension, dimension); Qz.resize(dimension, dimension);
	Qx = 0; Qy = 0; Qz = 0;

	std::vector<double> timed; duration2Timed(t, timed);
	for (int n=0; n<num_path_segment; ++n){
		double start_t = timed[n]; double end_t = timed[n+1];
		int segment_start_index = n * num_each_coeff;

		// Calculate Hessian Matrix:
		quadprogpp::Matrix<double> f; // create factor matrix for x, y and z seperately
		f.resize(num_each_coeff, 1);
		for (int i=0; i<num_each_coeff; ++i){
			if (i < diff_degree){
				f[i][0] = 0;
			}
			else{
				double factor = 1.0;
				for (int j=0; j<diff_degree; ++j){
					factor *= (double) (i-j);
				}
				factor *= (double) (1/(i-diff_degree+1.0)) * (pow(end_t, i-diff_degree+1.0) - pow(start_t, i-diff_degree+1.0));
				f[i][0] = factor;
			}
		}

		quadprogpp::Matrix<double> H = dot_prod(f, quadprogpp::t(f)); // Hessian for x, y, z this segment

		// Assign value to Q:
		for (int row=0; row<num_each_coeff; ++row){
			for (int col=0; col<num_each_coeff; ++col){
				Qx[segment_start_index+row][segment_start_index+col] = H[row][col];
				Qy[segment_start_index+row][segment_start_index+col] = H[row][col];
				Qz[segment_start_index+row][segment_start_index+col] = H[row][col];
			}
		}		
	}

	// purturb to make them PSD -> PD
	for (int i=0; i<dimension; ++i){
		Qx[i][i] += perturb;
		Qy[i][i] += perturb;
		Qz[i][i] += perturb;
	
	}

	quadprogpp::Matrix<double> x_sol, y_sol, z_sol;
	x_sol.resize(dimension, 1); y_sol.resize(dimension, 1); z_sol.resize(dimension, 1);
	x_sol.setColumn(0, this->x_param_sol); y_sol.setColumn(0, this->y_param_sol); z_sol.setColumn(0, this->z_param_sol);
	quadprogpp::Matrix<double> result_xm = quadprogpp::dot_prod(quadprogpp::dot_prod(quadprogpp::t(x_sol), Qx) ,x_sol);
	quadprogpp::Matrix<double> result_ym = quadprogpp::dot_prod(quadprogpp::dot_prod(quadprogpp::t(y_sol), Qy) ,y_sol);
	quadprogpp::Matrix<double> result_zm = quadprogpp::dot_prod(quadprogpp::dot_prod(quadprogpp::t(z_sol), Qz), z_sol);
	double result_x = result_xm[0][0]; double result_y = result_ym[0][0]; double result_z = result_zm[0][0];
	double result = result_x + result_y + result_z;
	return result;
}

void timeOptimizer::projection(quadprogpp::Vector<double>& t){
	for (int i=0; i<t.size(); ++i){
		if (t[i] < 0){
			t[i] = 0;
		}
	}
}


std::vector<double> timeOptimizer::optimize(double lr=0.01, int max_iteration=100){
	// TODO: implement gradient descent optimization
	std::vector<double> optimized_t;
	quadprogpp::Vector<double> current_t, prev_t;
	current_t = this->t0;

	int count_iter = 0;
	double delta = 10000; double tol = 1e-2;
	while (delta > tol){ // stop criteria
		if (count_iter > max_iteration){ // maximum iteration
			break;
		}

		// 1. Compute Gradient
		quadprogpp::Vector<double> grad;
		this->computeGradient(current_t, grad);

		// 2. Gradient Descent
		prev_t = current_t; // record previous value
		current_t = current_t - lr * grad;

		// 3. Projection
		this->projection(current_t);

		// 4. Evaluate delta 
		quadprogpp::Vector<double> diff = current_t - prev_t;
		delta = sqrt(dot_prod(diff, diff));

		++count_iter;
		// cout << grad << endl;
		cout << "current: " << current_t << endl;
		cout << "prev: " << prev_t << endl;
		cout << "grad: " << grad << endl;
		cout << "delta: " << delta << ", at: " << count_iter << endl;
	}

	cout << "delta: " << delta << ", at " << count_iter << " iterations." << endl;

	// Extract data from result
	for (int i=0; i<current_t.size(); ++i){
		optimized_t.push_back(current_t[i]);
	}

	return optimized_t;
}

