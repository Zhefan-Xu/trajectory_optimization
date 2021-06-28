double polyTraj::objective(const std::vector<double> &x, std::vector<double> &grad, void* f_data){
	double result = 0;
	int dimension = this->Q_vec.size();
	for (int i=0; i<dimension; ++i){
		result += pow(x[i],2) * this->Q_vec[i];
		if (!grad.empty()){
			grad[i] = x[i] * this->Q_vec[i] * 2;
		}
	}
	// return x[0]+x[1] + x[2] + x[3];
	return result;
}