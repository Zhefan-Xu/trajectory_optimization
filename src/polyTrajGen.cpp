#include <trajectory_optimization/polyTrajGen.h>

polyTraj::polyTraj(){
	this->degree = 6;
	this->velocityd = 0.5;
	this->diff_degree = 3;
	this->constructQ();
}

polyTraj::polyTraj(int _degree){
	this->degree = _degree;
	this->velocityd = 0.5;
	this->diff_degree = 3;
	this->constructQ();	

}

polyTraj::polyTraj(int _degree, double _velocityd, double _diff_degree){
	this->degree = _degree;
	this->velocityd = _velocityd;
	this->diff_degree = _diff_degree;
	this->constructQ();
}

void polyTraj::loadWaypointPath(std::vector<pose> _path){
	this->path = _path;
	this->timed.clear();
	double total_time = 0;
	this->timed.push_back(total_time);
	for (int i=0; i<this->path.size(); ++i){ // calculate desired time for each segment
		if (i != 0){
			double distance = getDistance(this->path[i], this->path[i-1]);
			total_time += distance/velocityd;
			this->timed.push_back(total_time);
		}
	}
}



Eigen::MatrixXd polyTraj::constructQ(){
	int num_path_segment = this->path.size() - 1;
	int num_each_coeff = this->degree + 1;
	int num_coeff = (this->degree + 1)*4;
	int dimension = ((this->degree+1) * 4) * num_path_segment;
	double weight_xyz = 0.5; double weight_yaw = 1 - weight_xyz;// optimization weights 
	Eigen::MatrixXd Q (dimension, dimension);
	for (int n=0; n<num_path_segment; ++n){
		double start_t = this->timed[n]; double end_t =  this->timed[n+1];

		int segment_start_index = n * num_coeff;
		// x coeff
		int x_coeff_start_index = segment_start_index;

		// y coeff
		int y_coeff_start_index = segment_start_index + num_each_coeff;
		
		// z coeff
		int z_coeff_start_index = segment_start_index + 2 * num_each_coeff;

		// yaw coeff
		int yaw_coeff_start_index = segment_start_index + 3 * num_each_coeff;
	
		for (int i=0; i<num_each_coeff; ++i){
			if (i < this->diff_degree){
				Q(x_coeff_start_index+i, x_coeff_start_index+i) = 0;
				Q(y_coeff_start_index+i, y_coeff_start_index+i) = 0;
				Q(z_coeff_start_index+i, z_coeff_start_index+i) = 0;
			}
			else{
				double factor = 1;
				for (int j=0; j<this->diff_degree; ++j){
					factor *= (i-j);
				}
				factor *= 1/(i-this->diff_degree+1)* (pow(end_t, i-this->diff_degree+1) - pow(start_t, i-this->diff_degree+1));
				factor *= weight_xyz;
				Q(x_coeff_start_index+i, x_coeff_start_index+i) = factor;
				Q(y_coeff_start_index+i, y_coeff_start_index+i) = factor;
				Q(z_coeff_start_index+i, z_coeff_start_index+i) = factor;
			}

			if (i < 2){
				Q(yaw_coeff_start_index+i, yaw_coeff_start_index+i) = 0; 
			}
			else{
				double factor_yaw = i * (i-1) * 1/(i-1) * (pow(end_t, i-1) - pow(start_t, i-1));
				factor_yaw *= weight_yaw;
			}
		}
	}
	this->Q = Q;
}

double polyTraj::constraint(const std::vector<double> &x, std::vector<double> &grad, void* c_data){
    constraint_data *cd = reinterpret_cast<constraint_data*>(c_data);
    int waypoint_index = cd->waypoint_index;
    int segment = cd->segment;
    int variable_index = cd->variable_index;
    int diff = cd->diff;
    double t = this->timed[waypoint_index];

    int num_each_coeff = this->degree + 1;
	int num_coeff = (this->degree + 1)*4;

    if (diff == 0){//fixed by waypoints
    	int start_index;
    	double target_value;
    	double eqn = 0;
    	start_index = (waypoint_index+segment-1) * num_coeff;

    	if (variable_index == 0){ // x
    		target_value = this->path[waypoint_index].x;
    		start_index = start_index;
    	}
    	else if (variable_index == 1){ // y
    		target_value = this->path[waypoint_index].y;
    		start_index = start_index + num_each_coeff;
    	}
    	else if (variable_index == 2){ // z
    		target_value = this->path[waypoint_index].z;
    		start_index = start_index + 2*num_each_coeff;
    	}
    	else if (variable_index == 3){ // yaw
    		target_value = this->path[waypoint_index].yaw;
    		start_index = start_index + 3*num_each_coeff;
    	}
    	else{
    		cout << "invalid variable index" << endl;
    	}


    	for (int i=0; i<num_each_coeff; ++i){
 			eqn += x[start_index+i]*pow(t, i);   		
    	}
    	eqn -= target_value;
    	return eqn;
    }
    else{
    	// continuity
    	int start_index1; int start_index2;
    	double eqn1 = 0;
    	double eqn2 = 0;
    	start_index1 = (waypoint_index-1) * num_coeff;
    	start_index2 = waypoint_index * num_coeff;

    	for (int i=0; i<num_each_coeff; ++i){
    		if (i < diff){
    			eqn1 += 0;
    			eqn2 += 0;
    		}
    		else{
    			double factor = 1;
    			for (int j=0; j<diff; ++j){
    				factor *= (i-j);
    			}
    			factor *= 1/(i-diff+1) * pow(t, i-diff+1);
    			eqn1 += factor * x[start_index1+i];
    			eqn2 += factor * x[start_index2+i];
    		}
    	}
    	return eqn1 - eqn2;
    }
}

// vairable order:[C_seg1, C_seg2, C_seg3 ...],  in each segment: [cx0, cx1, cx2,... cyawn]
double polyTraj::objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data){
	std::vector<double> x_copy (x);
	Eigen::Map<Eigen::VectorXd> x_vec (x_copy.data(), x_copy.size());
	double result = x_vec.transpose() * this->Q * x_vec;
	if (!grad.empty()){
		Eigen::VectorXd grad_vec = this->Q * x_vec;
		grad = std::vector<double>(grad_vec.data(), grad_vec.data() + grad_vec.size());
	}
	return result ;
}



// vairable order:[C_seg1, C_seg2, C_seg3 ...],  in each segment: [c0, c1, c2,... cn], c0 = [cx, cy, cz, cyaw]
// Min (Jerk and 2nd derivative for yaw)
void polyTraj::optimize(){
	int poly_degree = 6;
	int num_path_segment = this->path.size() - 1;
	int num_variables = ((poly_degree+1) * 4) * num_path_segment;

}

void polyTraj::printWaypointPath(){
	cout << fixed << setprecision(2); 
	if (this->path.size() == 0){
		cout << "You have to load path first!" << endl;
	}
	else{
		int count = 0;
		for (pose p: this->path){
			if (count != 0){
				// double distance = sqrt(pow((p.x - this->path[count-1].x),2) + pow((p.y - this->path[count-1].y),2) + pow((p.z - this->path[count-1].z),2));	
				double distance = getDistance(p, path[count-1]);
				cout << ">>[Gap (" << count-1 << "->" << count << "): " << distance << "m.]<<"<< endl;			
			}
			cout << "Waypoint " << count << ": (" << p.x << ", " << p.y << ", " << p.z << "), yaw: " << p.yaw << ". |TIME: " << this->timed[count] << " s.|" <<endl;
			++count; 
		}
	}
}