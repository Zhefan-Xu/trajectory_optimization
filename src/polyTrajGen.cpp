#include <trajectory_optimization/polyTrajGen.h>

polyTraj::polyTraj(){
	this->degree = 6;
	this->velocityd = 0.5;
}

polyTraj::polyTraj(int _degree){
	this->degree = _degree;
	this->velocityd = 0.5;
}

polyTraj::polyTraj(int _degree, double _velocityd){
	this->degree = _degree;
	this->velocityd = _velocityd;
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

std::vector<double> polyTraj::minJerkFuncCoeff(double start_t, double end_t){
	std::vector<double> factor_arr;
	for (int i=0; i<this->degree+1; ++i){// Iterate from c0 to cn
		if (i < 3){
			factor_arr.push_back(0);
		}
		else{
			double factor = i * (i-1) * (i-2) * (1/(i-2)) * (pow(end_t, i-2) - pow(start_t, i-2));
			factor_arr.push_back(factor);
		}
	}
	return factor_arr;
}

std::vector<double> polyTraj::minSecDiffCoeff(double start_t, double end_t){
	std::vector<double> factor_arr;
	for (int i=0; i<this->degree+1; ++i){// Iterate from c0 to cn
		if (i < 2){
			factor_arr.push_back(0);
		}
		else{
			double factor = i * (i-1) * (1/(i-1)) * (pow(end_t, i-1) - pow(start_t, i-1));
			factor_arr.push_back(factor);
		}
	}
	return factor_arr;
}


// vairable order:[C_seg1, C_seg2, C_seg3 ...],  in each segment: [cx0, cx1, cx2,... cyawn]
double polyTraj::objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data){
	int num_path_segment = this->path.size() - 1;
	int num_coeff = (this->degree + 1) * 4; // in each segment
	int num_each_coeff = this->degree + 1; // in each polynomial

	double norm_xyz_total = 0;
	double norm_yaw_total = 0;
	for (int n=0; n < num_path_segment; ++n){ // Iterate through each path segment (min jerk)
		// There are num_coeff in each iteration
		int segment_start_index = n * num_coeff;
		double start_t = this->timed[n]; double end_t =  this->timed[n+1];

		// x coeff
		int x_coeff_start_index = segment_start_index;


		// y coeff
		int y_coeff_start_index = segment_start_index + num_each_coeff;
		
		// z coeff
		int z_coeff_start_index = segment_start_index + 2 * num_each_coeff;

		// yaw coeff
		int yaw_coeff_start_index = segment_start_index + 3 * num_each_coeff;


		std::vector<double> factor_arr_xyz = minJerkFuncCoeff(start_t, end_t);
		std::vector<double> factor_arr_yaw = minSecDiffCoeff(start_t, end_t);
		double x_coeff_sum = 0; double y_coeff_sum = 0; double z_coeff_sum = 0; double yaw_coeff_sum = 0;
		for (int count_coeff_i=0; count_coeff_i<num_coeff; ++count_coeff_i){
			x_coeff_sum += factor_arr_xyz[count_coeff_i] * x[x_coeff_start_index + count_coeff_i];
			y_coeff_sum += factor_arr_xyz[count_coeff_i] * x[y_coeff_start_index + count_coeff_i];
			z_coeff_sum += factor_arr_xyz[count_coeff_i] * x[z_coeff_start_index + count_coeff_i];
			yaw_coeff_sum += factor_arr_yaw[count_coeff_i] * x[yaw_coeff_start_index + count_coeff_i];
		}
		double norm_coeff_xyz = pow(x_coeff_sum, 2) + pow(y_coeff_sum, 2) + pow(z_coeff_sum, 2);
		double norm_coeff_yaw =  pow(yaw_coeff_sum, 2);
		norm_xyz_total += norm_coeff_xyz;
		norm_yaw_total += norm_coeff_yaw;
	}

	double norm_total = 0.5 * norm_xyz_total + 0.5 * norm_yaw_total; // weights by default set to 0.5

	return norm_total;
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