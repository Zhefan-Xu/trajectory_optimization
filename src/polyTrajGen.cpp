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


// Variable Order: [[Cx0, Cxn, Cy0, Cyn.....], [...]]
void polyTraj::constructQ(){
	int num_path_segment = this->path.size() - 1;
	int num_each_coeff = this->degree + 1;
	int num_coeff = (this->degree + 1)*4;
	int dimension = ((this->degree+1) * 4) * num_path_segment;
	double weight_xyz = 0.5; double weight_yaw = 1 - weight_xyz;// optimization weights 

	std::vector<double> Q;
	for (int n=0; n<dimension; ++n){
		Q.push_back(0);
	}
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
				Q[x_coeff_start_index+i] = 0;
				Q[y_coeff_start_index+i] = 0;
				Q[z_coeff_start_index+i] = 0;
			}
			else{
				double factor = 1;
				for (int j=0; j<this->diff_degree; ++j){
					factor *= (i-j);
				}
				factor *= 1/(i-this->diff_degree+1)* (pow(end_t, i-this->diff_degree+1) - pow(start_t, i-this->diff_degree+1));
				factor *= weight_xyz;
				Q[x_coeff_start_index+i] = factor;
				Q[y_coeff_start_index+i] = factor;
				Q[z_coeff_start_index+i] = factor;
			}

			if (i < 2){
				Q[yaw_coeff_start_index+i] = 0; 
			}
			else{
				double factor_yaw = i * (i-1) * 1/(i-1) * (pow(end_t, i-1) - pow(start_t, i-1));
				factor_yaw *= weight_yaw;
				Q[yaw_coeff_start_index+i] = factor_yaw;
			}
		}
	}
	this->Q_vec = Q;
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

