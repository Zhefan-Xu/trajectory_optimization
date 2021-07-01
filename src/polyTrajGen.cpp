#include <trajectory_optimization/polyTrajGen.h>

polyTraj::polyTraj(){
	this->degree = 6;
	this->velocityd = 0.5;
	this->diff_degree = 3;
}

polyTraj::polyTraj(double _degree){
	this->degree = _degree;
	this->velocityd = 0.5;
	this->diff_degree = 3;

}

polyTraj::polyTraj(double _degree, double _velocityd, double _diff_degree){
	this->degree = _degree;
	this->velocityd = _velocityd;
	this->diff_degree = _diff_degree;
}

void polyTraj::loadWaypointPath(const std::vector<pose> &_path){
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

	this->constructQ();
	this->constructA();
}



// Variable Order: [[Cx0, Cxn, Cy0, Cyn.....], [...]]
void polyTraj::constructQ(){
	int num_path_segment = this->path.size() - 1;
	int num_each_coeff = this->degree + 1;
	int num_coeff = (this->degree + 1)*4;
	int dimension = ((this->degree+1) * 4) * num_path_segment;
	double weight_xyz = 0.5; double weight_yaw = 1 - weight_xyz; // optimization weights 

	// Construct Q Matrix
	this->Q.resize(dimension, dimension);
	this->Q = 0;


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
		

		// Calculate Hessian Matrix:
		quadprogpp::Matrix<double> f; // create factor matrix for x, y and z seperately
		f.resize(num_each_coeff, 1);
		for (int i=0; i<num_each_coeff; ++i){
			if (i < this->diff_degree){
				f[i][0] = 0;
			}
			else{
				double factor = 1.0;
				for (int j=0; j<this->diff_degree; ++j){
					factor *= (double) (i-j);
				}
				factor *= (double) (1/(i-this->diff_degree+1.0)) * (pow(end_t, i-this->diff_degree+1.0) - pow(start_t, i-this->diff_degree+1.0));
				f[i][0] = factor;
			}
		}

		quadprogpp::Matrix<double> f_yaw; // yaw vector factor
		f_yaw.resize(num_each_coeff, 1);
		for (int i=0; i<num_each_coeff; ++i){
			if (i < 2){
				f_yaw[i][0] = 0;
			}
			else{
				double factor_yaw = (double) i * (i-1.0) * (1.0/(i-1.0)) * (pow(end_t, i-1.0) - pow(start_t, i-1.0));
				f_yaw[i][0] = factor_yaw;
			}
		}


		quadprogpp::Matrix<double> H = dot_prod(f, quadprogpp::t(f)); // Hessian for x, y, z this segment
		quadprogpp::Matrix<double> H_yaw = dot_prod(f_yaw, quadprogpp::t(f_yaw)); // Hessian for yaw this segment
		H *= weight_xyz;
		H_yaw *= weight_yaw;


		// cout << "f" << f << endl;
		// cout << "f yaw" << f_yaw << endl;
		// cout << "H" << H << endl;
		// cout << "H yaw" << H_yaw << endl;

		// Assign value to Q:
		for (int row=0; row<num_each_coeff; ++row){
			for (int col=0; col<num_each_coeff; ++col){
				this->Q[x_coeff_start_index+row][x_coeff_start_index+col] = H[row][col];
				this->Q[y_coeff_start_index+row][y_coeff_start_index+col] = H[row][col];
				this->Q[z_coeff_start_index+row][z_coeff_start_index+col] = H[row][col];
				this->Q[yaw_coeff_start_index+row][yaw_coeff_start_index+col] = H_yaw[row][col];
			}
		}		
	}
	// quadprogpp::Matrix <double> test;
	// test.resize(num_each_coeff, num_each_coeff);
	// for (int i=0; i<7; ++i){
	// 	for (int j=0; j<7; ++j){
	// 		test[i][j] = this->Q[21+i][21+j];	
	// 	}
	// }
	// cout << test << endl;
}

void polyTraj::constructA(){

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

