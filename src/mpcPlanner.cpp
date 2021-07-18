#include <trajectory_optimization/mpcPlanner.h>

mpcPlanner::mpcPlanner(){
	this->g = 9.8;
	this->mass = 1.0; //kg
	this->k_roll = 1.0; this->tau_roll = 1.0; 
	this->k_pitch = 1.0; this->tau_pitch = 1.0;
	this->T_max = 2.5; //kg
	this->roll_max = PI_const/6; this->pitch_max = PI_const/6; this->yawdot_max = PI_const/6;
	this->horizon = 20;
}

mpcPlanner::mpcPlanner(int _horizon){
	this->g = 9.8;
	this->mass = 1.0; //kg
	this->k_roll = 1.0; this->tau_roll = 1.0; 
	this->k_pitch = 1.0; this->tau_pitch = 1.0;
	this->T_max = 2.5; //kg
	this->roll_max = PI_const/6; this->pitch_max = PI_const/6; this->yawdot_max = PI_const/6;
	this->horizon = _horizon;
}

void mpcPlanner::loadControlLimits(double _T_max, double _roll_max, double _pitch_max, double _yawdot_max){
	this->T_max = _T_max; //kg
	this->roll_max = _roll_max; this->pitch_max = _pitch_max; this->yawdot_max = _yawdot_max;
}

void mpcPlanner::loadParameters(double _mass, double _k_roll, double _tau_roll, double _k_pitch, double _tau_pitch){
	this->mass = _mass;
	this->k_roll = _k_roll; this->tau_roll = _tau_roll; 
	this->k_pitch = _k_pitch; this->tau_pitch = _tau_pitch;
}

void mpcPlanner::loadRefTrajectory(const std::vector<pose> &_ref_trajectory, double _delT){
	this->ref_trajectory = _ref_trajectory;
	this->delT = _delT;
	cout << "[MPC INFO]: " << "size of reference trajectory: " << this->ref_trajectory.size() << ", delT: " << this->delT << endl;
}


VariablesGrid mpcPlanner::getReference(int start_idx, int horizon){ // need to specify the start index of the trajectory
	VariablesGrid r (4, 0);
	double t = 0;
	for (int i=0; i<horizon; ++i){
		if (i > this->ref_trajectory.size()-1){
			break;
		}

		DVector pose_i (4);
		pose_i(0) = this->ref_trajectory[i].x;
		pose_i(1) = this->ref_trajectory[i].y;
		pose_i(2) = this->ref_trajectory[i].z;
		pose_i(3) = this->ref_trajectory[i].yaw;
		r.addVector(pose_i, t);
		t += this->delT;
	}
	cout << "[MPC INFO]: " << "reference data: \n" << r << endl;
	return r;
}

void mpcPlanner::optimize(int start_idx){
	DifferentialState x;
	DifferentialState y;
	DifferentialState z;
	DifferentialState vx;
	DifferentialState vy;
	DifferentialState vz;
	DifferentialState roll;
	DifferentialState pitch;
	DifferentialState yaw;

	Control T;
	Control roll_d;
	Control pitch_d;
	Control yawdot_d;

	// MODEL Definition
	DifferentialEquation f;
	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(z) == vz;
	f << dot(vx) == T*cos(roll)*sin(pitch)*cos(yaw) + T*sin(roll)*sin(yaw); 
	f << dot(vy) == T*cos(roll)*sin(pitch)*sin(yaw) - T*sin(roll)*cos(yaw);
	f << dot(vz) == T*cos(roll)*cos(pitch) - this->g;
	f << dot(roll) == (1/this->tau_roll) * (this->k_roll * roll_d - roll);
	f << dot(pitch) == (1/this->tau_pitch) * (this->k_pitch * pitch_d - pitch);
	f << dot(yaw) == yawdot_d;

	// Least Square Function
	Function h;
	h << x;
	h << y;
	h << z;
	h << yaw;

	DMatrix Q(4,4);
    Q.setIdentity(); Q(0,0) = 10.0; Q(1,1) = 10.0; Q(2,2) = 10.0;
	
	// get tracking trajectory (future N seconds)
	VariablesGrid r = this->getReference(start_idx, this->horizon);


	// setup OCP
	double t_start = r.getFirstTime(); double t_end = r.getLastTime();
	OCP ocp(t_start, t_end, this->horizon);
	ocp.minimizeLSQ(Q, h, r); // Objective

	// Initial Condition Constraint:
	ocp.subjectTo( AT_START, x  == 0.01 );
	ocp.subjectTo( AT_START, y == 0.0 );
	ocp.subjectTo( AT_START, z  == 0.0 );
	ocp.subjectTo( AT_START, roll == 0.0 );
	ocp.subjectTo( AT_START, pitch == 0.0 );
	ocp.subjectTo( AT_START, yaw == 0.0 );


	ocp.subjectTo(f); // Dynamic Constraint

	// Control Constraints
	ocp.subjectTo( 0 <= T <= this->T_max ); 
	ocp.subjectTo( -this->roll_max <= roll_d <= this->roll_max);
	ocp.subjectTo( -this->pitch_max <= pitch_d <= this->pitch_max);
	ocp.subjectTo( -this->yawdot_max <= yawdot_d <= this->yawdot_max);

	cout << "[MPC INFO]: " << "start optimizing..." << endl;

	// Algorithm
	OptimizationAlgorithm alg (ocp);
	alg.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON_WITH_BLOCK_BFGS )	;
	alg.set( KKT_TOLERANCE, 1e-6 );
	alg.solve();

	cout << "[MPC INFO]: " << "Complete!" << endl;


	VariablesGrid xd;
	alg.getDifferentialStates(xd); // get solutions
}
