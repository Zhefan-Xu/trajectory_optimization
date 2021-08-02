#include <trajectory_optimization/mpcPlanner.h>

mpcPlanner::mpcPlanner(){
	this->g = 9.8;
	this->mass = 1.0; //kg
	this->k_roll = 1.0; this->tau_roll = 1.0; 
	this->k_pitch = 1.0; this->tau_pitch = 1.0;
	this->T_max = 2.5; //kg
	this->roll_max = PI_const/6; this->pitch_max = PI_const/6;
	this->horizon = 20;
	this->first_time = true;
}

mpcPlanner::mpcPlanner(int _horizon){
	this->g = 9.8;
	this->mass = 1.0; //kg
	this->k_roll = 1.0; this->tau_roll = 1.0; 
	this->k_pitch = 1.0; this->tau_pitch = 1.0;
	this->T_max = 2.5; //kg
	this->roll_max = PI_const/6; this->pitch_max = PI_const/6;
	this->horizon = _horizon;
	this->first_time = true;
}

mpcPlanner::~mpcPlanner(){

}

void mpcPlanner::loadControlLimits(double _T_max, double _roll_max, double _pitch_max){
	this->T_max = _T_max; //kg
	this->roll_max = _roll_max; this->pitch_max = _pitch_max;
}

void mpcPlanner::loadParameters(double _mass, double _k_roll, double _tau_roll, double _k_pitch, double _tau_pitch){
	this->mass = _mass;
	this->k_roll = _k_roll; this->tau_roll = _tau_roll; 
	this->k_pitch = _k_pitch; this->tau_pitch = _tau_pitch;
}

void mpcPlanner::loadRefTrajectory(const std::vector<pose> &_ref_trajectory, double _delT){
	this->current_idx = 0;
	this->ref_trajectory = _ref_trajectory;
	this->delT = _delT;
	cout << "[MPC INFO]: " << "size of reference trajectory: " << this->ref_trajectory.size() << ", delT: " << this->delT << endl;
}

int mpcPlanner::findNearestPoseIndex(pose p){
	double min_dist = 1000000;
	int count_idx = 0; int min_idx = 0;
	for (pose ref_p: this->ref_trajectory){
		double dist = getDistance(p, ref_p);
		if (dist < min_dist){
			min_dist = dist;
			min_idx = count_idx;
		}
		++count_idx; 
	}
	
	return min_idx;
}

int mpcPlanner::findNearestPoseIndex(const DVector &states){
	pose p;
	p.x = states(0); p.y = states(1); p.z = states(2);
	return this->findNearestPoseIndex(p);
}

VariablesGrid mpcPlanner::getReference(int start_idx){ // need to specify the start index of the trajectory
	VariablesGrid r (3, 0);
	double t = 0; int count_horizon = 0;
	for (int i=start_idx; i<start_idx+this->horizon; ++i){
		if (i > this->ref_trajectory.size()-1){
			break;
		}

		DVector pose_i (3);
		pose_i(0) = this->ref_trajectory[i].x;
		pose_i(1) = this->ref_trajectory[i].y;
		pose_i(2) = this->ref_trajectory[i].z;
		// pose_i(3) = this->ref_trajectory[i].yaw;
		r.addVector(pose_i, t);
		t += this->delT;
		++count_horizon;
	}


	while (count_horizon != this->horizon){
		r.addVector(r.getLastVector(), t);
		t += this->delT;
		++count_horizon;
	}

	// cout << "[MPC INFO]: " << "reference data: \n" << r << endl;
	return r;
}

std::vector<pose> mpcPlanner::getTrajectory(const VariablesGrid &xd, int start_idx){
	std::vector<pose> mpc_trajectory;
	for (int i=0; i<this->horizon; ++i){
		pose p;
		DVector states = xd.getVector(i);
		p.x = states(0); p.y = states(1); p.z = states(2);
		// calcualte yaw:
		if (i < this->horizon-1){
			DVector nextStates = xd.getVector(i+1);
			double dx = nextStates(0) - states(0);
			double dy = nextStates(1) - states(1);
			if (start_idx + i >= this->ref_trajectory.size()-1){
				p.yaw = this->ref_trajectory[ref_trajectory.size()-1].yaw;
			}
			else{
				p.yaw = atan2(dy, dx);	
			}
		}
		else{ // last waypoint
			DVector prevStates = xd.getVector(i-1);
			double dx = states(0) - prevStates(0);
			double dy = states(1) - prevStates(1);
			if (start_idx + i >= this->ref_trajectory.size()-1){
				p.yaw = this->ref_trajectory[ref_trajectory.size()-1].yaw;
			}
			else{
				p.yaw = atan2(dy, dx);	
			}
		}
		mpc_trajectory.push_back(p);
	}
	return mpc_trajectory;
}



RealTimeAlgorithm mpcPlanner::constructOptimizer(const DVector &currentStates){
	DifferentialState x;
	DifferentialState y;
	DifferentialState z;
	DifferentialState vx;
	DifferentialState vy;
	DifferentialState vz;
	DifferentialState roll;
	DifferentialState pitch;
	double yaw = 0;

	Control T;
	Control roll_d;
	Control pitch_d;

	// MODEL Definition
	DifferentialEquation f;
	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(z) == vz;
	f << dot(vx) == T*cos(roll)*sin(pitch)*cos(yaw) + T*sin(roll)*sin(yaw); 
	f << dot(vy) == T*cos(roll)*sin(pitch)*sin(yaw) - T*sin(roll)*cos(yaw);
	f << dot(vz) == T*cos(roll)*cos(pitch) - this->g;
	f << dot(roll) == (1.0/this->tau_roll) * (this->k_roll * roll_d - roll);
	f << dot(pitch) == (1.0/this->tau_pitch) * (this->k_pitch * pitch_d - pitch);

	// Least Square Function
	Function h;
	h << x;
	h << y;
	h << z;

	DMatrix Q(3,3);
    Q.setIdentity(); Q(0,0) = 10.0; Q(1,1) = 10.0; Q(2,2) = 10.0; 
	
	// get tracking trajectory (future N seconds)
	int start_idx = this->findNearestPoseIndex(currentStates);
	// int start_idx = 0;

	cout <<"[MPC INFO]: " << "start_idx: " << start_idx << endl;
	VariablesGrid r = this->getReference(start_idx);

	// setup OCP
	OCP ocp(r);
	ocp.minimizeLSQ(Q, h, r); // Objective

	// Dynamic Constraint
	ocp.subjectTo(f); 
	// Control Constraints
	ocp.subjectTo( 0 <= T <= this->T_max ); 
	ocp.subjectTo( -this->roll_max <= roll_d <= this->roll_max);
	ocp.subjectTo( -this->pitch_max <= pitch_d <= this->pitch_max);
	ocp.subjectTo( -this->roll_max <= roll <= this->roll_max);
	ocp.subjectTo( -this->pitch_max <= pitch <= this->pitch_max);

	// Algorithm
	RealTimeAlgorithm RTalgorithm(ocp, this->delT);
	return RTalgorithm;
}

void mpcPlanner::optimize(const DVector &currentStates, DVector &nextStates, std::vector<pose> &mpc_trajectory, VariablesGrid &xd){
	int start_idx;
	if (this->first_time){
		algorithm = this->constructOptimizer(currentStates);
		this->first_time = false;
	}
	else{
		start_idx = this->findNearestPoseIndex(currentStates);
		VariablesGrid r = this->getReference(start_idx);
		algorithm.setReference(r);
	}

	cout << "[MPC INFO]: " << "Start optimizing..." << endl;
	algorithm.solve(0, currentStates);
	cout << "[MPC INFO]: " << "Complete!" << endl;

	// Get Solutions
	// VariablesGrid xd, cd;
	// VariablesGrid cd;
	algorithm.getDifferentialStates(xd); // get solutions
	// algorithm.getControls(cd);
	// cout << xd << endl;
	// cout << cd << endl;

	mpc_trajectory = this->getTrajectory(xd, start_idx);
	nextStates = xd.getVector(1);
}

void mpcPlanner::optimize(const DVector &currentStates, const std::vector<obstacle> &obstacles, DVector &nextStates, std::vector<pose> &mpc_trajectory, VariablesGrid &xd){ 
	DifferentialState pos ("position", 3, 1);
	DifferentialState vel ("velocity", 3, 1);
	DifferentialState angle ("angle", 2, 1);
	double yaw = 0;

	Control T; 
	Control roll_d; 
	Control pitch_d; 

	// MODEL Definition
	DifferentialEquation f;
	f << dot(pos(0)) == vel(0);
	f << dot(pos(1)) == vel(1);
	f << dot(pos(2)) == vel(2);
	f << dot(vel(0)) == T*cos(angle(0))*sin(angle(1))*cos(yaw) + T*sin(angle(0))*sin(yaw); 
	f << dot(vel(1)) == T*cos(angle(0))*sin(angle(1))*sin(yaw) - T*sin(angle(0))*cos(yaw);
	f << dot(vel(2)) == T*cos(angle(0))*cos(angle(1)) - this->g;
	f << dot(angle(0)) == (1.0/this->tau_roll) * (this->k_roll * roll_d - angle(0));
	f << dot(angle(1)) == (1.0/this->tau_pitch) * (this->k_pitch * pitch_d - angle(1));

	// Least Square Function
	Function h;
	h << pos(0);
	h << pos(1);
	h << pos(2);

	DMatrix Q(3,3);
    Q.setIdentity(); Q(0,0) = 10.0; Q(1,1) = 10.0; Q(2,2) = 10.0; 
	
	// get tracking trajectory (future N seconds)
	int start_idx = this->findNearestPoseIndex(currentStates);
	// int start_idx = 0;

	cout <<"[MPC INFO]: " << "start_idx: " << start_idx << endl;
	VariablesGrid r = this->getReference(start_idx);

	// setup OCP
	OCP ocp(r);
	ocp.minimizeLSQ(Q, h, r); // Objective

	// Dynamic Constraint
	ocp.subjectTo(f); 
	// Control Constraints
	// ConstraintComponent force_limit (0 <= T <= this->T_max);
	// ocp.subjectTo( force_limit );
	ocp.subjectTo( 0 <= T <= this->T_max ); 
	ocp.subjectTo( -this->roll_max <= roll_d <= this->roll_max);
	ocp.subjectTo( -this->pitch_max <= pitch_d <= this->pitch_max);


	// TODO: obstacle constraint:
	double delta = 0.2;
	for (int t=0; t < this->horizon; ++t){
		for (obstacle ob: obstacles){
			obstacle pred_ob = this->predictObstacleState(ob, t);
			ocp.subjectTo(t,   sqrt(pow((pos(0)-pred_ob.x), 2)/pow(pred_ob.xsize, 2) + pow((pos(1)-pred_ob.y), 2)/pow(pred_ob.ysize, 2) + pow((pos(2)-pred_ob.z), 2)/pow(pred_ob.zsize,2)) -1
			               >= 0 ) ; // without probability
			// ocp.subjectTo(t, sqrt(pow((pos(0)-pred_ob.x), 2)/pow(pred_ob.xsize, 2) + pow((pos(1)-pred_ob.y), 2)/pow(pred_ob.ysize, 2) + pow((pos(2)-pred_ob.z), 2)/pow(pred_ob.zsize,2)) -1
			//             - my_erfinvf(1-2*delta) * sqrt(2 * (pred_ob.varX*pow(pos(0)-pred_ob.x, 2)/pow(pred_ob.xsize, 4) + 
			//                								pred_ob.varY*pow(pos(1)-pred_ob.y, 2)/pow(pred_ob.ysize, 4) + // with probability
			// 											pred_ob.varZ*pow(pos(2)-pred_ob.z, 2)/pow(pred_ob.zsize, 4))
			// 											* 1/(pow((pos(0) - pred_ob.x)/pred_ob.xsize, 2) + pow((pos(1) - pred_ob.y)/pred_ob.ysize, 2) + pow((pos(2) - pred_ob.z)/pred_ob.zsize, 2))) >= 0 ) ;						
		}
	}


	// Algorithm
	RealTimeAlgorithm RTalgorithm(ocp, this->delT);
	cout << "[MPC INFO]: " << "Start optimizing..." << endl;
	RTalgorithm.solve(0, currentStates);
	cout << "[MPC INFO]: " << "Complete!" << endl;
	RTalgorithm.getDifferentialStates(xd);
	mpc_trajectory = this->getTrajectory(xd, start_idx);
	nextStates = xd.getVector(1);
	clearAllStaticCounters();
}


obstacle mpcPlanner::predictObstacleState(const obstacle &ob, int t){ // t is the time step
	obstacle pred_ob;
	// position prediction
	pred_ob.x = ob.x + t * this->delT * ob.vx;
	pred_ob.y = ob.y + t * this->delT * ob.vy;
	pred_ob.z = ob.z + t * this->delT * ob.vz;

	// uncertainty propagation
	pred_ob.varX = ob.varX + ob.varVx * t * pow(this->delT, 2);
	pred_ob.varY = ob.varY + ob.varVy * t * pow(this->delT, 2);
	pred_ob.varZ = ob.varZ + ob.varVz * t * pow(this->delT, 2);

	// size does not change:
	pred_ob.xsize = ob.xsize; 
	pred_ob.ysize = ob.ysize;
	pred_ob.zsize = ob.zsize;
	return pred_ob;
}