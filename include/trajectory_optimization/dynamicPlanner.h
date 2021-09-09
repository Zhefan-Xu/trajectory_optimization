#ifndef DYNAMICPLANNER_H
#define DYNAMICPLANNER_H
#include <trajectory_optimization/mpcPlanner.h>
#include <trajectory_optimization/mapModuleOctomap.h>


std::vector<pose> dynamicPlanner(const std::vector<pose> &trajectory, const std::vector<obstacle> &obstacles, int horizon, double mass, double k_roll, double tau_roll, 
								 double k_pitch, double tau_pitch, double T_max, double roll_max, double pitch_max, double delT,
								 const DVector &currentStates, double currentYaw, DVector &nextStates, VariablesGrid &xd){
	std::vector<pose> mpc_trajectory;
	mpcPlanner mp (horizon);
	mp.loadParameters(mass, k_roll, tau_roll, k_pitch, tau_pitch);
	mp.loadControlLimits(T_max, roll_max, pitch_max);
	mp.loadRefTrajectory(trajectory, delT);
	mp.optimize(currentStates, currentYaw, obstacles, nextStates, mpc_trajectory, xd); 

	// TODO: include collision checking
	
	
	return mpc_trajectory;
} 


#endif