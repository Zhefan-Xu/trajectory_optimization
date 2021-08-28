#ifndef DYNAMICPLANNER_H
#define DYNAMICPLANNER_H
#include <trajectory_optimization/mpcPlanner.h>

std::vector<pose> dynamicPlanner(const std::vector<pose> &trajectory, const std::vector<obstacle> &obstacles, int horizon, double mass, double k_roll, double tau_roll, 
								 double k_pitch, double tau_pitch, double T_max, double roll_max, double pitch_max, double delT,
								 const DVector &currentStates, DVector &nextStates, VariablesGrid &xd){
	std::vector<pose> mpc_trajectory;
	mpcPlanner mp (horizon);
	mp.loadParameters(mass, k_roll, tau_roll, k_pitch, tau_pitch);
	mp.loadControlLimits(T_max, roll_max, pitch_max);
	mp.loadRefTrajectory(trajectory, delT);
	mp.optimize(currentStates, 0, obstacles, nextStates, mpc_trajectory, xd); 
	return mpc_trajectory;
} 

std::vector<pose> dynamicPlanner(const std::vector<pose> &trajectory, const std::vector<obstacle> &obstacles, int horizon, double mass, double k_roll, double tau_roll, 
								 double k_pitch, double tau_pitch, double T_max, double roll_max, double pitch_max, double delT,
								 const DVector &currentStates, double currentYaw, DVector &nextStates, VariablesGrid &xd){
	std::vector<pose> mpc_trajectory;
	mpcPlanner mp (horizon);
	mp.loadParameters(mass, k_roll, tau_roll, k_pitch, tau_pitch);
	mp.loadControlLimits(T_max, roll_max, pitch_max);
	mp.loadRefTrajectory(trajectory, delT);
	mp.optimize(currentStates, currentYaw, obstacles, nextStates, mpc_trajectory, xd); 
	return mpc_trajectory;
} 

std::vector<pose> dynamicPlanner(const std::vector<pose> &trajectory, const std::vector<obstacle> &obstacles, int horizon, double mass, double k_roll, double tau_roll, 
								 double k_pitch, double tau_pitch, double T_max, double roll_max, double pitch_max, double delT,
								 const DVector &currentStates, double currentYaw, DVector &nextStates, VariablesGrid &xd, int &obstacle_idx){
	std::vector<pose> mpc_trajectory;
	mpcPlanner mp (horizon);
	mp.loadParameters(mass, k_roll, tau_roll, k_pitch, tau_pitch);
	mp.loadControlLimits(T_max, roll_max, pitch_max);
	mp.loadRefTrajectory(trajectory, delT);
	obstacle_idx = mp.optimize(currentStates, currentYaw, obstacles, nextStates, mpc_trajectory, xd); 
	return mpc_trajectory;
} 


#endif