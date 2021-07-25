#ifndef MPCPLANNER_H
#define MPCPLANNER_H
#include <trajectory_optimization/utils.h>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

USING_NAMESPACE_ACADO

class mpcPlanner{
private:
	// Internal paramters
	double g; // gravity constant
	double mass;
	double k_roll, tau_roll;
	double k_pitch, tau_pitch;

	// Control limit
	double T_max;
	double roll_max;
	double pitch_max;

	int horizon;

	int current_idx;
	std::vector<pose> ref_trajectory;
	double delT;

	RealTimeAlgorithm algorithm;
	bool first_time;
public:
	mpcPlanner();
	mpcPlanner(int _horizon);
	void loadControlLimits(double _T_max, double _roll_max, double _pitch_max);
	void loadParameters(double _mass, double _k_roll, double _tau_roll, double _k_pitch, double _tau_pitch);
	void loadRefTrajectory(const std::vector<pose> &_ref_trajectory, double _delT);
	int findNearestPoseIndex(pose p);
	int findNearestPoseIndex(const DVector &states); // states
	VariablesGrid getReference(int start_idx);
	std::vector<pose> getTrajectory(const VariablesGrid &xd, int start_idx);
	RealTimeAlgorithm constructOptimizer(const DVector &currentStates);
	void optimize(const DVector &currentStates, DVector &nextStates, std::vector<pose> &mpc_trajectory, VariablesGrid &xd);
};


#endif