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
	double k_yaw, tau_yaw; 

	// Control limit
	double T_max;
	double roll_max;
	double pitch_max;
	double yawdot_max;


public:
	mpcPlanner();
	void loadControlLimits(double _T_max, double _roll_max, double _pitch_max, double _yawdot_max);
	void loadParameters(double _mass, double _k_roll, double _tau_roll, double _k_pitch, double _tau_pitch, double _k_yaw, double _tau_yaw);
	void optimize();
};


#endif