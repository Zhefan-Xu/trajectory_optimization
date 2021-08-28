#ifndef MPCPLANNER_H
#define MPCPLANNER_H
#include <trajectory_optimization/utils.h>
#include <trajectory_optimization/fakeDetector.h>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_optimal_control.hpp>


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
	~mpcPlanner();
	void loadControlLimits(double _T_max, double _roll_max, double _pitch_max);
	void loadParameters(double _mass, double _k_roll, double _tau_roll, double _k_pitch, double _tau_pitch);
	void loadRefTrajectory(const std::vector<pose> &_ref_trajectory, double _delT);
	int findNearestPoseIndex(pose p);
	int findNearestPoseIndex(const DVector &states); // states
	VariablesGrid getReference(int start_idx);
	VariablesGrid getReference(const pose &p);
	VariablesGrid getReference(const pose &p, int target_idx, double ratio);
	std::vector<pose> getTrajectory(const VariablesGrid &xd, int start_idx);
	pose getClosestObstaclePose(int start_idx, const obstacle &ob);
	pose getAvoidanceTarget(int start_idx, const obstacle &ob, int &target_idx);
	bool isObstacleFront(const pose &p, const pose &ob_p);
	bool isMeetingObstacle(const DVector &currentStates, double currentYaw, const std::vector<obstacle> &obstacles, int &obstacle_idx);
	RealTimeAlgorithm constructOptimizer(const DVector &currentStates);
	void optimize(const DVector &currentStates, DVector &nextStates, std::vector<pose> &mpc_trajectory, VariablesGrid &xd);
	int optimize(const DVector &currentStates, double currentYaw, const std::vector<obstacle> &obstacles, DVector &nextStates, std::vector<pose> &mpc_trajectory, VariablesGrid &xd);
	obstacle predictObstacleState(const obstacle &ob, int t);
};


#endif