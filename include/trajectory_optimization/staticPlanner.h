#ifndef STATICPLANNER_H
#define STATICPLANNER_H
#include <ros/ros.h>
#include <trajectory_optimization/mapModuleOctomap.h>
#include <trajectory_optimization/polyTrajGen.h>


std::vector<pose> staticPlanner(const ros::NodeHandle& nh, double res, double xsize, double ysize, double zsize,
								int degree, double velocityd, int diff_degree, double perturb,
								const std::vector<pose>& path, bool shortcut, double delT, std::vector<pose>& loadPath){
	// TODO: implement the static planner

	// Initialization:
	polyTraj polytrajOptimizer (degree, velocityd, diff_degree, perturb);
	mapModule mapModuleOctomap (nh, res, xsize, ysize, zsize);

	mapModuleOctomap.updateMap();

	if (shortcut){ // if we need to short cut path
		loadPath = mapModuleOctomap.shortcutWaypointPath(path);
	}
	else{
		loadPath = path;
	}

	// Step 1: Generate trajectory without collision consideration:
	polytrajOptimizer.loadWaypointPath(loadPath);
	polytrajOptimizer.optimize();
	std::vector<pose> trajectory = polytrajOptimizer.getTrajectory(delT);

	// Step 2: Collision checking (map module):
	std::vector<int> collision_idx;
	bool valid = mapModuleOctomap.checkCollisionTrajectory(trajectory, collision_idx);


	// step 3: If collision: repeat solving corridor constraint until no collision:
	int max_iter = 1; double radius = 1.0;
	if (not valid){
		cout << "[Planner INFO]: " << "adding corridor constraint..." << endl;
		for (int i=0; i<max_iter; ++i){
			polytrajOptimizer.adjustCorridorConstraint(collision_idx, radius, delT);
			polytrajOptimizer.optimize();
			std::vector<pose> trajectory = polytrajOptimizer.getTrajectory(delT);
			collision_idx.clear();
			valid = mapModuleOctomap.checkCollisionTrajectory(trajectory, collision_idx);
			if (valid){
				break;
			}
			radius /= 2;
		}
	}

	return trajectory;
}


#endif