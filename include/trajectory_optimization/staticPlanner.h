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

	// Step 1: Load waypoints
	polytrajOptimizer.loadWaypointPath(loadPath);


	// step 2: corridor constraint for all segments
	std::set<int> collision_seg;
	for (int i=0; i<loadPath.size()-1; ++i){
		collision_seg.insert(i);
	}

	int max_iter = 10; double radius = 0.5;
	std::vector<pose> trajectory;
	std::vector<int> collision_idx;
	bool valid = false;
	if (not valid){
		cout << "[Planner INFO]: " << "adding corridor constraint..." << endl;
		for (int i=0; i<max_iter; ++i){
			polytrajOptimizer.adjustCorridorConstraint(collision_seg, radius, delT);
			polytrajOptimizer.optimize();
			trajectory = polytrajOptimizer.getTrajectory(delT);
			collision_idx.clear();
			valid = mapModuleOctomap.checkCollisionTrajectory(trajectory, collision_idx);
			if (valid){
				break;
			}
			radius *= 0.9;
			cout << "[Planner INFO]: " << "radius: " << radius << endl;
		}
	}

	return trajectory;
}


#endif