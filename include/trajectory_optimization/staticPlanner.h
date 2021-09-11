#ifndef STATICPLANNER_H
#define STATICPLANNER_H
#include <ros/ros.h>
#include <trajectory_optimization/mapModuleOctomap.h>
#include <trajectory_optimization/polyTrajGen.h>
#include <algorithm>


std::vector<pose> staticPlanner(mapModule* mapModuleOctomap, int degree, double velocityd, int diff_degree, double perturb,
								const std::vector<pose>& path, bool shortcut, double delT, std::vector<pose>& loadPath){
	// TODO: implement the static planner

	// Initialization:
	polyTraj polytrajOptimizer (degree, velocityd, diff_degree, perturb);


	if (shortcut){ // if we need to short cut path
		loadPath = mapModuleOctomap->shortcutWaypointPath(path);
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

	int max_iter = 10; std::vector<double> radius; double init_radius = 0.5; // adaptive radius
	for (int i=0; i<collision_seg.size(); ++i){// add initial radius
		radius.push_back(init_radius);
	}
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
			valid = mapModuleOctomap->checkCollisionTrajectory(trajectory, collision_idx, true);
			if (valid){
				break;
			}
			std::set<int> radius_adjust_collision_idx = polytrajOptimizer.findCollisionSegment(collision_idx, delT);

			for (int adjust_idx: radius_adjust_collision_idx){
				radius[adjust_idx] *= 0.9;
			}
			double min_radius = *min_element(radius.begin(), radius.end());

			cout << "[Planner INFO]: " << "min radius: " << min_radius << endl;
		}
	}

	return trajectory;
}


#endif