#include<trajectory_optimization/mapModuleOctomap.h>

mapModule::mapModule(const ros::NodeHandle& _nh): nh(_nh){
	res = 0.1;
	octomap_client = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
	xsize = 0.3; ysize = 0.3; zsize = 0.1;
}


mapModule::mapModule(const ros::NodeHandle& _nh, double _res): nh(_nh){
	res = _res;
	octomap_client = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
	xsize = 0.3; ysize = 0.3; zsize = 0.1;
}

mapModule::mapModule(const ros::NodeHandle& _nh, double _res, double _xsize, double _ysize, double _zsize){
	res = _res;
	octomap_client = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
	xsize = _xsize; ysize = _ysize; zsize = _zsize;
}

void mapModule::updateMap(){
	octomap_msgs::GetOctomap map_srv;
	bool service_success = octomap_client.call(map_srv);
	ros::Rate rate(1);
	while (not service_success and ros::ok()){
		service_success = octomap_client.call(map_srv);
		ROS_INFO("Wait for Octomap Service...");
		rate.sleep();
	}
	abtree = octomap_msgs::binaryMsgToMap(map_srv.response.map);
	tree_ptr = dynamic_cast<OcTree*>(abtree);
	tree_ptr->setResolution(res);
}


bool mapModule::checkCollision(point3d p, bool ignore_unknown){
	//True: no collision, False: collision
	OcTreeNode* nptr = tree_ptr->search(p);
	if (nptr == NULL){
		if (not ignore_unknown){
			return false;
		}
		else{
			return true;
		}
	}
	return !(tree_ptr->isNodeOccupied(nptr));
}

bool mapModule::checkCollision(pose p, bool ignore_unknown){
	point3d p_point3d (p.x, p.y, p.z);
	return this->checkCollision(p_point3d, ignore_unknown);
}

bool mapModule::checkCollisionRobot(point3d p, bool ignore_unknown){
	double xmin, xmax, ymin, ymax, zmin, zmax; // bounding box for collision checking
	xmin = p.x() - xsize/2; xmax = p.x() + xsize/2;
	ymin = p.y() - ysize/2; ymax = p.y() + ysize/2;
	zmin = p.z() - zsize/2; zmax = p.z() + zsize/2;

	for (double x=xmin; x<xmax; x+=res){
		for (double y=ymin; y<ymax; y+=res){
			for (double z=zmin; z<zmax; z+=res){
				if (this->checkCollision(point3d (x, y, z)), ignore_unknown){
					// do nothing
				}
				else{
					return false;
				}
			}
		}
	}
	return true;
}

bool mapModule::checkCollisionRobot(pose p, bool ignore_unknown){
	point3d p_point3d (p.x, p.y, p.z);
	return this->checkCollisionRobot(p_point3d, ignore_unknown);
}

bool mapModule::checkCollisionTrajectory(const std::vector<pose>& trajectory, std::vector<int>& collision_idx, bool ignore_unknown){
	bool valid = true;
	int count = 0;
	for (pose p: trajectory){
		if (!this->checkCollisionRobot(p, ignore_unknown)){
			valid = false;
			collision_idx.push_back(count);
		}
		++count;
	}
	cout << "[MapModule INFO]: " << "Trajectory is valid? " << valid << endl;
	return valid;
}

bool mapModule::checkCollisionLine(point3d p1, point3d p2, bool ignore_unknown){
	// True: no collision; False: collision
	std::vector<point3d> ray;
	tree_ptr->computeRay(p1, p2, ray);
	for (point3d p: ray){
		if (!this->checkCollisionRobot(p, ignore_unknown)){
			return false;
		}
	}
	return true;
}

bool mapModule::checkCollisionLine(pose p1, pose p2, bool ignore_unknown){
	point3d p1_point3d (p1.x, p1.y, p1.z);
	point3d p2_point3d (p2.x, p2.y, p2.z);
	return this->checkCollisionLine(p1_point3d, p2_point3d, ignore_unknown);
}

std::vector<pose> mapModule::shortcutWaypointPath(std::vector<pose> path, bool ignore_unknown){
	std::vector<pose> path_sc;
	int ptr1 = 0; int ptr2 = 1;
	path_sc.push_back(path[ptr1]);
	while (true and ros::ok()){
		pose p1 = path[ptr1]; pose p2 = path[ptr2];
		if (checkCollisionLine(p1, p2, ignore_unknown)){
			if (ptr2 >= path.size()-1){
				path_sc.push_back(p2);
				break;
			}
			++ptr2;
		}
		else{
			path_sc.push_back(path[ptr2-1]);
			ptr1 = ptr2-1;
		}
	}
	return path_sc;
}


double mapModule::getMapResolution(){
	return tree_ptr->getResolution();
}