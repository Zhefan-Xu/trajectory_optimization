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


bool mapModule::checkCollision(point3d p){
	OcTreeNode* nptr = tree_ptr->search(p);
	if (nptr == NULL){return false;}
	return !(tree_ptr->isNodeOccupied(nptr));
}

bool mapModule::checkCollision(pose p){
	point3d p_point3d (p.x, p.y, p.z);
	return this->checkCollision(p_point3d);
}

bool mapModule::checkCollisionRobot(point3d p){
	// TODO
	return true;
}

bool mapModule::checkCollisionRobot(pose p){
	point3d p_point3d (p.x, p.y, p.z);
	return this->checkCollisionRobot(p_point3d);
}

bool mapModule::checkCollisionTrajectory(std::vector<pose> trajectory, std::vector<pose>& collision_poses, int& collision_idx){
	// TODO
	return true;
}

bool mapModule::checkCollisionLine(point3d p1, point3d p2){
	// TODO
	return true;
}

bool mapModule::checkCollisionLine(pose p1, pose p2){
	point3d p1_point3d (p1.x, p1.y, p1.z);
	point3d p2_point3d (p2.x, p2.y, p2.z);
	return this->checkCollisionLine(p1_point3d, p2_point3d);
}

std::vector<pose> mapModule::shortcutWaypointPath(std::vector<pose> path){
	// TODO
	std::vector<pose> path_sc;
	return path_sc;
}


double mapModule::getMapResolution(){
	return tree_ptr->getResolution();
}
