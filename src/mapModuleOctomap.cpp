#include<trajectory_optimization/mapModuleOctomap.h>

mapModule::mapModule(const ros::NodeHandle& _nh): nh(_nh){
	res = 0.1;
	octomap_client = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");


}


mapModule::mapModule(const ros::NodeHandle& _nh, double _res): nh(_nh){
	res = _res;
	octomap_client = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
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

double mapModule::getMapResolution(){
	return tree_ptr->getResolution();
}
