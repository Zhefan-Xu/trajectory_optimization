#include<trajectory_optimization/mapModuleOctomap.h>

mapModule::mapModule(const ros::NodeHandle& _nh): nh(_nh){
	octomap_client = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
}


void mapModule::updateMap(){
	octomap_msgs::GetOctomap map_srv;
	octomap_client.call(map_srv);
	AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(map_srv.response.map);
	tree_ptr = dynamic_cast<OcTree*>(abtree);
	tree_ptr->setResolution(0.1);
}