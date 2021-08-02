#include <trajectory_optimization/fakeDetector.h>

fakeDetector::fakeDetector(const ros::NodeHandle &_nh):nh(_nh){
	cout << "[fake detector INFO]: " << "detector initialized" << endl;
	gazebo_getModel_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
}


void fakeDetector::detect(std::vector<obstacle> &obstacles){
	// TODO implement this
	gazebo_msgs::GetModelState gms_srv;
	for (std::string modelName: this->obstaclesType){
		obstacle ob;
		gms_srv.request.model_name = modelName;
		gazebo_getModel_client.call(gms_srv);
		ob.x = gms_srv.response.pose.position.x;
		ob.y = gms_srv.response.pose.position.y;
		ob.z = gms_srv.response.pose.position.z+1; // center of bounding box
		ob.vx = gms_srv.response.twist.linear.x;
		ob.vy = gms_srv.response.twist.linear.y;
		ob.vz = gms_srv.response.twist.linear.z;
		ob.xsize = 0.6; ob.ysize = 0.6; ob.zsize = 1.8;
		ob.varX = 1e-4; ob.varY = 1e-4; ob.varZ = 1e-4;
		ob.varVx = 1e-4; ob.varVy = 1e-4; ob.varVz = 1e-4;
		obstacles.push_back(ob);
		cout << "[fake detector INFO]: " << modelName << " x: " << ob.x <<  " y: " << ob.y  << " z: " << ob.z <<  " vx: " << ob.vx <<  " vy: " << ob.vy  << " vz: " << ob.vz << endl;
	}
}

void fakeDetector::loadObstacleType(const std::vector<std::string> &_obstaclesType){
	this->obstaclesType = _obstaclesType;
	cout << "[fake detector INFO]: obstalce types: ";
	for (std::string name: this->obstaclesType){
		cout << name << " ";
	}
	cout << endl;
}
