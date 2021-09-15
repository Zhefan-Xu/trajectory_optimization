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
		ob.xsize = 0.5; ob.ysize = 0.5; ob.zsize = 2.2;
		double pos_var = 1e-4; double vel_var = 1e-4;
		// double pos_var = 4e-4; double vel_var = 4e-4;
		ob.varX = pos_var; ob.varY = pos_var; ob.varZ = pos_var;
		ob.varVx = vel_var; ob.varVy = vel_var; ob.varVz = vel_var;
		std::random_device rd{};
    	std::mt19937 gen{rd()};
    	std::normal_distribution<> position_d{0, sqrt(pos_var)};
    	std::normal_distribution<> velocity_d{0, sqrt(vel_var)};

    	ob.x = ob.x + position_d(gen);
		ob.y = ob.y + position_d(gen);
		ob.z = ob.z + position_d(gen);
		ob.vx = ob.vx + velocity_d(gen);
		ob.vy = ob.vy + velocity_d(gen);
		ob.vz = ob.vz + velocity_d(gen);
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
