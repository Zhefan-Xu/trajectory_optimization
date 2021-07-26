#include <trajectory_optimization/mavrosTest.h>

mavrosTest::mavrosTest(const ros::NodeHandle &_nh):nh(_nh){
	this->odom_received = false;
	this->state_received = false;
	odom_sub = nh.subscribe("/mavros/local_position/odom", 10, &mavrosTest::odom_cb, this);
	state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &mavrosTest::state_cb, this);

	goal_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    
    // goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/cerlab_uav/goal", 10);

    gazebo_setModel_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");


}

void mavrosTest::odom_cb(const nav_msgs::OdometryConstPtr& odom){
	current_odom = *odom;
	if (not this->odom_received){
		this->odom_received = true;
	}
}

void mavrosTest::state_cb(const mavros_msgs::State::ConstPtr& mavros_state){
	current_state = *mavros_state;
	if (not this->state_received){
		this->state_received = true;
	}
}

void mavrosTest::loadPath(std::string filename, int idx){
	std::vector<std::vector<pose>> paths = read_waypoint_file(filename);
	this->path = paths[idx];
}

void mavrosTest::setInitialPosition(){
	std::string modelName = "iris";
	double setX = this->path[0].x; double setY = this->path[0].y;
	double setZ = this->current_odom.pose.pose.position.z;
	gazebo_msgs::SetModelState state_srv;
	gazebo_msgs::ModelState modelState;
	modelState.model_name = modelName;
	modelState.pose.position.x = setX; modelState.pose.position.y = setY; modelState.pose.position.z = setZ;
	geometry_msgs::Quaternion quat = quaternion_from_rpy(0, 0, this->path[0].yaw);
	modelState.pose.orientation = quat; 
	state_srv.request.model_state = modelState;
	gazebo_setModel_client.call(state_srv);
}