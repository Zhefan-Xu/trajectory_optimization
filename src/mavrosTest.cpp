#include <trajectory_optimization/mavrosTest.h>

mavrosTest::mavrosTest(const ros::NodeHandle &_nh, double _delT=0.1):nh(_nh){
	this->odom_received = false;
	this->state_received = false;
	this->delT = _delT;
	odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &mavrosTest::odom_cb, this);
	state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &mavrosTest::state_cb, this);

	goal_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
	path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/waypoint_path", 0);
    trajectory_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory", 0);
    mpc_trajectory_vis_pub = nh.advertise<nav_msgs::Path>("/mpc_trajectory", 0);
    obstacle_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/obstacles", 0);
 	waypoint_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/waypoints", 0);
    
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
	waypoint_msg = wrapWaypointMsg(this->path, 1, 1, 0);
} 

void mavrosTest::loadObstacleType(const std::vector<std::string> &_obstaclesType){
	this->obstaclesType = _obstaclesType;
}

void mavrosTest::setInitialPosition(){
	std::string modelName = "iris";
	double setX = this->path[0].x; double setY = this->path[0].y;
	double setZ = 0; //this->current_odom.pose.pose.position.z;
	gazebo_msgs::SetModelState state_srv;
	gazebo_msgs::ModelState modelState;
	modelState.model_name = modelName;
	modelState.pose.position.x = setX; modelState.pose.position.y = setY; modelState.pose.position.z = setZ;
	geometry_msgs::Quaternion quat = quaternion_from_rpy(0, 0, this->path[0].yaw);
	modelState.pose.orientation = quat; 
	state_srv.request.model_state = modelState;
	gazebo_setModel_client.call(state_srv);
}

void mavrosTest::run(){
	goal_worker_ = std::thread(&mavrosTest::publishGoal, this);
	vis_worker_ = std::thread(&mavrosTest::publishVisMsg, this);
	// bool write_file = false;

	// takeoff at the first given point attitude:
	this->takeoff();

	// static planner:
	// parameters
	double res = 0.1; // map resolution
	double xsize = 0.2; double ysize = 0.2; double zsize = 0.1; // Robot collision box size
	int degree = 7; // polynomial degree
	double velocityd = 1.5; // desired average velocity
	int diff_degree = 4; // Minimum snap (4), minimum jerk (3)
	double perturb = 1; // Regularization term and also make PSD -> PD
	// bool shortcut = true; // shortcut waypoints
	bool shortcut = false; // shortcut waypoints
	std::vector<pose> loadPath;


	// solve trajectory:
	int num_run = 19;
	// std::ofstream static_planner_data; std::string static_filename = "/home/zhefan/Desktop/experiment_data/static/static" + std::to_string(num_run) +".txt";
	// static_planner_data.open(static_filename);
	auto start_time_static = high_resolution_clock::now();
	std::vector<pose> trajectory = staticPlanner(nh, res, xsize, ysize, zsize, degree, velocityd, diff_degree, perturb, path, shortcut, delT, loadPath);
	auto end_time_static = high_resolution_clock::now();
	auto duration_total_static = duration_cast<microseconds>(end_time_static - start_time_static);
	cout << "Total: "<< duration_total_static.count()/1e6 << " seconds. " << endl;	
	// if (write_file){
	// 	static_planner_data << duration_total_static.count()/1e6 << endl;
	// }
	path_msg = wrapVisMsg(loadPath, 0, 0, 0);
	trajectory_msg = wrapVisMsg(trajectory, 0, 1, 0);

	// MPC
	int horizon = 20; // MPC horizon

	double mass = 1.0; double k_roll = 1.0; double tau_roll = 1.0; double k_pitch = 1.0; double tau_pitch = 1.0; 
	double T_max = 2.0 * 9.8; double roll_max = PI_const/6; double pitch_max = PI_const/6;

	
	double currentYaw;
	DVector currentStates = this->getCurrentState(currentYaw);
	DVector nextStates;
	std::vector<pose> mpc_trajectory;
	VariablesGrid xd;

	// Detector:
	// std::vector<std::string> obstaclesType {"person_walking", "person_walking_0"};
	fakeDetector d (nh);
	d.loadObstacleType(obstaclesType);

	ros::Rate rate(1/this->delT);
	// std::ofstream dynamic_planner_data; std::string dynamic_filename = "/home/zhefan/Desktop/experiment_data/dynamic/dynamic" + std::to_string(num_run) +".txt";
	// std::ofstream dynamic_planner_obstacle_data; std::string dynamic_obstacle_filename = "/home/zhefan/Desktop/experiment_data/dynamic_obstacle/dynamic_obstacle" + std::to_string(num_run) +".txt";
	// std::ofstream obstacle_distance_data; std::string obstacle_distance_filename = "/home/zhefan/Desktop/experiment_data/distance/distance" + std::to_string(num_run) +".txt";
	// if (write_file){
	// 	dynamic_planner_data.open(dynamic_filename);
	// 	dynamic_planner_obstacle_data.open(dynamic_obstacle_filename);
	// 	obstacle_distance_data.open(obstacle_distance_filename);
	// }
	while (ros::ok()){
		std::vector<obstacle> obstacles; d.detect(obstacles);
		auto start_time = high_resolution_clock::now();
		int obstacle_idx = -1;
		mpc_trajectory = dynamicPlanner(trajectory, obstacles, horizon, mass, k_roll, tau_roll, k_pitch, tau_pitch, T_max, roll_max, pitch_max, delT, currentStates, currentYaw, nextStates, xd, obstacle_idx);
		// currentStates = nextStates;
		currentStates = this->getCurrentState(currentYaw); 
		// currentStates(3) = nextStates(3); currentStates(4) = nextStates(4); currentStates(5) = nextStates(5); 
		currentStates(6) = nextStates(6); currentStates(7) = nextStates(7);

		auto end_time = high_resolution_clock::now();
		auto duration_total = duration_cast<microseconds>(end_time - start_time);
		cout << "Total: "<< duration_total.count()/1e6 << " seconds. " << endl;		
		// if (write_file){
		// 	if (obstacle_idx != -1){
		// 		dynamic_planner_obstacle_data << duration_total.count()/1e6 << endl;

		// 	}
		// 	else{
		// 		dynamic_planner_data << duration_total.count()/1e6 << endl;
		// 	}
		

		// 	double obstacle_distance = getDistance(obstacles[1], pose(currentStates(0), currentStates(1), currentStates(2)));
		// 	if (obstacle_distance < 5){
		// 		obstacle_distance_data << obstacle_distance << endl;
		// 	}

		// }
		this->modifyMPCGoal(mpc_trajectory, xd);
		mpc_trajectory_msg = wrapPathMsg(mpc_trajectory);

		obstacle_msg = wrapObstacleMsg(obstacles);
		
		rate.sleep();
	}
}

// TODO detect obstacles:

void mavrosTest::takeoff(){
	bool takeoff = false;
	goal.position.x = this->path[0].x; goal.position.y = this->path[0].y; goal.position.z = this->path[0].z;
	goal.yaw = this->path[0].yaw; goal.coordinate_frame = goal.FRAME_LOCAL_NED; goal.header.frame_id = "map";

	ros::Rate rate(1/this->delT);
	while (takeoff == false){
		takeoff = this->isReach();
		rate.sleep();
	}
	cout << "[mavros test INFO]: " << "takeoff success!" << endl;
}


void mavrosTest::publishGoal(){
	ros::Rate rate(1/this->delT);
	for(int i = 100; ros::ok() && i > 0; --i){
        goal.header.stamp = ros::Time::now();
        goal_pub.publish(goal);
        ros::spinOnce();
        rate.sleep();
    }

	ros::Time last_request = ros::Time::now();
	mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
	while (ros::ok()){	
		if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
		goal.header.stamp = ros::Time::now();
		goal_pub.publish(goal);
		// cout << "[PUBLISHED GOAL INFO]: " << goal.position.x << " " << goal.position.y << " " << goal.position.z << endl;
		ros::spinOnce();
		rate.sleep();
	}
}

void mavrosTest::publishVisMsg(){
	ros::Rate rate(10);
	while (ros::ok()){
		path_vis_pub.publish(path_msg);
        trajectory_vis_pub.publish(trajectory_msg);
        mpc_trajectory_vis_pub.publish(mpc_trajectory_msg);
        obstacle_vis_pub.publish(obstacle_msg);
 		waypoint_vis_pub.publish(waypoint_msg);
        rate.sleep();
	}
}

bool mavrosTest::isReach(){
	double dx = std::abs(this->current_odom.pose.pose.position.x - this->goal.position.x);
	double dy = std::abs(this->current_odom.pose.pose.position.y - this->goal.position.y);
	double dz = std::abs(this->current_odom.pose.pose.position.z - this->goal.position.z);
	double eps = 0.1;
	if (dx < eps and dy < eps and dz < eps){
		return true;
	}
	else{
		return false;
	}
}

DVector mavrosTest::getCurrentState(double &currentYaw){
	// TODO implement this
	// convert odom to states
	DVector currentStates(8); currentStates.setAll(0.0);

	// position
	currentStates(0) = this->current_odom.pose.pose.position.x; currentStates(1) = this->current_odom.pose.pose.position.y; currentStates(2) = this->current_odom.pose.pose.position.z; 

	// velocity:
	// TODO: coordinate transform 
	std::string body_frame = this->current_odom.child_frame_id; std::string map_frame = this->current_odom.header.frame_id;
	double vx_body = this->current_odom.twist.twist.linear.x; double vy_body = this->current_odom.twist.twist.linear.y; double vz_body = this->current_odom.twist.twist.linear.z; 
	geometry_msgs::Vector3Stamped velocity_body; 
	velocity_body.header.frame_id = body_frame;
	velocity_body.vector.x = vx_body; velocity_body.vector.y = vy_body; velocity_body.vector.z = vz_body;
	geometry_msgs::Vector3Stamped velocity_map;
	tf_listener.transformVector(map_frame, velocity_body, velocity_map);
	currentStates(4) = velocity_map.vector.x; currentStates(5) = velocity_map.vector.y; currentStates(6) = velocity_map.vector.z;
	// cout << "[Velocity Before: ]" << velocity_body.vector.x << " " << velocity_body.vector.y << " " << velocity_body.vector.z << endl;
	// cout << "[Velocity Transformed: ]" << velocity_map.vector.x << " " << velocity_map.vector.y << " " << velocity_map.vector.z << endl;

	// listen to tf and apply the transform

	// orientation:
	double roll; double pitch; double yaw;
	rpy_from_quaternion(this->current_odom.pose.pose.orientation, roll, pitch, yaw);
	currentStates(6) = roll; currentStates(7) = pitch;  currentYaw = yaw;
	return currentStates;
}

void mavrosTest::modifyMPCGoal(const std::vector<pose> &mpc_trajectory, const VariablesGrid &xd){
	// int forward_idx = 1;
	// int forward_idx = 5;
	int forward_idx = 10;
	// goal.type_mask =  goal.IGNORE_VX + goal.IGNORE_VY + goal.IGNORE_VZ
	// 				+ goal.IGNORE_AFX + goal.IGNORE_AFY + goal.IGNORE_AFZ
	// 				+ goal.IGNORE_YAW_RATE;
	goal.type_mask =  goal.IGNORE_AFX + goal.IGNORE_AFY + goal.IGNORE_AFZ
					+ goal.IGNORE_YAW_RATE;
	double yaw = mpc_trajectory[forward_idx].yaw;
	DVector goalStates = xd.getVector(forward_idx);
	goal.position.x = goalStates(0); goal.position.y = goalStates(1); goal.position.z = goalStates(2);
	// goal.velocity.x = goalStates(3); goal.velocity.y = goalStates(4); goal.velocity.z = goalStates(5);
	// cout << goalStates << endl;
	goal.yaw = yaw; // goal.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048;
	// goal.type_mask =  8 + 16 +  32 + 64 + 128 + 256 + 2048;
	// cout << "TYPE MASK: " << goal.type_mask << endl;
	// cout << "[GOAL TARGET INFO]: " << goal.position.x << " " << goal.position.y << " " << goal.position.z << endl;
	// cout << "[GOAL TARGET INFO]: " << goal.velocity.x << " " << goal.velocity.y << " " << goal.velocity.z << endl;
}