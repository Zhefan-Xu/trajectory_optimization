#ifndef MAVROSTEST_H
#define MAVROSTEST_H
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_optimization/readfile.h>
#include <trajectory_optimization/mpcPlanner.h>
#include <trajectory_optimization/staticPlanner.h>
#include <trajectory_optimization/dynamicPlanner.h>
#include <trajectory_optimization/vis_utils.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <thread>
#include <mutex>


class mavrosTest{
private:
	ros::NodeHandle nh;
	nav_msgs::Odometry current_odom;
	mavros_msgs::State current_state;

	ros::Subscriber odom_sub;
	ros::Subscriber state_sub;
	tf::TransformListener tf_listener;


	ros::Publisher goal_pub;
	ros::Publisher path_vis_pub;
	ros::Publisher trajectory_vis_pub;
	ros::Publisher mpc_trajectory_vis_pub;
	ros::Publisher obstacle_vis_pub;
	ros::Publisher waypoint_vis_pub;

	ros::ServiceClient gazebo_setModel_client;
	ros::ServiceClient arming_client;
	ros::ServiceClient set_mode_client;

	bool odom_received, state_received;
	std::vector<pose> path;
	mavros_msgs::PositionTarget goal;
	visualization_msgs::MarkerArray path_msg;
	visualization_msgs::MarkerArray trajectory_msg;
	visualization_msgs::MarkerArray obstacle_msg;
	visualization_msgs::MarkerArray waypoint_msg;
	nav_msgs::Path mpc_trajectory_msg;


	double delT;
	std::vector<std::string> obstaclesType;

public:
	std::thread goal_worker_;
	std::thread vis_worker_;

	mavrosTest(const ros::NodeHandle &_nh, double _delT);
	void odom_cb(const nav_msgs::OdometryConstPtr& odom);
	void state_cb(const mavros_msgs::State::ConstPtr& mavros_state);
	void loadPath(std::string filename, int idx);
	void loadObstacleType(const std::vector<std::string> &_obstaclesType);
	void setInitialPosition();
	void run();
	void takeoff();
	void publishGoal();
	void publishVisMsg();
	bool isReach();
	DVector getCurrentState(double &currentYaw);
	void modifyMPCGoal(const std::vector<pose> &mpc_trajectory, const VariablesGrid &xd);
};



#endif