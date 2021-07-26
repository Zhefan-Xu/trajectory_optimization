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



class mavrosTest{
private:
	ros::NodeHandle nh;
	nav_msgs::Odometry current_odom;
	mavros_msgs::State current_state;

	ros::Subscriber odom_sub;
	ros::Subscriber state_sub;

	ros::Publisher goal_pub;

	ros::ServiceClient gazebo_setModel_client;
	ros::ServiceClient arming_client;
	ros::ServiceClient set_mode_client;

	bool odom_received, state_received;
	std::vector<pose> path;
	mavros_msgs::PositionTarget goal;


public:
	mavrosTest(const ros::NodeHandle &_nh);
	void odom_cb(const nav_msgs::OdometryConstPtr& odom);
	void state_cb(const mavros_msgs::State::ConstPtr& mavros_state);
	void loadPath(std::string filename, int idx);
	void setInitialPosition();
};



#endif