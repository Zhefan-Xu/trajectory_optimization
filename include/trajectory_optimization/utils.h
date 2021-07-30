#ifndef UTILS_H
#define UTILS_H
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>

#define PI_const 3.1415926
using namespace std;


struct pose{
	double x;
	double y;
	double z;
	double yaw;
};

struct obstacle{
	double x, y, z;
	double xsize, ysize, zsize;
	double vx, vy, vz;
	double varX, varY, varZ;
};


geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
	if (yaw > PI_const){
		yaw = yaw - 2*PI_const;
	}
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    return quaternion;
}

double rpy_from_quaternion(geometry_msgs::Quaternion quat){
	// return is [0, 2pi]
	tf2::Quaternion tf_quat;
	tf2::convert(quat, tf_quat);
	double roll, pitch, yaw;
	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
	return yaw;
}

void rpy_from_quaternion(geometry_msgs::Quaternion quat, double &roll, double &pitch, double &yaw){
	tf2::Quaternion tf_quat;
	tf2::convert(quat, tf_quat);
	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
}

double getDistance(pose p1, pose p2){
	return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));	

}

#endif