#ifndef UTILS_H
#define UTILS_H
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>

#define PI_const 3.1415926
using namespace std;




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
	// in the planner, the angle is [0, 2pi] instead of [-pi, pi]
	// if (yaw < 0){
	// 	yaw = 2 * PI_const - (-yaw);
	// }
	return yaw;
}

#endif