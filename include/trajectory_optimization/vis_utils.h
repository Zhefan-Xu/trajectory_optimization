#ifndef VIS_UTILS_H
#define VIS_UTILS_H
#include <trajectory_optimization/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

visualization_msgs::MarkerArray wrapVisMsg(const std::vector<pose> &path, double r=0, double g=0, double b=0){
		visualization_msgs::MarkerArray msg;
		std::vector<visualization_msgs::Marker> path_vis_array;
		std::vector<geometry_msgs::Point> line_vis;
		visualization_msgs::Marker line_marker;
		visualization_msgs::Marker waypoint;
		visualization_msgs::Marker direction;
		for (int i=0; i < path.size(); ++i){
				// line
				if (i<path.size()-1){
					geometry_msgs::Point p1, p2;
					p1.x = path[i].x;
					p1.y = path[i].y;
					p1.z = path[i].z;
					p2.x = path[i+1].x;
					p2.y = path[i+1].y;
					p2.z = path[i+1].z;
					line_vis.push_back(p1);
					line_vis.push_back(p2);
				}

				// waypoint
				waypoint.header.frame_id = "map";
				waypoint.id = 8888+i;
				waypoint.type = visualization_msgs::Marker::SPHERE;
				waypoint.pose.position.x = path[i].x;
				waypoint.pose.position.y = path[i].y;
				waypoint.pose.position.z = path[i].z;
				waypoint.lifetime = ros::Duration(0.5);
				waypoint.scale.x = 0.1;
				waypoint.scale.y = 0.1;
				waypoint.scale.z = 0.1;
				waypoint.color.a = 1.0;
				waypoint.color.r = 1.0;
				waypoint.color.g = 0.0;
				waypoint.color.b = 0.0;

				// Direction
				direction.header.frame_id = "map";
				direction.id = 66666+i;
				direction.type = visualization_msgs::Marker::ARROW;
				direction.pose.position.x = path[i].x;
				direction.pose.position.y = path[i].y;
				direction.pose.position.z = path[i].z;
				// tf2::Quaternion quat;
				// quat.setRPY(0, 0, path[i].yaw);
				geometry_msgs::Quaternion quat = quaternion_from_rpy(0, 0, path[i].yaw);
				// direction.pose.orientation.x = quat[0];
				// direction.pose.orientation.y = quat[1];
				// direction.pose.orientation.z = quat[2];
				// direction.pose.orientation.w = quat[3];
				direction.pose.orientation = quat;
				direction.lifetime = ros::Duration(0.5);
				direction.scale.x = 0.3;
				direction.scale.y = 0.03;
				direction.scale.z = 0.03;
				direction.color.a = 1;
				direction.color.r = 0;
				direction.color.g = 0;
				direction.color.b = 1;


				path_vis_array.push_back(waypoint);
				path_vis_array.push_back(direction);
			}
			line_marker.header.frame_id = "map";
			line_marker.points = line_vis;
			line_marker.id = 1000000;
			line_marker.type = visualization_msgs::Marker::LINE_LIST;
			line_marker.scale.x = 0.05;
			line_marker.scale.y = 0.05;
			line_marker.scale.z = 0.05;
			line_marker.color.a = 1.0;
			line_marker.color.r = r;
			line_marker.color.g = g;
			line_marker.color.b = b;
			line_marker.lifetime = ros::Duration(0.5);
			path_vis_array.push_back(line_marker);
			msg.markers = path_vis_array;
	return msg;
}

nav_msgs::Path wrapPathMsg(const std::vector<pose> &path){
	nav_msgs::Path msg;
	std::vector<geometry_msgs::PoseStamped> poses;
	for (int i=0; i<path.size(); ++i){
		pose pt = path[i];
		geometry_msgs::PoseStamped p;
		p.pose.position.x = pt.x; p.pose.position.y = pt.y; p.pose.position.z = pt.z;
		geometry_msgs::Quaternion quat = quaternion_from_rpy(0, 0, pt.yaw);
		p.pose.orientation = quat;
		poses.push_back(p);
	}
	msg.poses = poses;
	msg.header.frame_id = "map";
	return msg;
}

visualization_msgs::MarkerArray wrapObstacleMsg(const std::vector<obstacle> &obstacles){
	visualization_msgs::MarkerArray msg;
	std::vector<visualization_msgs::Marker> markers;
	int i = 0;
	for (obstacle ob: obstacles){
		visualization_msgs::Marker m;
		m.header.frame_id = "map";
		m.header.stamp = ros::Time();
		m.ns = "obstacles";
		m.id = 99999 + i; ++i;
		m.type = visualization_msgs::Marker::SPHERE;
		m.action = visualization_msgs::Marker::ADD;
		m.pose.position.x = ob.x;
		m.pose.position.y = ob.y;
		m.pose.position.z = ob.z;
		m.pose.orientation.x = 0;
		m.pose.orientation.y = 0;
		m.pose.orientation.z = 0;
		m.pose.orientation.w = 1;
		m.scale.x = ob.xsize;
		m.scale.y = ob.ysize;
		m.scale.z = ob.zsize;
		m.color.a = 1.0;
		m.color.r = 1.0;
		m.color.g = 0.0;
		m.color.b = 0.0;
		markers.push_back(m);
	}
	// cout << markers.size() << endl;
	msg.markers = markers;

	return msg;
}

#endif