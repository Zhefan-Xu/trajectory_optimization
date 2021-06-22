#include <trajectory_optimization/polyTrajGen.h>

polyTraj::polyTraj(){

}

void polyTraj::loadWaypointPath(std::vector<pose> _path){
	this->path = _path;
}

void polyTraj::optimize(){

}

void polyTraj::printWaypointPath(){
	cout << fixed << setprecision(2); 
	if (this->path.size() == 0){
		cout << "You have to load path first!" << endl;
	}
	else{
		int count = 0;
		for (pose p: this->path){
			cout << "Waypoint " << count << ": (" << p.x << ", " << p.y << ", " << p.z << "), yaw: " << p.yaw << endl;
			if (count != 0){
				double distance = sqrt(pow((p.x - this->path[count-1].x),2) + pow((p.y - this->path[count-1].y),2) + pow((p.z - this->path[count-1].z),2));	
				cout << ">>[Gap (" << count-1 << "->" << count << "): " << distance << "m.]<<"<< endl;			
			}
			++count; 
		}
	}
}