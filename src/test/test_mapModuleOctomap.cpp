#include<ros/ros.h>
#include<trajectory_optimization/mapModuleOctomap.h>
#include <chrono> 
using namespace std::chrono;


int main(int argc, char** argv){
	ros::init(argc, argv, "test_mapModuleOctomap_node");
	ros::NodeHandle nh;
	mapModule m (nh);

	auto start_time = high_resolution_clock::now();
	m.updateMap();
	auto end_time = high_resolution_clock::now();
	auto duration_total = duration_cast<microseconds>(end_time - start_time);
	cout << "Total: "<< duration_total.count()/1e6 << " seconds. " << endl;

	cout << m.getMapResolution() << endl;

	return 0;
}