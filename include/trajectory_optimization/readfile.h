#ifndef READFILE_H
#define READFILE_H
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <trajectory_optimization/polyTrajGen.h> // for pose

using namespace std;
// struct pose{
// 	double x;
// 	double y;
// 	double z;
// 	double yaw;
// };

std::vector<std::vector<pose>> read_waypoint_file(std::string filename){
	std::string line;
	std::ifstream waypoint_file (filename);

	int path_number = 0;
	std::vector<pose> path;
	std::vector<std::vector<pose>> paths;
	if (waypoint_file.is_open()){
		while (getline(waypoint_file, line)){
			// cout << line << endl;
			if (line.find("N") != std::string::npos){
				if (path_number != 0){
					paths.push_back(path);
				}

				++path_number;
				path.clear();
				continue;
			}

			double x, y, z, yaw;
			std::string num = "";
			double numd;
			pose p;

			int variable_count = 0;
			for (int i=0; i<line.size(); ++i){
				if (isspace(line[i]) or i == line.size()-1){			
					// cout << num << endl;
					numd = stod(num);
					if (variable_count == 0){
						p.x = numd;
					}
					else if (variable_count == 1){
						p.y = numd;
					}
					else if (variable_count == 2){
						p.z = numd;
					}
					else if (variable_count == 3){
						p.yaw = numd;
					}

					num = "";
					++variable_count;
				}
				else{
					num += line[i];
				}
			}
			if (line.size() != 0){
				path.push_back(p);
			}
		}
		paths.push_back(path);
		waypoint_file.close();
	}
	else{
		cout << "unable to open file" << endl;
	}
	return paths;
}

#endif