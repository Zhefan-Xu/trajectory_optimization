#include <Eigen/Eigen>
#include <iostream>

int main(){
	std::vector<double> data1 {1.1,2.2,3.3,4.4,5.5,6.6};
	// Eigen::Map<Eigen::Vector3d> v1 (data);
	Eigen::Map<Eigen::VectorXd> v1 (data1.data(), data1.size());
	std::cout << v1 << std::endl;
}