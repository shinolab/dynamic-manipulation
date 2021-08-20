#include <fstream>
#include <iostream>
#include <string>
#include "FloatingObject.hpp"


void read_line(std::ifstream& ifs, DWORD& time, float& x, float& y, float& z) {
	std::string str_t, str_x, str_y, str_z;
	char delim = ',';
	std::getline(ifs, str_t, delim);
	std::getline(ifs, str_x, delim);
	std::getline(ifs, str_y, delim);
	std::getline(ifs, str_z);
	time = std::atoll(str_t.c_str());
	x = std::atof(str_x.c_str());
	y = std::atof(str_y.c_str());
	z = std::atof(str_z.c_str());
}

/*
* nCk = (n-1)_C_k + n_C_(k-1)
*/
std::vector<std::vector<int>> combination(int max, int num) {
	if (num == 1) {
		std::vector<std::vector<int>> v;
		for (int i = 0; i <= max; i++) {
			v.push_back({ i });
		}
		return v;
	}
	if (num == max+1) {
		std::vector<int> v;
		for (int i = 0; i <= max; i++) {
			v.push_back(i);
		}
		return { v };
	}
	auto others = combination(max - 1, num);
	auto inc = combination(max - 1, num - 1);
	for (auto itr = inc.begin(); itr != inc.end(); itr++) {
		itr->push_back(max);
	}
	inc.insert(inc.end(), others.begin(), others.end());
	return inc;
}

int main(int argc, char** argv) {
	int num_device = 11;
	int num_slice = 3;
	auto c = combination(num_device, num_slice);
	for (auto itr = c.begin(); itr != c.end(); itr++) {
		std::cout << "{ ";
		for (auto itr_e = itr->begin(); itr_e != itr->end(); itr_e++) {
			std::cout << *itr_e << ",";
		}
		std::cout << "}" << std::endl;
	}
	std::cout << "Total: " << c.size() << std::endl;
	return 0;

	std::ifstream ifs("20210819_Bangbang_Obs_x2.csv");
	if (ifs.is_open()) {
		std::cout << "file opened" << std::endl;
	}
	std::string header;
	std::getline(ifs, header);


	auto pObject = dynaman::FloatingObject::Create(
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::Constant(-1000),
		Eigen::Vector3f::Constant(1000),
		0.0f,
		50.0f
	);

	char input;
	do {
		DWORD time;
		float x, y, z;
		read_line(ifs, time, x, y, z);
		pObject->updateStates(time, Eigen::Vector3f(x, y, z));

		Eigen::Vector3f pos, vel, integ;
		pObject->getStates(pos, vel, integ);
		std::cout
			//<< "x:" << x << ", y: " << y << ", z:" << z << ", "
			<< "time: " << time
			<< " pos: " << pos.x() //<< ", " << pos.y() << ", " << pos.z() << ", "
			<< " vel: " << vel.x();//<< ", " << vel.y() << ", " << vel.z();
		input = getchar();
	} while (input != 'q');

	return 0;
}