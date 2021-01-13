#include "K4aHeadTracker.hpp"
#include <fstream>
#include <iostream>
#include <Windows.h>

#pragma comment(lib, "winmm")

int main(int argc, char** argv) {
	Eigen::Vector3f pos_k4a(-3.8887, -868.463, 312.043);
	Eigen::Matrix3f rot_k4a;
	rot_k4a <<
		0.999971, 0.00766788, -0.000237879,
		0.00113676, -0.117437, 0.99308,
		0.00758687, -0.993051, -0.117443;
	auto pTracker = K4aHeadTracker::Create(pos_k4a, rot_k4a);
	pTracker->Open();
	Eigen::Vector3f zero, xGlobal, yGlobal, zGlobal;
	pTracker->TransformHead2Global(Eigen::Vector3f::Zero(), zero);

	pTracker->TransformHead2Global(100 * Eigen::Vector3f::UnitX(), xGlobal);
	pTracker->TransformHead2Global(100 * Eigen::Vector3f::UnitY(), yGlobal);
	pTracker->TransformHead2Global(100 * Eigen::Vector3f::UnitZ(), zGlobal);
	std::cout << "origin:" << zero.transpose() << std::endl;
	std::cout << "XGlobal:" << (xGlobal - zero).transpose() << std::endl;
	std::cout << "YGlobal:" << (yGlobal - zero).transpose() << std::endl;
	std::cout << "ZGlobal:" << (zGlobal - zero).transpose() << std::endl;

	std::ofstream ofs("20200916_head_tracking_log.csv");
	for (int i = 0; i < 300; i++) {
		Eigen::Vector3f front(1000, 0, 0);
		Eigen::Vector3f front_global;
		pTracker->TransformHead2Global(front, front_global);
		std::cout << timeGetTime() << "," << front_global.x() << "," << front_global.y() << "," << front_global.z() << std::endl;

		ofs << timeGetTime() <<"," << front_global.x() <<"," << front_global.y() << "," << front_global.z() << std::endl;
	}
	pTracker->Close();
	return 0;
}