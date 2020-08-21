#include <iostream>
#include <chrono>
#include <thread>
#include "pcl_viewer.hpp"
#include "pcl_grabber.hpp"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

int main(int argc, char** argv) {
	Eigen::Matrix3f rot;
	rot<<
		-0.698625, -0.0134938, 0.715361,
		-0.71529, -0.0103563, -0.698751,
		0.0168372, -0.999855, -0.00241686;
	Eigen::Vector3f pos(-409.233, 460.217, -7.72512);

	auto grabber = rs2_pcl_grabber::Create(0.001*pos, rot, "827312072688", 0.15, 0.5);
	//auto grabber = rs2_pcl_grabber::Create(Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(), "001622070259", 0.15, 0.5);

	grabber->Open();
	pcl_viewer viewer("pointcloud", 1280, 720);
	
	while (viewer) {
		std::vector<pcl_ptr> clouds{ grabber->Capture() };
		viewer.draw(clouds);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return 0;
}