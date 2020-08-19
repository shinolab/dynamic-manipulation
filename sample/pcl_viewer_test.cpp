#include <iostream>
#include <chrono>
#include <thread>
#include "pcl_viewer.hpp"
#include "pcl_grabber.hpp"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

int main(int argc, char** argv) {

	auto grabber = rs2_pcl_grabber::Create(Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(), "001622070259", 0.15, 0.5);
	grabber->Open();
	pcl_viewer viewer("pointcloud", 1280, 720);
	
	while (viewer) {
		std::vector<pcl_ptr> clouds{ grabber->Capture() };
		viewer.draw(clouds);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return 0;
}