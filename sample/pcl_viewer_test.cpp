#include <iostream>
#include <chrono>
#include <thread>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl_util.hpp"
#include "pcl_viewer.hpp"
#include "pcl_grabber.hpp"
#include "FloatingObject.hpp"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

int main(int argc, char** argv) {
	Eigen::Matrix3f rot;
	rot<<
		-0.698625, -0.0134938, 0.715361,
		-0.71529, -0.0103563, -0.698751,
		0.0168372, -0.999855, -0.00241686;
	Eigen::Vector3f pos(-409.233, 460.217, -7.72512);

	auto grabber = rs2_pcl_grabber::Create(0.001*pos, rot, "825513025618", 0.15, 0.5);
	//auto grabber = rs2_pcl_grabber::Create(Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(), "001622070259", 0.15, 0.5);
	
	grabber->Open();
	pcl_viewer viewer("pointcloud", 1280, 720);
	
	auto pObject = dynaman::FloatingObject::Create(Eigen::Vector3f::Zero(),
		Eigen::Vector3f::Constant(-500),
		Eigen::Vector3f::Constant(1000),
		0,
		50
	);

	while (viewer) {
		auto pCloud = grabber->Capture();
		auto pCloudTrim = pcl_util::TrimPointsOutsideWorkspace(pObject, pCloud);
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
		vg.setInputCloud(pCloudTrim);
		vg.setLeafSize(0.005f, 0.005f, 0.005f);
		vg.filter(*pCloudFiltered);

		std::vector<pcl_ptr> clouds{ pCloudFiltered };
		auto clusterIndices = pcl_util::EuclidianClusterExtraction(pCloudFiltered, 0.05f, 10, 1000000);
		std::cout << clusterIndices.size() << " clusters are detected." << std::endl;
		viewer.draw(clouds);
		std::this_thread::sleep_for(std::chrono::milliseconds(15));
	}
	return 0;
}