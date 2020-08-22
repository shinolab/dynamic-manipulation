#include <vector>
#include <memory>
#include <iostream>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "StereoTracker.hpp"
#include "autd3.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"
#include "pcl_viewer.hpp"
#include "pcl_grabber.hpp"
#include "balloon_interface.hpp"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl_ptr make_sphere(const Eigen::Vector3f& center, float radius) {
	int num_long = 50;
	int num_lat = 25;
	int num_points = num_long * num_lat;
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(num_points);
	cloud->height = num_lat;
	cloud->width = num_long;
	for (int i_lat = 0; i_lat < num_lat; i_lat++)
	{
		for (int i_long = 0; i_long < num_long; i_long++) {
			int i_vert = i_lat * num_long + i_long;
			float theta = M_PI * i_lat / num_lat;
			float phi = 2 * M_PI * i_long / num_long;
			cloud->points[i_vert].x = center.x() + radius * sin(theta) * cos(phi);
			cloud->points[i_vert].y = center.y() + radius * sin(theta) * sin(phi);
			cloud->points[i_vert].z = center.z() + radius * cos(theta);
		}
	}
	return cloud;
}

pcl_ptr points_to_pcl(const std::vector<Eigen::Vector3f>& points) {
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = points.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	for (int iv = 0; iv < points.size(); iv++) {
		cloud->points[iv].x = points[iv].x();
		cloud->points[iv].y = points[iv].y();
		cloud->points[iv].z = points[iv].z();
	}
	return cloud;
}

int main(int argc, char** argv) {
	std::string target_image_name("blue_target_no_cover.png");
	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();
	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen()) {
		return ENXIO;
	}
	haptic_icon::SetGeometry(pAupa);

	auto pObject = dynaman::FloatingObject::Create(
		Eigen::Vector3f(0, 0, 0),
		Eigen::Vector3f::Constant(-400),
		Eigen::Vector3f::Constant(400),
		-0.036e-3f,
		50.f
	);

	auto pManipulator = dynaman::MultiplexManipulator::Create(
		20 * Eigen::Vector3f::Constant(-1.6f), // gainP
		5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		100, //freqLM
		10,
		5,
		0
	);
	pManipulator->StartManipulation(pAupa, pTracker, pObject);

	Eigen::Matrix3f rot_rs;
	rot_rs <<
		-0.698625, -0.0134938, 0.715361,
		-0.71529, -0.0103563, -0.698751,
		0.0168372, -0.999855, -0.00241686;
	Eigen::Vector3f pos_rs(-409.233, 460.217, -7.72512);
	
	auto grabber = rs2_pcl_grabber::Create(0.001f*pos_rs, rot_rs, "827312072688", 0.15f, 1.0f);
	grabber->Open();


	std::this_thread::sleep_for(std::chrono::seconds(5)); // wait until stabilized

	auto binterface = dynaman::balloon_interface::Create(
		pObject,
		grabber
	);
	binterface->Open();
	binterface->Run();
	pcl_viewer viewer("pointcloud", 1280, 720);
	while (viewer) {
		auto cloud = binterface->CopyPointCloud();
		auto pos_balloon = pObject->getPosition();
		auto threshold_min = make_sphere(0.001f * pos_balloon, binterface->RadiusColliderMin());
		auto threshold_max = make_sphere(0.001f * pos_balloon, binterface->RadiusColliderMax());
		std::vector<pcl_ptr> cloud_ptrs{ 
			cloud, 
			threshold_min, 
			threshold_max 
		};
		viewer.draw(cloud_ptrs);

		std::this_thread::sleep_for(std::chrono::microseconds(30));
	}
	binterface->Close();
	grabber->Close();
	pManipulator->FinishManipulation();
	return 0;
}