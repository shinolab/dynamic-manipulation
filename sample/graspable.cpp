#include <vector>
#include <memory>
#include <thread>
#include <iostream>
#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include "stb_easy_font.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "StereoTracker.hpp"
#include "autd3.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"
#include "pcl_viewer.hpp"

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

pcl_ptr passthrough_pcl(pcl_ptr cloud, const std::string& field_name, float limit_min, float limit_max) {
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName(field_name);
	pass.setFilterLimits(limit_min, limit_max);
	pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pass.filter(*cloud_filtered);
	return cloud_filtered;
}

pcl_ptr points_to_pcl(const rs2::points& points) {
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto pVertex = points.get_vertices();
	for (auto& p : cloud->points) {
		p.x = pVertex->x;
		p.y = pVertex->y;
		p.z = pVertex->z;
		pVertex++;
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
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
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
	
	std::thread t_viewer(
		[&pObject]() {

			rs2::pipeline pipe;
			rs2::config cfg;
			cfg.enable_stream(RS2_STREAM_DEPTH);
			pipe.start(cfg);
			Eigen::Matrix3f rot_rs;
			rot_rs <<
				-0.698625, -0.0134938, 0.715361,
				-0.71529, -0.0103563, -0.698751,
				0.0168372, -0.999855, -0.00241686;
			Eigen::Vector3f pos_rs(-409.233, 460.217, -7.72512);
			Eigen::Affine3f affine_rs
				= Eigen::Translation3f(0.001f * pos_rs) * rot_rs;

			std::cout << "running viewer" << std::endl;
			pcl_viewer viewer("pointcloud", 1280, 720);
			while (viewer) {
				rs2::frameset frame = pipe.wait_for_frames();
				auto depth = frame.get_depth_frame();
				rs2::pointcloud points_rs;

				//std::cout << "frame aquired." << std::endl;
				//std::cout << "converting depth to point cloud" << std::endl;
				auto cloud = points_to_pcl(points_rs.calculate(depth));
				auto cloud_filtered_x = passthrough_pcl(cloud, "x", -0.5f, 0.5f);
				auto cloud_filtered_xy = passthrough_pcl(cloud, "y", -0.5f, 0.5f);
				auto cloud_filtered_xyz = passthrough_pcl(cloud, "z", -0.5f, 0.8f);
				pcl_ptr cloud_global(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::transformPointCloud(*cloud_filtered_xyz, *cloud_global, affine_rs);

				pcl_ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
				pcl::VoxelGrid<pcl::PointXYZ> sor;
				sor.setInputCloud(cloud_global);
				sor.setLeafSize(0.005f, 0.005f, 0.005f);
				sor.filter(*cloud_voxel_filtered);
				auto pos_balloon = pObject->getPosition();
				auto balloon_cloud = make_sphere(0.001f * pos_balloon, 0.01f);

				pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
				kdtree.setInputCloud(cloud_voxel_filtered);
				pcl::PointXYZ searchPoint(
					0.001f * pos_balloon.x(),
					0.001f * pos_balloon.y(),
					0.001f * pos_balloon.z()
				);
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				float contact_threshold_min = 0.07f;
				float contact_threshold_max = 0.1f;
				kdtree.radiusSearch(
					searchPoint,
					contact_threshold_max,
					pointIdxRadiusSearch,
					pointRadiusSquaredDistance
				);
				auto itr_contact_min = std::find_if(
					pointRadiusSquaredDistance.begin(),
					pointRadiusSquaredDistance.end(),
					[&contact_threshold_min](float dist) { return dist > contact_threshold_min * contact_threshold_min; }
				);
				int num_points_contact = std::distance(itr_contact_min, pointRadiusSquaredDistance.end());
				if (num_points_contact > 30) {
					std::cout << "contact ( num_points_contact: " << num_points_contact << ")" <<std::endl;
				}
				auto threshold_min = make_sphere(0.001f * pos_balloon, contact_threshold_min);
				auto threshold_max = make_sphere(0.001f * pos_balloon, contact_threshold_max);

				std::vector<pcl_ptr> cloud_ptrs{ cloud_voxel_filtered, balloon_cloud, threshold_min, threshold_max};
				viewer.draw(cloud_ptrs);
			}	
		}
	);

	t_viewer.join();
	pManipulator->FinishManipulation();
	return 0;
}


