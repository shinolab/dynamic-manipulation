#include <iostream>
#include <chrono>
#include <thread>
#include "pcl_viewer.hpp"
#include "librealsense2/rs.hpp"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl_ptr points_to_pcl(const rs2::points& points) {
	pcl_ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud_ptr->width = sp.width();
	cloud_ptr->height = sp.height();
	cloud_ptr->is_dense = false;
	cloud_ptr->points.resize(points.size());
	auto pVertex = points.get_vertices();
	for (auto& p : cloud_ptr->points) {
		p.x = pVertex->x;
		p.y = pVertex->y;
		p.z = pVertex->z;
		pVertex++;
	}
	return cloud_ptr;
}

int main(int argc, char** argv) {

	pcl_viewer viewer("pointcloud", 1280, 720);
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH);
	pipe.start(cfg);
	
	while (viewer) {
		rs2::frameset frame = pipe.wait_for_frames();
		auto depth_frame = frame.get_depth_frame();
		rs2::pointcloud points_rs;

		auto cloud = points_to_pcl(points_rs.calculate(depth_frame));
		std::vector<pcl_ptr> clouds{ cloud };
		viewer.draw(clouds);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return 0;
}