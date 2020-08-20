#include "pcl_grabber.hpp"
#include <pcl/common/transforms.h>
#include <librealsense2/rs.hpp>

rs2_pcl_grabber::rs2_pcl_grabber(const Eigen::Vector3f& pos, const Eigen::Matrix3f& rot, const std::string& id, float range_min, float range_max):
	m_affine(Eigen::Translation3f(pos) * rot), m_id(id), m_thres_filter(range_min, range_max){}

rs2_pcl_grabber::~rs2_pcl_grabber() {}

std::shared_ptr<pcl_grabber> rs2_pcl_grabber::Create(const Eigen::Vector3f& pos, const Eigen::Matrix3f& rot, const std::string& id, float range_min, float range_max) {
	return std::make_shared<rs2_pcl_grabber>(pos, rot, id, range_min, range_max);
}

void rs2_pcl_grabber::Open() {
	rs2::config cfg;
	cfg.enable_device(m_id);
	cfg.enable_stream(RS2_STREAM_DEPTH);
	m_pipe.start();
}

void rs2_pcl_grabber::Close() {
	m_pipe.stop();
}

pcl_grabber::pcl_ptr rs2_pcl_grabber::Capture() {
	rs2::frameset frames = m_pipe.wait_for_frames();
	auto depth_frame = frames.get_depth_frame();
	rs2::pointcloud rsPoints;
	auto cloud_raw = points_to_pcl(rsPoints.calculate(m_thres_filter.process(depth_frame)));
	pcl_ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloud_raw, *cloud_transformed, m_affine);
	return cloud_transformed;
}

pcl_grabber::pcl_ptr rs2_pcl_grabber::points_to_pcl(const rs2::points& points) {
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