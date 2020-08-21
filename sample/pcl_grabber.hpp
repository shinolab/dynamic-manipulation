#ifndef _DYNAMAN_PCL_GRABBER_HPP
#define _DYNAMAN_PCL_GRABBER_HPP

#include <memory>
#include <string>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>

class pcl_grabber {
public:
	using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
	virtual ~pcl_grabber() {};
	virtual void Open() = 0;
	virtual void Close() = 0;
	virtual pcl_ptr Capture() = 0;
};

class rs2_pcl_grabber : public pcl_grabber {
public:
	rs2_pcl_grabber(const Eigen::Vector3f& pos, const Eigen::Matrix3f& rot, const std::string& id, float range_min = 0.15f, float range_max = 1.0f);
	~rs2_pcl_grabber();
	static std::shared_ptr<pcl_grabber> Create(const Eigen::Vector3f& pos, const Eigen::Matrix3f& rot, const std::string& id, float range_min = 0.15f, float range_max = 1.0f);
	void Open() override;
	void Close() override;
	pcl_ptr Capture() override;
private:
	pcl_ptr points_to_pcl(const rs2::points& points);
	rs2::pipeline m_pipe;
	std::string m_id;
	Eigen::Affine3f m_affine;
	rs2::threshold_filter m_thr_filter;
	rs2::decimation_filter m_dec_filter;
	rs2::spatial_filter m_spt_filter;
	rs2::temporal_filter m_tmp_filter;
	rs2::disparity_transform m_depth_to_disparity;
	rs2::disparity_transform m_disparity_to_depth;
};
#endif // !_DYNAMAN_PCL_GRABBER_HPP
