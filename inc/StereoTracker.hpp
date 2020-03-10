#ifndef _STEREO_TRACKER_HPP
#define _STEREO_TRACKER_HPP
#include "odcs.hpp"
#include "CameraDevice.hpp"
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <memory>

namespace dynaman {

	class stereoTracker : public dynaman::PositionSensor{
	public:
		stereoTracker(std::shared_ptr<CameraDevice> leftCameraPtr,
			std::shared_ptr<CameraDevice> rightCameraPtr,
			Eigen::Vector3f &pos,
			Eigen::Quaternionf &quo,
			cv::Mat &img_target);
		bool observe(DWORD& time, Eigen::Vector3f& pos, FloatingObjectPtr objPtr) override;
		void open();
		void close();
		cv::Point3f triangulate(const cv::Point2f point_left, const cv::Point2f point_right);
		void imgRightRect(cv::Mat& img);
		void imgLeftRect(cv::Mat& img);
	private:
		std::shared_ptr<CameraDevice> _leftCameraPtr;
		std::shared_ptr<CameraDevice> _rightCameraPtr;
		cv::Mat _proj_left;
		cv::Mat _proj_right;
		cv::Mat _mapx_left, _mapy_left, _mapx_right, _mapy_right;
		Eigen::Vector3f _pos;
		Eigen::Quaternionf _quo;
		cv::Mat _hist;
	};
}
#endif // !_STEREO_TRACKER_HPP
