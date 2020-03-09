#ifndef _STEREO_TRACKER_HPP
#define _STEREO_TRACKER_HPP
#include "CameraDevice.hpp"
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <memory>

class stereoTracker {
public:
	stereoTracker(std::shared_ptr<CameraDevice> leftCameraPtr, std::shared_ptr<CameraDevice> rightCameraPtr);
	void open();
	void close();
	Eigen::Vector3f position();
	cv::Point3f triangulate(const cv::Point2f point_left, const cv::Point2f point_right);
	void imgRightRect(cv::Mat& img);
	void imgLeftRect(cv::Mat& img); 
private:
	static void openDefault(xiAPIplusCameraOcv& cam, std::string &cam_id);
	std::shared_ptr<CameraDevice> _leftCameraPtr;
	std::shared_ptr<CameraDevice> _rightCameraPtr;
	cv::Mat _proj_left;
	cv::Mat _proj_right;
	cv::Mat _mapx_left, _mapy_left, _mapx_right, _mapy_right;
};

#endif // !_STEREO_TRACKER_HPP
