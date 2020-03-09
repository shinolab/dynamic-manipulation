#include "StereoTracker.hpp"
#include "ImgProcUtil.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>
#include <iostream>

stereoTracker::stereoTracker(
	std::shared_ptr<CameraDevice> leftCameraPtr,
	std::shared_ptr<CameraDevice> rightCameraPtr) :
	_leftCameraPtr(leftCameraPtr), _rightCameraPtr(rightCameraPtr) {
	cv::FileStorage fs("cam_stereo.yml", cv::FileStorage::READ);
	if (!fs.isOpened()) {
		throw std::runtime_error("cannot open cam_stereo.yml");
	};
	cv::Mat intrinsic_left, intrinsic_right, dist_left, dist_right, R_left, R_right;
	cv::Size imageSize;
	fs["imageSize"] >> imageSize;
	fs["K1"] >> intrinsic_left;
	fs["K2"] >> intrinsic_right;
	fs["D1"] >> dist_left;
	fs["D2"] >> dist_right;
	fs["R1"] >> R_left;
	fs["R2"] >> R_right;
	fs["P1"] >> _proj_left;
	fs["P2"] >> _proj_right;
	rightCameraPtr->setIntrinsic(intrinsic_right);
	rightCameraPtr->setDistCoeff(dist_right);
	leftCameraPtr->setIntrinsic(intrinsic_left);
	leftCameraPtr->setDistCoeff(dist_left);
	cv::initUndistortRectifyMap(intrinsic_left, dist_left, R_left, _proj_left, imageSize, CV_32FC1, _mapx_left, _mapy_left);
	cv::initUndistortRectifyMap(intrinsic_right, dist_right, R_right, _proj_right, imageSize, CV_32FC1, _mapx_right, _mapy_right);
	fs.release();
}

void stereoTracker::open() {
	_leftCameraPtr->open();
	_rightCameraPtr->open();
}

void stereoTracker::close() {
	_leftCameraPtr->close();
	_rightCameraPtr->close();
}

void stereoTracker::imgLeftRect(cv::Mat &img) {
	cv::Mat img_raw;
	_leftCameraPtr->fetch_frame(img_raw);
	cv::remap(img_raw, img, _mapx_left, _mapy_left, cv::INTER_LINEAR);
}

void stereoTracker::imgRightRect(cv::Mat& img) {
	cv::Mat img_raw;
	_rightCameraPtr->fetch_frame(img_raw);
	cv::remap(img_raw, img, _mapx_right, _mapy_right, cv::INTER_LINEAR);
}


Eigen::Vector3f stereoTracker::position() {
	//acquire cm from both cameras;
	//rectify
	cv::Mat img_rect_left, img_rect_right;
	std::cout << "remapping ..." << std::endl;
	this->imgLeftRect(img_rect_left);
	this->imgRightRect(img_rect_right);
#ifdef _DEBUG
	cv::imshow("left (rectified)", img_rect_left);
	cv::imshow("right (rectified)", img_rect_right);
#endif
	cv::Point2f point_left, point_right;
	cv::Scalar lowerbound(10, 50, 150);
	cv::Scalar upperbound(30, 255, 255);
	imgProcUtil::get_center_thr(img_rect_left, lowerbound, upperbound, point_left, cv::COLOR_BGR2HSV);
	imgProcUtil::get_center_thr(img_rect_right, lowerbound, upperbound, point_right, cv::COLOR_BGR2HSV);
	cv::Mat cvPosHomo;
	cv::triangulatePoints(_proj_left, _proj_right, cv::Mat(point_left), cv::Mat(point_right), cvPosHomo);
	cv::Mat cvPos;
	std::cout << cvPosHomo << std::endl << std::endl;
	cv::convertPointsFromHomogeneous(cvPosHomo.reshape(4, 1), cvPos);
	Eigen::Vector3f pos;
	cv::cv2eigen(cvPos.reshape(1, 3), pos);
	return pos;
}

cv::Point3f stereoTracker::triangulate(const cv::Point2f point_left, const cv::Point2f point_right) {
	cv::Mat cvPosHomo;
	cv::triangulatePoints(_proj_left, _proj_right, cv::Mat(point_left), cv::Mat(point_right), cvPosHomo);
	cv::Mat cvPos;
	cv::convertPointsFromHomogeneous(cvPosHomo.reshape(4, 1), cvPos);
	return cv::Point3f(cvPos.reshape(1,3));
}