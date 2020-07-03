#include "StereoTracker.hpp"
#include "ImgProcUtil.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>
#include <iostream>
#include <Windows.h>

#pragma comment(lib, "winmm")

namespace dynaman {

	stereoCamera::stereoCamera(
		std::shared_ptr<CameraDevice> leftCameraPtr,
		std::shared_ptr<CameraDevice> rightCameraPtr):
		_leftCameraPtr(leftCameraPtr),
		_rightCameraPtr(rightCameraPtr) {
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

	void stereoCamera::open() {
		if (!_leftCameraPtr->isOpen()) {
			_leftCameraPtr->open();
		}
		if (!_rightCameraPtr->isOpen()) {
			_rightCameraPtr->open();
		}
	}

	void stereoCamera::close() {
		if (_leftCameraPtr->isOpen()) {
			_leftCameraPtr->close();
		}
		if (_rightCameraPtr->isOpen()) {
			_rightCameraPtr->close();
		}
	}

	void stereoCamera::imgLeftRect(cv::Mat& img) {
		cv::Mat img_raw;
		_leftCameraPtr->fetch_frame(img_raw);
		cv::remap(img_raw, img, _mapx_left, _mapy_left, cv::INTER_LINEAR);
	}

	void stereoCamera::imgRightRect(cv::Mat& img) {
		cv::Mat img_raw;
		_rightCameraPtr->fetch_frame(img_raw);
		cv::remap(img_raw, img, _mapx_right, _mapy_right, cv::INTER_LINEAR);
	}

	bool stereoCamera::isOpen() {
		return _rightCameraPtr->isOpen() && _leftCameraPtr->isOpen();
	}

	std::shared_ptr<stereoCamera> stereoCamera::create(
		std::shared_ptr<CameraDevice> leftCamPtr,
		std::shared_ptr<CameraDevice> rightCamPtr
	) {
		return std::make_shared<stereoCamera>(leftCamPtr, rightCamPtr);
	}

	cv::Point3f stereoCamera::triangulate(const cv::Point2f point_left, const cv::Point2f point_right) {
		cv::Mat cvPosHomo;
		cv::triangulatePoints(_proj_left, _proj_right, cv::Mat(point_left), cv::Mat(point_right), cvPosHomo);
		cv::Mat cvPos;
		cv::convertPointsFromHomogeneous(cvPosHomo.reshape(4, 1), cvPos);
		return cv::Point3f(cvPos.reshape(1, 3));
	}

	stereoTracker::stereoTracker(
		std::shared_ptr<stereoCamera> stereoCamPtr,
		std::shared_ptr<imgProc::extractor> extractorPtrLeft,
		std::shared_ptr<imgProc::extractor> extractorPtrRight,
		const Eigen::Vector3f &pos,
		const Eigen::Quaternionf &quo,
		Eigen::Vector3f bias) :
		_stereoCamPtr(stereoCamPtr),
		_extPtrLeft(extractorPtrLeft),
		_extPtrRight(extractorPtrRight),
		_pos(pos),
		_quo(quo),
		_bias(bias){}

	std::shared_ptr<stereoTracker> stereoTracker::create(
		std::shared_ptr<stereoCamera> stereoCamPtr,
		std::shared_ptr<imgProc::extractor> extractorPtrLeft,
		std::shared_ptr<imgProc::extractor> extractorPtrRight,
		const Eigen::Vector3f& pos,
		const Eigen::Quaternionf& quo,
		const Eigen::Vector3f& bias) {
		return std::make_shared<stereoTracker>(stereoCamPtr, extractorPtrLeft, extractorPtrRight, pos, quo, bias);
	}

	bool stereoTracker::observe(DWORD &time, Eigen::Vector3f &pos, FloatingObjectPtr objPtr) {
		//acquire cm from both cameras;
		//rectify
		cv::Mat img_left_rect, img_right_rect;
		time = timeGetTime();
		_stereoCamPtr->imgLeftRect(img_left_rect);
		_stereoCamPtr->imgRightRect(img_right_rect);

		cv::Point2f point_left = _extPtrLeft->extract_center(img_left_rect);
		cv::Point2f point_right = _extPtrRight->extract_center(img_right_rect);
		cv::circle(img_left_rect, point_left, 1, cv::Scalar(0, 0, 255), -1);
		cv::circle(img_right_rect, point_right, 1, cv::Scalar(0, 0, 255), -1);
		//cv::imshow("left view", img_left_rect);
		//cv::imshow("right view", img_right_rect);
		//cv::imshow("left (processed)", _extPtrLeft->img_debug());
		//cv::imshow("right (processed", _extPtrRight->img_debug());
		//cv::waitKey(3);
		cv::Point3f cvPos = _stereoCamPtr->triangulate(point_left, point_right);
		Eigen::Vector3f posObserved;
		cv::cv2eigen(1000*cv::Mat(cvPos).reshape(1, 3), posObserved);
		pos = _quo * (posObserved + _bias) + _pos;
		return true;
	}

	bool stereoTracker::open() {
		_stereoCamPtr->open();
		return _stereoCamPtr->isOpen();
	}

	bool stereoTracker::isOpen() {
		return _stereoCamPtr->isOpen();
	}
}
