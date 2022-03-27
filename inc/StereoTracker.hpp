#pragma once
#include "tracker.hpp"
#include "CameraDevice.hpp"
#include "ImgProcUtil.hpp"
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <memory>

namespace dynaman {
	class stereoCamera {
	public:
		stereoCamera(
			std::shared_ptr<CameraDevice> leftCameraPtr,
			std::shared_ptr<CameraDevice> rightCameraPtr
		);

		static std::shared_ptr<stereoCamera> create(
			std::shared_ptr<CameraDevice> leftCamPtr,
			std::shared_ptr<CameraDevice> rightCamPtr
		);

		void open();
		void close();
		cv::Point3f triangulate(const cv::Point2f point_left, const cv::Point2f point_right);
		void imgRightRect(cv::Mat& img);
		void imgLeftRect(cv::Mat& img);
		bool isOpen();
	private:
		std::shared_ptr<CameraDevice> _leftCameraPtr;
		std::shared_ptr<CameraDevice> _rightCameraPtr;
		cv::Mat _proj_left;
		cv::Mat _proj_right;
		cv::Mat _mapx_left, _mapy_left, _mapx_right, _mapy_right;
	};

	class stereoTracker : public dynaman::Tracker {
	public:
		stereoTracker(
			std::shared_ptr<stereoCamera> stereoCamPtr,
			std::shared_ptr<imgProc::extractor> extractorPtrLeft,
			std::shared_ptr<imgProc::extractor> extractorPtrRight,
			const Eigen::Vector3f& pos,
			const Eigen::Quaternionf& quo,
			Eigen::Vector3f bias = Eigen::Vector3f::Zero()
		);

		static std::shared_ptr<stereoTracker> create(
			std::shared_ptr<stereoCamera> stereoCamPtr,
			std::shared_ptr<imgProc::extractor> _extPtrLeft,
			std::shared_ptr<imgProc::extractor> _extPtrRight,
			const Eigen::Vector3f &pos,
			const Eigen::Quaternionf &quo,
			const Eigen::Vector3f& bias = Eigen::Vector3f::Zero()
		);

		bool observe(DWORD& time, Eigen::Vector3f& pos, FloatingObjectPtr objPtr) override;
		bool open() override;
		bool isOpen() override;
	private:
		std::shared_ptr<imgProc::extractor> _extPtrLeft;
		std::shared_ptr<imgProc::extractor> _extPtrRight;
		std::shared_ptr<stereoCamera> _stereoCamPtr;
		Eigen::Vector3f _pos;
		Eigen::Quaternionf _quo;
		Eigen::Vector3f _bias; //bias in sensor coordinate
	};

}
