#ifndef _KINECT_SPHERE_TRACKER_HPP
#define _KINECT_SPHERE_TRACKER_HPP

#include <atlbase.h>
#include <vector>
#include <opencv2/core.hpp>
#include "KinectUtility.hpp"
#include "odcs.hpp"

namespace dynaman {

	class KinectDepthSphereTracker : public Tracker {
	public:
		KinectDepthSphereTracker(Eigen::Vector3f const &pos,
			Eigen::Quaternionf const &quo,
			bool useROI = true,
			Eigen::Vector3f bias = Eigen::Vector3f::Zero());
		bool observe(DWORD &time, Eigen::Vector3f &pos, FloatingObjectPtr objPtr) override;
		Eigen::Quaternionf rot_sensor();
		Eigen::Vector3f pos_sensor();
	private:
		KinectUtility::KinectDepthManager _depthManager;
		KinectUtility::KinectCoordManager _coordManager;
		std::vector<UINT16> _depthBuffer;
		Eigen::Vector3f _pos;
		Eigen::Quaternionf _quo;
		Eigen::Vector3f _bias;
		bool _useROI;
		void maskWorkspace(
			Eigen::Vector3f const &lowerbound,
			Eigen::Vector3f const &upperbound,
			cv::Mat& mask);
	};

	class KinectColorSphereTracker : public Tracker {
	public:
		enum ColorSpace { RGB = 0, HSV = 1 };
		KinectColorSphereTracker(Eigen::Vector3f const &pos,
			Eigen::Quaternionf const &quo,
			const cv::Scalar &lowerbound,
			const cv::Scalar &upperbound,
			ColorSpace colorSpace);
		bool observe(DWORD &time, Eigen::Vector3f &pos, FloatingObjectPtr objPtr) override;
		Eigen::Vector3f pos_sensor();
		Eigen::Quaternionf rot_sensor();

	private:
		KinectUtility::KinectDepthManager _depthManager;
		KinectUtility::KinectColorManager _colorManager;
		KinectUtility::KinectCoordManager _coordManager;
		std::vector<UINT16> _depthBuffer;
		std::vector<BYTE> _colorBuffer;
		Eigen::Vector3f _pos;
		Eigen::Quaternionf _quo;
		cv::Scalar _lowerbound;
		cv::Scalar _upperbound;
		ColorSpace _colorSpace;
	};
}

#endif // !_KINECT_SPHERE_TRACKER_HPP

