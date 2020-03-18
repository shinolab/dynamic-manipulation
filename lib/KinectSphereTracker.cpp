#include "KinectSphereTracker.hpp"
#include "KinectUtility.hpp"
#include "geometryUtil.hpp"
#include "Kinect.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/shape.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <vector>
#include <array>

#include <Windows.h>
#pragma comment (lib, "winmm")
//write.

namespace dynaman {

#pragma region DepthSphereTracker
	KinectDepthSphereTracker::KinectDepthSphereTracker(Eigen::Vector3f const &pos,
		Eigen::Quaternionf const &quo,
		bool useROI,
		Eigen::Vector3f bias)
		:_depthManager(),
		_coordManager(),
		_depthBuffer(_depthManager.numPixels()),
		_pos(pos),
		_quo(quo),
		_useROI(useROI),
		_bias(bias) {}

	void KinectDepthSphereTracker::maskWorkspace(Eigen::Vector3f const &lowerbound, Eigen::Vector3f const &upperbound, cv::Mat &mask) {
		std::vector<cv::Point2i> cornerPixels;
		std::vector<Eigen::Vector3f> corners;
		corners = cornersWorkspaceAll(lowerbound, upperbound);
		for (auto itr_corner = corners.begin(); itr_corner != corners.end(); itr_corner++)
		{
			Eigen::Vector3f corner_kinect = _quo.inverse() * (*itr_corner - _pos);
			CameraSpacePoint cspCorner{ corner_kinect.x() / 1000.f, corner_kinect.y() / 1000.f, corner_kinect.z() / 1000.f };
			DepthSpacePoint dspCorner;
			_coordManager.projectPositionToDepthPixel(cspCorner, dspCorner);
			cornerPixels.push_back(cv::Point2i(static_cast<int>(dspCorner.X), static_cast<int>(dspCorner.Y)));
		}
		std::vector<cv::Point> hull;
		cv::convexHull(cornerPixels, hull);
		mask = cv::Scalar::all(0);
		cv::fillConvexPoly(mask, hull, cv::Scalar::all(255));
	}

	Eigen::Vector3f KinectDepthSphereTracker::pos_sensor() {
		return _pos;
	}

	Eigen::Quaternionf KinectDepthSphereTracker::rot_sensor() {
		return _quo;
	}

	bool KinectDepthSphereTracker::observe(DWORD &time, Eigen::Vector3f &pos, FloatingObjectPtr objPtr) {
		bool isValid = false;
		HRESULT hr = _depthManager.acquireBuffer(_depthBuffer);
		DWORD observationTime = timeGetTime();
		if (SUCCEEDED(hr))
		{
			cv::Mat depthImageRaw = cv::Mat(_depthManager.height(), _depthManager.width(), CV_16UC1, &_depthBuffer[0]).clone();

			cv::Mat depthImageUc8;
			depthImageRaw.convertTo(depthImageUc8, CV_8UC1, 255.0 / (float)_depthManager.maxReliableDistance(), 0);
			//cv::imshow("Raw", depthImageUc8);

			cv::Mat maskedImage;
			cv::Mat mask = cv::Mat::zeros(_depthManager.height(), _depthManager.width(), CV_8UC1);

			//=====truncate region around the object=====
			if (objPtr->IsTracked() && _useROI)
			{
				Eigen::Vector3f pos = _quo.inverse() * (objPtr->getPosition() - _pos);
				cv::Point p(pos.x() * 365.6 / pos.z() + 0.5 * _depthManager.width()
					, -pos.y() * 367.2 / pos.z() + 0.5 * _depthManager.height()); //get pixel corresponding to the latest position of the object
				cv::circle(mask, p, 1.5f *objPtr->Radius() * 365.6f / pos.z(), cv::Scalar(255), -1, 8);
			}
			else
			{
				maskWorkspace(objPtr->lowerbound(), objPtr->upperbound(), mask);
			}
			depthImageUc8.copyTo(maskedImage, mask);
			//cv::imshow("ROI-masked", maskedImage); cv::waitKey(1);
			auto corners = cornersWorkspaceAll(objPtr->lowerbound(), objPtr->upperbound());
			auto rangesWorkspace = range2Points(_pos, _quo, corners);
			auto minRange = *std::min_element(rangesWorkspace.begin(), rangesWorkspace.end());
			auto maxRange = *std::max_element(rangesWorkspace.begin(), rangesWorkspace.end());
			cv::inRange(maskedImage, cv::Scalar(255 * minRange / _depthManager.maxReliableDistance()), cv::Scalar(255 * maxRange / _depthManager.maxReliableDistance()), maskedImage);
			cv::morphologyEx(maskedImage, maskedImage, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);

			//detect position of the object
			cv::Moments mu = cv::moments(maskedImage, true);
			cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));

			if (_depthManager.isInsideView(center.x, center.y))
			{
				CameraSpacePoint detectPos = _coordManager.getPositionAtDepthPixel(center.x, center.y, _depthBuffer);
				if (_depthManager.isReliable(detectPos))
				{
					float detectR = 1000.f * sqrt(detectPos.X * detectPos.X + detectPos.Y * detectPos.Y + detectPos.Z * detectPos.Z);
					float outpor = (detectR + objPtr->Radius()) / detectR;
					time = observationTime;
					pos = _quo * (1000.f * outpor * Eigen::Vector3f(detectPos.X, detectPos.Y, detectPos.Z) + _bias) + _pos;
					isValid = true;
				}
			}
		}
		return isValid;
	}
#pragma endregion

#pragma region ColorSphereTracker
	KinectColorSphereTracker::KinectColorSphereTracker(Eigen::Vector3f const &pos,
		Eigen::Quaternionf const &quo,
		cv::Scalar const &lowerbound,
		cv::Scalar const &upperbound,
		ColorSpace colorspace = ColorSpace::RGB)
		:_depthManager(),
		_colorManager(),
		_coordManager(),
		_depthBuffer(_depthManager.makeBuffer()),
		_colorBuffer(_colorManager.makeBuffer()),
		_pos(pos),
		_quo(quo),
		_lowerbound(lowerbound),
		_upperbound(upperbound),
		_colorSpace(colorspace){}

	Eigen::Vector3f KinectColorSphereTracker::pos_sensor() {
		return _pos;
	}

	Eigen::Quaternionf KinectColorSphereTracker::rot_sensor() {
		return _quo;
	}

	bool KinectColorSphereTracker::observe(DWORD &time, Eigen::Vector3f &pos, FloatingObjectPtr objPtr) {
		bool isValid = false;
		auto hr_depth = _depthManager.acquireBuffer(_depthBuffer);
		auto hr_color = _colorManager.acquireBuffer(_colorBuffer);
		if (SUCCEEDED(hr_depth) && SUCCEEDED(hr_color)) {
			cv::Mat img_raw(_depthManager.height(), _depthManager.width(), CV_8UC4, &_colorBuffer[0]);
			cv::Mat img_color;
			cv::cvtColor(img_raw, img_color, cv::COLOR_BGRA2BGR);
			if (_colorSpace == ColorSpace::HSV) {
				cv::Mat temp;
				cv::cvtColor(img_color, temp, cv::COLOR_BGR2HSV);
				img_color = temp;
			}
			cv::Mat img_bin;
			cv::inRange(img_color, img_bin, _lowerbound, _upperbound);
			auto mu = cv::moments(img_bin, true);
			cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));
			if (_colorManager.isInsideView(center.x, center.y)) {
				CameraSpacePoint detectPos = _coordManager.getPositionAtColorPixel(center.x, center.y, _depthBuffer);
				if (_depthManager.isReliable(detectPos)) {
					float detectR = 1000 * sqrt(detectPos.X * detectPos.X + detectPos.Y * detectPos.Y + detectPos.Z * detectPos.Z);
					float outpor = (detectR + objPtr->Radius()) / detectR;
					pos << _quo * (1000 * outpor * Eigen::Vector3f(detectPos.X, detectPos.Y, detectPos.Z)) + _pos;
					isValid = true;
				}
			}
		}
		return isValid;
	}

#pragma endregion

}

