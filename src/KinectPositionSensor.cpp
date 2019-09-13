#include "KinectPositionSensor.hpp"
#include "KinectApp.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/shape.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace dynaman {
	KinectDepthPositionSensor::KinectDepthPositionSensor(Eigen::Vector3f const &pos,
		Eigen::Quaternionf const &quo,
		Eigen::Vector3f const &corner0,
		Eigen::Vector3f const &corner1,
		bool useROI)
		:_corner0(corner0), _corner1(corner1), _useROI(useROI), _pos(pos), _quo(quo){
		kinect.initialize();
	}

	bool KinectDepthPositionSensor::isInsideWorkspace(Eigen::Vector3f const &pos) {
		Eigen::Vector3f v0 = pos - _corner0;
		Eigen::Vector3f v1 = pos - _corner1;
		return (v0.x() * v1.x() <= 0) && (v0.y() * v1.y() <= 0) && (v0.z() * v1.z() <= 0);
	}

	std::vector<Eigen::Vector3f> KinectDepthPositionSensor::cornersWorkspaceAll() {
		std::vector<Eigen::Vector3f> corners;
		int index[3];
		for (int i = 0; i < 8; i++) { // for the number of the corners
			int res = i;
			for (int j = 0; j < 3; j++) { // for x, y, z
				index[j] = res % 2;
				res /= 2;
			}
			float x = (index[0] == 0 ? _corner0.x() : _corner1.x());
			float y = (index[1] == 0 ? _corner0.y() : _corner1.y());
			float z = (index[2] == 0 ? _corner0.z() : _corner1.z());
			corners.push_back(Eigen::Vector3f(x, y, z));
		}
		return std::move(corners);
	}

	void KinectDepthPositionSensor::maskWorkspace(cv::Mat &mask) {
		std::vector<cv::Point2i> cornerPixels;
		std::vector<Eigen::Vector3f> corners;
		corners = cornersWorkspaceAll();
		for (auto itr_corner = corners.begin(); itr_corner != corners.end(); itr_corner++)
		{
			Eigen::Vector3f corner_kinect = _quo.inverse() * (*itr_corner - _pos);
			CameraSpacePoint cspCorner{ corner_kinect.x()/1000.f, corner_kinect.y()/1000.f, corner_kinect.z()/1000.f };
			DepthSpacePoint dspCorner;
			kinect.coordinateMapper->MapCameraPointToDepthSpace(cspCorner, &dspCorner);
			cornerPixels.push_back(cv::Point2i(static_cast<int>(dspCorner.X), static_cast<int>(dspCorner.Y)));
		}
		std::vector<cv::Point> hull;
		cv::convexHull(cornerPixels, hull);
		mask = cv::Scalar::all(0);
		cv::fillConvexPoly(mask, hull, cv::Scalar::all(255));
	}

	Eigen::Vector3f KinectDepthPositionSensor::position() {
		return _pos;
	}

	Eigen::Quaternionf KinectDepthPositionSensor::rotation() {
		return _quo;
	}

	float KinectDepthPositionSensor::rangeWorkspaceMax() {
		std::vector<Eigen::Vector3f> corners = cornersWorkspaceAll();
		float maxDist = 0;
		for (auto itr_corner = corners.begin(); itr_corner != corners.end(); itr_corner++) {
			float dist_corner = (*itr_corner - _pos).dot(_quo * Eigen::Vector3f::UnitZ());
			if (dist_corner > maxDist) {
				maxDist = dist_corner;
			}
		}
		return maxDist;
	}

	float KinectDepthPositionSensor::rangeWorkspaceMin() {
		std::vector<Eigen::Vector3f> corners = cornersWorkspaceAll();
		float minDist = kinect.depthMaxReliableDistance;
		for (auto itr_corner = corners.begin(); itr_corner != corners.end(); itr_corner++) {
			float dist_corner = (*itr_corner - _pos).dot(_quo * Eigen::Vector3f::UnitZ());
			if (dist_corner < minDist) {
				minDist = dist_corner;
			}
		}
		return minDist;
	}

	bool KinectDepthPositionSensor::updateStates(FloatingObjectPtr objPtr) {
		bool isValid = false;
		HRESULT hr = kinect.getDepthBuffer();
		DWORD observationTime = timeGetTime();
		if (SUCCEEDED(hr))
		{
			cv::Mat depthImageRaw = cv::Mat(kinect.getDepthHeight(), kinect.getDepthWidth(), CV_16UC1, &kinect.depthBuffer[0]).clone();

			cv::Mat depthImageUc8;
			depthImageRaw.convertTo(depthImageUc8, CV_8UC1, 255.0 / (float)kinect.depthMaxReliableDistance, 0);
			//cv::imshow("Raw", depthImageUc8);

			cv::Mat maskedImage;
			cv::Mat mask = cv::Mat::zeros(kinect.getDepthHeight(), kinect.getDepthWidth(), CV_8UC1);

			//=====truncate region around the object=====
			if (objPtr->IsTracked() && _useROI)
			{
				Eigen::Vector3f pos = _quo.inverse() * (objPtr->getPosition() - _pos);
				cv::Point p(pos.x() * 365.6 / pos.z() + 0.5 * kinect.getDepthWidth()
					, -pos.y() * 367.2 / pos.z() + 0.5 * kinect.getDepthHeight()); //get pixel corresponding to the latest position of the object
				cv::circle(mask, p, 1.5f *objPtr->Radius() * 365.6 / pos.z(), cv::Scalar(255), -1, 8);
			}
			else
			{
				maskWorkspace(mask);
				//cv::rectangle(mask, cv::Point(0.05 * kinectApp.getDepthWidth(), 0.05f * kinectApp.getDepthHeight()), cv::Point(0.95 * kinectApp.getDepthWidth(), 0.7f * kinectApp.getDepthHeight()), cv::Scalar(255), -1, 8);
			}
			depthImageUc8.copyTo(maskedImage, mask);
			//cv::imshow("ROI-masked", maskedImage);
			cv::inRange(maskedImage, cv::Scalar(255 * rangeWorkspaceMin() / kinect.depthMaxReliableDistance), cv::Scalar(255 * rangeWorkspaceMax() / kinect.depthMaxReliableDistance), maskedImage);
			cv::morphologyEx(maskedImage, maskedImage, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);

			//detect position of the object
			cv::Moments mu = cv::moments(maskedImage, true);
			cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));

			if (kinect.isInsideDepthView(center.x, center.y))
			{
				CameraSpacePoint detectPosition = kinect.getPositionAtDepthPixel(center.x, center.y);
				DWORD currentTime = timeGetTime();
				if (kinect.isReliablePosition(detectPosition))
				{
					float detectX = detectPosition.X * 1000.f;
					float detectY = detectPosition.Y * 1000.f;
					float detectZ = detectPosition.Z * 1000.f;
					float detectR = sqrt(detectX * detectX + detectY * detectY + detectZ * detectZ);
					float outpor = (detectR + objPtr->radius) / detectR;
					Eigen::Vector3f positionObserved = _quo * (outpor * Eigen::Vector3f(detectX, detectY, detectZ)) + _pos;
					objPtr->updateStates(observationTime, positionObserved);
					objPtr->SetTrackingStatus(true);
					isValid = true;
				}
			}
		}
		return isValid;
	}
}