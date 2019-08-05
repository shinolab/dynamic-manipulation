#include "autd3.hpp"
#include "KinectApp.hpp"
#include "odcs.hpp"
#include <algorithm>
#include <vector>
#include <deque>
#include <atlbase.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/shape.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <Windows.h>
#include <Eigen/Geometry>

#define _USE_MATH_DEFINES
#include <math.h>

int ods::Initialize()
{
	kinectApp.initialize();
	//set workspace
	SetWorkSpace(Eigen::Vector3f(-1000, -1000, 800), Eigen::Vector3f(1000, 1000, 2000));
	// ========== Initialize Kinect ==========
	positionKinect = Eigen::Vector3f(41.7, -1006, 1313);
	/*
	dcmGlobal2Kinect = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX())
		*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

	*/
	dcmKinect2Global = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX())
		*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

	//affineKinect2Global = Eigen::Translation3f(positionKinect) * dcmKinect2Global;
	/*
	while (1)
	{
		HRESULT hr = updateBackgroundDepth();
		if (SUCCEEDED(hr))
		{
			break;
		}
		Sleep(10);
	}
	*/	
	return 0;
}

void ods::SetSensorGeometry(Eigen::Vector3f const &position, Eigen::Vector3f const &eulerAngle) {
	positionKinect = position;
	dcmKinect2Global = Eigen::AngleAxisf(eulerAngle.x(), Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(eulerAngle.y(), Eigen::Vector3f::UnitY())
		*Eigen::AngleAxisf(eulerAngle.z(), Eigen::Vector3f::UnitZ());
	//dcmGlobal2Kinect = dcmKinect2Global.transpose();
	//affineKinect2Global = Eigen::Translation3f(positionKinect) * dcmKinect2Global;
}

void ods::SetSensorGeometry(Eigen::Vector3f const &position, Eigen::Matrix3f const &rotKinect2Global) {
	positionKinect = position;
	dcmKinect2Global = rotKinect2Global;
	//dcmGlobal2Kinect = dcmKinect2Global.transpose();
	//affineKinect2Global = Eigen::Translation3f(positionKinect) * dcmKinect2Global;
}

KinectApp* ods::kinect() {
	return &kinectApp;
}

void ods::SetWorkSpace(Eigen::Vector3f const &corner1, Eigen::Vector3f const &corner2) {
	workspace << corner1, corner2;
}

void ods::CornersWorkspaceAll(Matrix38f &corners) {
	int index[3];
	for (int i = 0; i < 8; i++) {
		int res = i;
		for (int j = 0; j < 8; j++) {
			index[j] = res % 2;
			res /= 2;
		}
		corners.col(i) << Eigen::Vector3f(workspace(0, index[0]), workspace(1, index[1]), workspace(2, index[2]));
	}
}

float ods::RangeWorkspace() {
	Matrix38f corners;
	CornersWorkspaceAll(corners);
	return ((corners - positionKinect.replicate(1, corners.cols())).transpose()*(DcmKinect2Global() * Eigen::Vector3f::UnitZ())).maxCoeff();
}

float ods::RangeWorkspaceMin() {
	Matrix38f corners;
	CornersWorkspaceAll(corners);
	return ((corners - positionKinect.replicate(1, corners.cols())).transpose()*(DcmKinect2Global() * Eigen::Vector3f::UnitZ())).minCoeff();
}

void ods::MaskWorkspace(cv::Mat &mask) {
	std::vector<cv::Point2i> cornerPixels;
	Matrix38f corners;
	CornersWorkspaceAll(corners);
	for (int i = 0; i < corners.cols(); i++)
	{
		CameraSpacePoint cspCorner;
		cspCorner.X = (AffineGlobal2Kinect() * corners.col(i)).x() / 1000.f;
		cspCorner.Y = (AffineGlobal2Kinect() * corners.col(i)).y() / 1000.f;
		cspCorner.Z = (AffineGlobal2Kinect() * corners.col(i)).z() / 1000.f;
		DepthSpacePoint dspCorner;
		kinectApp.coordinateMapper->MapCameraPointToDepthSpace(cspCorner, &dspCorner);
		cornerPixels.push_back(cv::Point2i(static_cast<int>(dspCorner.X), static_cast<int>(dspCorner.Y)));
	}
	std::vector<cv::Point> hull;
	cv::convexHull(cornerPixels, hull);
	mask = cv::Scalar::all(0);
	cv::fillConvexPoly(mask, hull, cv::Scalar::all(255));
}

//==================== ODS Module ====================

bool ods::isInsideWorkSpace(const Eigen::Vector3f &pos)
{
	Eigen::Vector3f v0 = pos - workspace.col(0);
	Eigen::Vector3f v1 = pos - workspace.col(1);
	return (v0.x() * v1.x() <= 0) && (v0.y() * v1.y() <= 0) && (v0.z() * v1.z() <= 0);
}

HRESULT ods::updateBackgroundDepth()
{
	HRESULT hr = kinectApp.getDepthBuffer();
	if (SUCCEEDED(hr))
	{
		backgroundDepth.resize(kinectApp.depthBuffer.size());
		backgroundDepth = kinectApp.depthBuffer;
	}
	return hr;
}

//==================== Primitive Determination Functions ====================

void ods::DeterminePositionByHSV(FloatingObjectPtr objPtr, cv::Scalar lb, cv::Scalar ub)
{
	Eigen::Vector3f currentPosition;
	bool isValid = GetPositionByHSV(objPtr, currentPosition, lb, ub);
	if (isValid)
	{
		DWORD currentTime = timeGetTime();
		objPtr->updateStates(currentTime, currentPosition);
		objPtr->SetTrackingStatus(true);
	}
}

void ods::DeterminePositionByBGR(FloatingObjectPtr objPtr, cv::Scalar lb, cv::Scalar ub)
{
	Eigen::Vector3f currentPosition;
	bool isValid = GetPositionByBGR(objPtr, currentPosition, lb, ub);
	if (isValid)
	{
		if (isInsideWorkSpace(currentPosition))
		{
			DWORD currentTime = timeGetTime();
			objPtr->updateStates(currentTime, currentPosition);
			objPtr->SetTrackingStatus(true);
		}
		else
		{
			objPtr->SetTrackingStatus(false);
		}
	}
}

void ods::DeterminePositionByDepth(FloatingObjectPtr objPtr, bool useROI)
{
	Eigen::Vector3f currentPosition;
	bool isValid = GetPositionByDepth(objPtr, currentPosition, useROI);
	if (isValid)
	{
		if (isInsideWorkSpace(currentPosition))
		{
			DWORD currentTime = timeGetTime();
			objPtr->updateStates(currentTime, currentPosition);
			objPtr->SetTrackingStatus(true);
		}
	}
}

//===================== Observation Functions =====================

bool ods::GetPositionByBGR(FloatingObjectPtr objPtr, Eigen::Vector3f &pos, cv::Scalar lb, cv::Scalar ub)
{
	bool isValid = false;
	HRESULT hrColor = kinectApp.getColorBuffer(); 
	HRESULT hrDepth = kinectApp.getDepthBuffer();
	if (SUCCEEDED(hrColor) && SUCCEEDED(hrDepth))
	{
		cv::Mat colorImage = cv::Mat(kinectApp.getColorHeight(), kinectApp.getColorWidth(), CV_8UC4, &kinectApp.colorBuffer[0]).clone();
		cv::Mat bgrImage;
		cv::cvtColor(colorImage, bgrImage, CV_BGRA2BGR);
		cv::Mat binaryImage;
		cv::inRange(bgrImage, lb, ub, binaryImage);
		cv::Moments mu = cv::moments(binaryImage, true);
		cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));
		if (kinectApp.isInsideColorView(center.x, center.y))
		{
			CameraSpacePoint detectPosition = kinectApp.getPositionAtColorPixel(center.x, center.y);
			if (kinectApp.isReliablePosition(detectPosition))
			{
				float detectX = 1000 * detectPosition.X;
				float detectY = 1000 * detectPosition.Y;
				float detectZ = 1000 * detectPosition.Z;
				float detectR = sqrt(detectX * detectX + detectY * detectY + detectZ * detectZ);
				float outpor = (detectR + objPtr->radius) / detectR;
				pos << AffineKinect2Global() * Eigen::Vector3f(outpor * detectX, outpor * detectY, outpor * detectZ);
				isValid = true;
			}
		}
	}
	return isValid;
}

bool ods::GetPositionByHSV(FloatingObjectPtr objPtr, Eigen::Vector3f &pos, cv::Scalar lb, cv::Scalar ub)
{
	bool isValid = false;
	HRESULT hrColor = kinectApp.getColorBuffer();
	HRESULT hrDepth = kinectApp.getDepthBuffer();
	if (SUCCEEDED(hrColor) && SUCCEEDED(hrDepth))
	{
		//pre-processing
		cv::Mat colorImage = cv::Mat(kinectApp.getColorHeight(), kinectApp.getColorWidth(), CV_8UC4, &kinectApp.colorBuffer[0]).clone();
		cv::Mat bgrImage;
		cv::cvtColor(colorImage, bgrImage, CV_BGRA2BGR);
		cv::Mat hsvImage;
		cv::cvtColor(bgrImage, hsvImage, CV_BGR2HSV);
		cv::Mat binaryImage;
		cv::inRange(hsvImage, lb, ub, binaryImage);
		cv::Moments mu = cv::moments(binaryImage, true);
		cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));
		if (kinectApp.isInsideColorView(center.x, center.y))
		{
			CameraSpacePoint detectPosition = kinectApp.getPositionAtColorPixel(center.x, center.y);
			if (kinectApp.isReliablePosition(detectPosition))
			{
				float detectX = 1000 * detectPosition.X;
				float detectY = 1000 * detectPosition.Y;
				float detectZ = 1000 * detectPosition.Z;
				float detectR = sqrt(detectX * detectX + detectY * detectY + detectZ * detectZ);
				float outpor = (detectR + objPtr->radius) / detectR;
				pos << AffineKinect2Global() * Eigen::Vector3f(outpor * detectX, outpor * detectY, outpor * detectZ);
				isValid = true;
			}
		}
	}
	return isValid;
}

bool ods::GetPositionByDepth(FloatingObjectPtr objPtr, Eigen::Vector3f &pos, bool useROI)
{
	bool isValid = false;
	HRESULT hr = kinectApp.getDepthBuffer();
	if (SUCCEEDED(hr))
	{
		cv::Mat depthImageRaw = cv::Mat(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_16UC1, &kinectApp.depthBuffer[0]).clone();
		
		cv::Mat depthImageUc8;
		depthImageRaw.convertTo(depthImageUc8, CV_8UC1, 255.0 / (float)kinectApp.depthMaxReliableDistance, 0);
		//cv::imshow("Raw", depthImageUc8);

		//Background Subtraction
		if (!backgroundDepth.empty())
		{
			cv::Mat subtracted;
			cv::Mat imgBackground = cv::Mat(depthImageRaw.size(), CV_16UC1, &backgroundDepth[0]);
			cv::Mat invalidPixels; cv::compare(imgBackground, cv::Mat::zeros(imgBackground.size(), imgBackground.type()), invalidPixels, cv::CMP_EQ);
			cv::Mat maskBackground;
			cv::inRange(imgBackground - depthImageRaw, cv::Scalar(10), cv::Scalar(kinectApp.depthMaxReliableDistance), maskBackground);
			depthImageRaw.copyTo(subtracted, maskBackground + invalidPixels);
			subtracted.convertTo(depthImageUc8, CV_8UC1, 255.0 / (float)kinectApp.depthMaxReliableDistance);
			//cv::imshow("Subtracted", depthImageUc8);
		}
		cv::Mat maskedImage;
		cv::Mat mask = cv::Mat::zeros(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_8UC1);
		;
		//=====truncate region around the object=====
		if (objPtr->IsTracked() && useROI)
		{
			Eigen::Vector3f pos = AffineGlobal2Kinect() * (objPtr->getPosition());
			cv::Point p(pos.x() * 365.6 / pos.z() + 0.5 * kinectApp.getDepthWidth()
				, -pos.y() * 367.2 / pos.z() + 0.5 * kinectApp.getDepthHeight()); //get pixel corresponding to the latest position of the object
			cv::circle(mask, p, 1.5f *objPtr->Radius() * 365.6 / pos.z(), cv::Scalar(255), -1, 8);
		}
		else
		{
			MaskWorkspace(mask);
			//cv::rectangle(mask, cv::Point(0.05 * kinectApp.getDepthWidth(), 0.05f * kinectApp.getDepthHeight()), cv::Point(0.95 * kinectApp.getDepthWidth(), 0.7f * kinectApp.getDepthHeight()), cv::Scalar(255), -1, 8);
		}
		depthImageUc8.copyTo(maskedImage, mask);
		//cv::imshow("ROI-masked", maskedImage);
		cv::inRange(maskedImage, cv::Scalar(255 * RangeWorkspaceMin()/kinectApp.depthMaxReliableDistance), cv::Scalar(255 * RangeWorkspace() / kinectApp.depthMaxReliableDistance), maskedImage);
		cv::morphologyEx(maskedImage, maskedImage, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);

		
		//detect position of the object
		cv::Moments mu = cv::moments(maskedImage, true);
		cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));

		if (kinectApp.isInsideDepthView(center.x, center.y))
		{
			CameraSpacePoint detectPosition = kinectApp.getPositionAtDepthPixel(center.x, center.y);
			DWORD currentTime = timeGetTime();
			if (kinectApp.isReliablePosition(detectPosition))
			{
				float detectX = detectPosition.X * 1000.f;
				float detectY = detectPosition.Y * 1000.f;
				float detectZ = detectPosition.Z * 1000.f;
				float detectR = sqrt(detectX * detectX + detectY * detectY + detectZ * detectZ);
				float outpor = (detectR + objPtr->radius) / detectR;
				pos << outpor * detectX, outpor * detectY, outpor * detectZ;
				pos = AffineKinect2Global() * pos;
				isValid = true;
			}
		}
	}
	return isValid;
}

bool ods::findSphere(const cv::Mat depthMap, cv::Point &center, float &radius)
{
	cv::Mat mask(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_8UC1, cv::Scalar::all(0));
	cv::rectangle(mask, cv::Point(0.05f * kinectApp.getDepthWidth(), 0.f * kinectApp.getDepthHeight()), cv::Point(0.95f * kinectApp.getDepthWidth(), 1.0f * kinectApp.getDepthHeight()), cv::Scalar(255), -1, 8);
	cv::Mat maskDepth; cv::inRange(depthMap, cv::Scalar(5), cv::Scalar(102), maskDepth);
	cv::bitwise_and(mask, maskDepth, mask);
	depthMap.copyTo(depthMap, mask);
	//cv::imshow("raw", depthMap);
	//cv::imshow("depthMask", maskDepth);
	cv::Mat depthMapDenoised;
	cv::morphologyEx(depthMap, depthMapDenoised, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);
	const float threshold1 = 100.f;
	const float threshold2 = 200.f;
	cv::Mat edges;
	cv::Canny(depthMapDenoised, edges, threshold1, threshold2);
	CameraIntrinsics depthIntrinsics;
	kinectApp.coordinateMapper->GetDepthCameraIntrinsics(&depthIntrinsics);
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
	if (!contours.empty())
	{
		cv::Point centerCandidate(-1, -1);
		float radiusCandidate = 0;
		float similarity = 10000000;
		for (auto itrCont = contours.begin(); itrCont != contours.end(); itrCont++)
		{
			cv::Point2f centerTemp;
			float radiusTemp;
			cv::minEnclosingCircle(*itrCont, centerTemp, radiusTemp);
			float distance = kinectApp.getDepthAtDepthPixel(centerTemp.x, centerTemp.y);
			float scale = distance / depthIntrinsics.FocalLengthX;
			float similarityTemp = (radiusTemp * scale - 130) * (radiusTemp * scale - 130) + abs(cv::contourArea(*itrCont) * scale * scale - M_PI * 130 * 130);
			
			if (similarityTemp < similarity)
			{
				similarity = similarityTemp;
				centerCandidate = centerTemp;
				radiusCandidate = radiusTemp;
			}
		}
		//std::cout << similarity << std::endl;

		if (similarity < 20000)
		{
			center = centerCandidate;
			radius = radiusCandidate;
			cv::circle(edges, centerCandidate, radiusCandidate, cv::Scalar::all(155));
			//std::cout << "similar" << std::endl;
			return true;
		}
		//cv::imshow("edge", edges);

	}
	return false;
}