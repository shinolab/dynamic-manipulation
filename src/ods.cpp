#include "autd3.hpp"
#include "KinectApp.hpp"
#include "odcs.hpp"
#include <algorithm>
#include <vector>
#include <deque>
#include <atlbase.h>
#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\shape.hpp>
#include <Windows.h>
#include <Eigen\Geometry>

#define _USE_MATH_DEFINES
#include <math.h>

int ods::Initialize()
{
	//set workspace
	Eigen::Vector3f wsCorner1(-1000, -1000, 800);
	Eigen::Vector3f wsCorner2(1000, 1000, 2000);
	workspace << wsCorner1, wsCorner2;
	// ========== Initialize Kinect ==========
	kinectApp.initialize();
	positionKinect = Eigen::Vector3f(41.7, -1006, 1313);

	dcmGlobal2Kinect = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX())
		*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	dcmKinect2Global = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX())
		*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

	affineKinect2Global = Eigen::Translation3f(positionKinect) * dcmKinect2Global;
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
	dcmGlobal2Kinect = Eigen::AngleAxisf(eulerAngle.x(), Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(eulerAngle.y(), Eigen::Vector3f::UnitY())
		*Eigen::AngleAxisf(eulerAngle.z(), Eigen::Vector3f::UnitZ());
	dcmKinect2Global = Eigen::AngleAxisf(eulerAngle.x(), Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(eulerAngle.y(), Eigen::Vector3f::UnitY())
		*Eigen::AngleAxisf(eulerAngle.z(), Eigen::Vector3f::UnitZ());
	affineKinect2Global = Eigen::Translation3f(positionKinect) * dcmKinect2Global;
}

void ods::SetWorkSpace(Eigen::Vector3f const &corner1, Eigen::Vector3f const &corner2) {
	workspace << corner1, corner2;
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

void ods::DeterminePositionByDepth(std::vector<FloatingObjectPtr> objPtrs)
{
	kinectApp.getDepthBuffer();
	//Masking
	cv::Mat depthImage = cv::Mat(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_16UC1, &kinectApp.depthBuffer[0]);
	cv::Mat objectImage;
	depthImage.convertTo(objectImage, CV_8UC1, 255.0 / (float)kinectApp.depthMaxReliableDistance, 0);
	cv::Mat maskedImage;
	cv::Mat depthMask = cv::Mat::zeros(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_8UC1);
	cv::rectangle(depthMask, cv::Point(0.05 * kinectApp.getDepthWidth(), 0 * kinectApp.getDepthHeight()), cv::Point(0.95 * kinectApp.getDepthWidth(), 1.0 * kinectApp.getDepthHeight()), cv::Scalar(255), -1, 8);
	objectImage.copyTo(maskedImage, depthMask); //Masking
	cv::inRange(maskedImage, cv::Scalar(1), cv::Scalar(105), maskedImage);
	cv::imshow("depth", maskedImage);

	//clip
	cv::Mat imgOpened;
	cv::Mat strElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::morphologyEx(maskedImage.clone(), imgOpened, CV_MOP_OPEN, strElement, cv::Point(-1, -1), 2, CV_HAL_BORDER_CONSTANT, cv::Scalar(0));
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(imgOpened.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	if (!contours.empty())
	{
		cv::Mat imgBound = cv::Mat::zeros(maskedImage.size(), CV_8UC1);
		std::vector<CameraSpacePoint> cameraPoints(contours.size());
		auto itrP = cameraPoints.begin();
		for (auto itrC = contours.begin(); itrC != contours.end(); itrC++, itrP++)
		{
			cv::drawContours(imgBound, contours, std::distance(contours.begin(), itrC), cv::Scalar(255), CV_FILLED);
			cv::Mat mask2 = cv::Mat::zeros(imgOpened.size(), CV_8UC1);
			cv::drawContours(mask2, contours, std::distance(contours.begin(), itrC), cv::Scalar(255), CV_FILLED);
			cv::Mat extract; imgOpened.copyTo(extract, mask2);
			cv::Moments m = cv::moments(extract, true);
			*itrP = kinectApp.getPositionAtDepthPixel(m.m10 / m.m00, m.m01 / m.m00);
		}
		DWORD currentTime = timeGetTime();
		for (auto itrOP = objPtrs.begin(); itrOP != objPtrs.end(); itrOP++)
		{
			if (cameraPoints.empty())
			{
				break;
			}
			std::vector<float> dists(cameraPoints.size());
			auto itrP = cameraPoints.begin();
			auto itrDist = dists.begin();
			for (auto itrP = cameraPoints.begin(); itrP != cameraPoints.end(); itrP++, itrDist++)
			{
				Eigen::Vector3f posTemp(1000 * (*itrP).X, 1000 * (*itrP).Y, 1000 * (*itrP).Z);
				float distTemp = ((*itrOP)->getPosition() - dcmKinect2Global * posTemp - positionKinect).squaredNorm();
				*itrDist = distTemp;
			}
			auto i = std::distance(dists.begin(), std::min_element(dists.begin(), dists.end()));
			CameraSpacePoint detectPosition = cameraPoints[i];
			cameraPoints.erase(cameraPoints.begin() + i);
			if (kinectApp.isReliablePosition(detectPosition))
			{
				float detectX = detectPosition.X * 1000;
				float detectY = detectPosition.Y * 1000;
				float detectZ = detectPosition.Z * 1000;
				float detectR = sqrt(detectX * detectX + detectY * detectY + detectZ * detectZ);
				float outpor = (detectR + (*itrOP)->radius) / detectR;
				Eigen::Vector3f currentPosition; currentPosition << outpor * detectX, outpor * detectY, outpor * detectZ;
				currentPosition = dcmKinect2Global * currentPosition + positionKinect;
				if (isInsideWorkSpace(currentPosition))
				{
					(*itrOP)->updateStates(currentTime, currentPosition);
					(*itrOP)->SetTrackingStatus(true);
				}
				else
				{
					(*itrOP)->SetTrackingStatus(false);
				}
			}			
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
				pos << affineKinect2Global * Eigen::Vector3f(outpor * detectX, outpor * detectY, outpor * detectZ);
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
				pos << affineKinect2Global * Eigen::Vector3f(outpor * detectX, outpor * detectY, outpor * detectZ);
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
		cv::imshow("Raw", depthImageUc8);

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
			cv::imshow("Subtracted", depthImageUc8);
		}
		cv::Mat maskedImage;
		cv::Mat mask = cv::Mat::zeros(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_8UC1);
		//=====truncate region around the object=====
		if (objPtr->IsTracked() && useROI)
		{
			Eigen::Vector3f pos = affineKinect2Global.inverse() * (objPtr->getPosition());
			cv::Point p(pos.x() * 365.6 / pos.z() + 0.5 * kinectApp.getDepthWidth()
				, -pos.y() * 367.2 / pos.z() + 0.5 * kinectApp.getDepthHeight()); //get pixel corresponding to the latest position of the object
			cv::circle(mask, p, 105 * 365.6 / pos.z(), cv::Scalar(255), -1, 8);
		}
		else
		{
			cv::rectangle(mask, cv::Point(0.05 * kinectApp.getDepthWidth(), 0 * kinectApp.getDepthHeight()), cv::Point(0.95 * kinectApp.getDepthWidth(), 1.0 * kinectApp.getDepthHeight()), cv::Scalar(255), -1, 8);		
		}
		depthImageUc8.copyTo(maskedImage, mask);
		cv::inRange(maskedImage, cv::Scalar(1), cv::Scalar(105), maskedImage);
		cv::morphologyEx(maskedImage, maskedImage, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);
		cv::imshow("ROI-masked", maskedImage);
		//cv::imshow("In-range", maskedImage);
		
		//detect position of the object
		cv::Moments mu = cv::moments(maskedImage, true);
		cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));

		if (kinectApp.isInsideDepthView(center.x, center.y))
		{
			CameraSpacePoint detectPosition = kinectApp.getPositionAtDepthPixel(center.x, center.y);
			DWORD currentTime = timeGetTime();
			if (kinectApp.isReliablePosition(detectPosition))
			{
				float detectX = detectPosition.X * 1000;
				float detectY = detectPosition.Y * 1000;
				float detectZ = detectPosition.Z * 1000;
				float detectR = sqrt(detectX * detectX + detectY * detectY + detectZ * detectZ);
				float outpor = (detectR + objPtr->radius) / detectR;
				pos << outpor * detectX, outpor * detectY, outpor * detectZ;
				pos = affineKinect2Global * pos;
				isValid = true;
			}
		}
	}
	return isValid;
}

bool ods::findSphere(const cv::Mat depthMap, cv::Point &center, float &radius)
{
	cv::Mat mask(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_8UC1, cv::Scalar::all(0));
	cv::rectangle(mask, cv::Point(0.05 * kinectApp.getDepthWidth(), 0 * kinectApp.getDepthHeight()), cv::Point(0.95 * kinectApp.getDepthWidth(), 1.0 * kinectApp.getDepthHeight()), cv::Scalar(255), -1, 8);
	cv::Mat maskDepth; cv::inRange(depthMap, cv::Scalar(5), cv::Scalar(102), maskDepth);
	cv::bitwise_and(mask, maskDepth, mask);
	depthMap.copyTo(depthMap, mask);
	//cv::imshow("raw", depthMap);
	//cv::imshow("depthMask", maskDepth);
	cv::Mat depthMapDenoised;
	cv::morphologyEx(depthMap, depthMapDenoised, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);
	const float threshold1 = 100;
	const float threshold2 = 200;
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
		cv::imshow("edge", edges);

	}
	return false;
}