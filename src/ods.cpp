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
	Eigen::Vector3f wsCorner1(-1000, -1000, 0);
	Eigen::Vector3f wsCorner2(1000, 1000, 2000);
	workspace << wsCorner1, wsCorner2;
	// ========== Initialize Kinect ==========
	kinectApp.initialize();
	positionKinect = Eigen::Vector3f(35, -962, 1350);
	dcmGlobal2Kinect = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX())
		*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	dcmKinect2Global = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX())
		*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	affineKinect2Global = Eigen::Translation3f(positionKinect) * dcmKinect2Global;
	return 0;
}

//==================== ODS Module ====================

bool ods::isInsideWorkSpace(Eigen::Vector3f pos)
{
	Eigen::Vector3f v0 = pos - workspace.col(0);
	Eigen::Vector3f v1 = pos - workspace.col(1);
	return (v0.x() * v1.x() <= 0) && (v0.y() * v1.y() <= 0) && (v0.z() * v1.z() <= 0);
}

void ods::DeterminePositionByHSV(FloatingObjectPtr objPtr, cv::Scalar lb, cv::Scalar ub)
{
	kinectApp.getColorBuffer(); kinectApp.getDepthBuffer();
	//pre-processing
	cv::Mat colorImage = cv::Mat(kinectApp.getColorHeight(), kinectApp.getColorWidth(), CV_8UC4, &kinectApp.colorBuffer[0]);
	cv::Mat hsvImage;
	cv::cvtColor(colorImage, colorImage, CV_BGRA2BGR);
	cv::cvtColor(colorImage, hsvImage, CV_BGR2HSV);
	cv::Mat binaryImage;
	cv::inRange(hsvImage, lb, ub, binaryImage);
	cv::Moments mu = cv::moments(binaryImage, true);
	cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));
	cv::circle(colorImage, center, 10, cv::Scalar(50, 255, 255), 2, 8);
	cv::Mat display;
	cv::resize(colorImage, display, cv::Size(), 0.5, 0.5);
	cv::imshow("DISPLAY", display);

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
			Eigen::Vector3f currentPosition(outpor * detectX, outpor * detectY, outpor * detectZ);
			currentPosition = dcmKinect2Global * currentPosition + positionKinect;
			DWORD currentTime = timeGetTime();
			objPtr->updateStates(currentTime, currentPosition);
		}
	}
}

void ods::DeterminePositionByBGR(FloatingObjectPtr objPtr, cv::Scalar lb, cv::Scalar ub)
{
	kinectApp.getColorBuffer(); kinectApp.getDepthBuffer();
	cv::Mat colorImage = cv::Mat(kinectApp.getColorHeight(), kinectApp.getColorWidth(), CV_8UC4, &kinectApp.colorBuffer[0]);
	cv::Mat rgbImage;
	cv::cvtColor(colorImage, rgbImage, CV_BGRA2BGR);
	cv::Mat binaryImage;
	cv::inRange(rgbImage, lb, ub, binaryImage);
	cv::Moments mu = cv::moments(binaryImage, true);
	cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));
	cv::Mat display;
	cv::resize(binaryImage, display, cv::Size(), 0.5, 0.5);
	cv::imshow("DISPLAY", display);
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
			Eigen::Vector3f currentPosition; currentPosition << outpor * detectX, outpor * detectY, outpor * detectZ;
			currentPosition = dcmKinect2Global * currentPosition + positionKinect;
			DWORD currentTime = timeGetTime();
			objPtr->updateStates(currentTime, currentPosition);
		}
		else
		{
			objPtr->isTracked = false;
		}
	}
	else
	{
		objPtr->isTracked = false;
	}
}

void ods::DeterminePositionByDepth(FloatingObjectPtr objPtr)
{
	kinectApp.getDepthBuffer();
	//=====Preprocessing=====
	cv::Mat depthImage = cv::Mat(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_16UC1, &kinectApp.depthBuffer[0]);
	cv::Mat objectImage;
	depthImage.convertTo(objectImage, CV_8UC1, 255.0 / (float)kinectApp.depthMaxReliableDistance, 0);
	cv::Rect roi(
		cv::Point(0.05 * kinectApp.getDepthWidth(), 0 * kinectApp.getDepthHeight()),
		cv::Point(0.95 * kinectApp.getDepthWidth(), 1.0 * kinectApp.getDepthHeight()));
	cv::Mat maskedImage = objectImage(roi); //Masking
	cv::inRange(maskedImage, cv::Scalar(1), cv::Scalar(105), maskedImage);

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
			Eigen::Vector3f currentPosition; currentPosition << outpor * detectX, outpor * detectY, outpor * detectZ;
			currentPosition = dcmKinect2Global * currentPosition + positionKinect;
			if (isInsideWorkSpace(currentPosition))
			{
				objPtr->updateStates(currentTime, currentPosition);
				objPtr->isTracked = true;
			}
			else
			{
				objPtr->isTracked = false;
			}
		}
		else
		{
			objPtr->isTracked = false;
		}
	}
	else
	{
		objPtr->isTracked = false;
	}
	cv::imshow("POSITION", maskedImage);
}

void ods::DeterminePositionByDepthWithROI(FloatingObjectPtr objPtr)
{
	HRESULT hr = kinectApp.getDepthBuffer();
	if (SUCCEEDED(hr))
	{
		//=====Preprocessing=====
		cv::Mat depthImageRaw = cv::Mat(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_16UC1, &kinectApp.depthBuffer[0]);
		cv::Mat depthImageUc8;
		depthImageRaw.convertTo(depthImageUc8, CV_8UC1, 255.0 / (float)kinectApp.depthMaxReliableDistance, 0);
		cv::Mat maskedImage;
		cv::imshow("Raw", depthImageUc8); cv::waitKey(1);
		cv::Mat mask = cv::Mat::zeros(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_8UC1);
		if (objPtr->isTracked)
		{
			Eigen::Vector3f pos = affineKinect2Global.inverse() * (objPtr->getPosition());
			cv::Point p(pos.x() * 365.6 / pos.z() + 0.5 * kinectApp.getDepthWidth()
				, -pos.y() * 367.2 / pos.z() + 0.5 * kinectApp.getDepthHeight()); //get pixel corresponding to the latest position of the object
			cv::circle(mask, p, 150 * 365.6 / pos.z(), cv::Scalar(255), -1, 8);
		}
		else
		{
			cv::rectangle(mask, cv::Point(0.05 * kinectApp.getDepthWidth(), 0 * kinectApp.getDepthHeight()), cv::Point(0.95 * kinectApp.getDepthWidth(), 1.0 * kinectApp.getDepthHeight()), cv::Scalar(255), -1, 8);
		}
		depthImageUc8.copyTo(maskedImage, mask);
		//cv::imshow("ROI-masked", maskedImage);
		//cv::waitKey(1);
		cv::inRange(maskedImage, cv::Scalar(1), cv::Scalar(105), maskedImage);
		cv::imshow("ROI-masked", maskedImage);

		//detect position of the object
		cv::Moments mu = cv::moments(maskedImage, true);
		cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));

		if (kinectApp.isInsideDepthView(center.x, center.y))
		{
			std::cout << center.x << ", " << center.y << std::endl;
			CameraSpacePoint detectPosition;
			int capcount = 0;
			for (int ix = -3; ix <= 3; ix++)
			{
				for (int iy = -3; iy <= 3; iy++)
				{
					CameraSpacePoint postemp = kinectApp.getPositionAtDepthPixel(center.x, center.y);
					if (kinectApp.isReliablePosition(postemp))
					{
						detectPosition.X += postemp.X;
						detectPosition.Y += postemp.Y;
						detectPosition.Z += postemp.Z;
						capcount++;
					}
				}
			}
			std::cout << "capcount :" << capcount << " ";
			detectPosition.X /= capcount;
			detectPosition.Y /= capcount;
			detectPosition.Z /= capcount;
			//CameraSpacePoint detectPosition = kinectApp.getPositionAtDepthPixel(center.x, center.y);
			DWORD currentTime = timeGetTime();
			if (kinectApp.isReliablePosition(detectPosition))
			{
				float detectX = detectPosition.X * 1000;
				float detectY = detectPosition.Y * 1000;
				float detectZ = detectPosition.Z * 1000;
				float detectR = sqrt(detectX * detectX + detectY * detectY + detectZ * detectZ);
				float outpor = (detectR + objPtr->radius) / detectR;
				Eigen::Vector3f currentPosition(outpor * detectX, outpor * detectY, outpor * detectZ);
				currentPosition = affineKinect2Global * currentPosition;
				if (isInsideWorkSpace(currentPosition))
				{
					objPtr->updateStates(currentTime, currentPosition);
					objPtr->isTracked = true;
				}
				else
				{
					//objPtr->isTracked = false;
				}
			}
			else
			{
				//objPtr->isTracked = false;
			}
		}
		else
		{
			objPtr->isTracked = false;
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
					(*itrOP)->isTracked = true;
				}
				else
				{
					(*itrOP)->isTracked = false;
				}
			}			
		}

	}

}

void ods::GetMarkerPosition()
{
	while (1)
	{
		HRESULT hr1 = kinectApp.getInfraredBuffer();
		HRESULT hr2 = kinectApp.getDepthBuffer();
		if (SUCCEEDED(hr1))
		{
			if (SUCCEEDED(hr2))
			{
				break;
			}
		}
	}
	//cut high intensity noise
	cv::Mat infraredImage = cv::Mat(kinectApp.getInfraredHeight(), kinectApp.getInfraredWidth(), CV_16UC1, &kinectApp.infraredBuffer[0]);
	cv::Mat mask = cv::Mat::zeros(kinectApp.getInfraredHeight(), kinectApp.getInfraredWidth(), CV_16UC1);
	cv::rectangle(mask, cv::Point(100, 100), cv::Point(412, 324), cv::Scalar(1), -1);
	//infraredImage.copyTo(infraredImage, mask);
	infraredImage = infraredImage.mul(mask);
	cv::imshow("Infrared", infraredImage);
	cv::Mat binaryImage = cv::Mat(kinectApp.getInfraredHeight(), kinectApp.getInfraredWidth(), CV_8UC1);
	cv::inRange(infraredImage, cv::Scalar(0), cv::Scalar(600), binaryImage);
	binaryImage = ~binaryImage;
	cv::imshow("Binary", binaryImage);
	//find boundaries
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(binaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	int markerCount = 0;
	std::vector<Eigen::Vector3f> markerPositionsK;

	for (auto contour = contours.begin(); contour != contours.end(); contour++)
	{
		cv::Mat markerImage = cv::Mat::zeros(kinectApp.getInfraredHeight(), kinectApp.getInfraredWidth(), CV_8UC1);
		//cv::polylines(markerMaskImage, *contour, true, cv::Scalar(255), 1);
		cv::fillConvexPoly(markerImage, *contour, cv::Scalar(255));

		double markerArea = cv::contourArea(*contour, false);
		if (markerArea > 5)
		{
			cv::imshow("marker", markerImage);

			//get marker position
			cv::Moments mu = cv::moments(markerImage, true);
			cv::Point center = cv::Point((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));
			CameraSpacePoint cameraMarkerPoint = kinectApp.getPositionAtDepthPixel(center.x, center.y);
			std::cout << "Marker Area : " << markerArea << ", position : [" << cameraMarkerPoint.X * 1000 << ", " << cameraMarkerPoint.Y * 1000 << ", " << cameraMarkerPoint.Z * 1000 << "]" << std::endl;
			Eigen::Vector3f markerPosition; markerPosition << cameraMarkerPoint.X, cameraMarkerPoint.Y, cameraMarkerPoint.Z;
			markerPositionsK.insert(markerPositionsK.end(), markerPosition);
			markerCount++;
		}
	}
	if (markerCount != 3)
	{
		std::cout << "ERROR : THE NUMBER OF MARKERS is INVALID." << std::endl;
		return;
	}

	Eigen::Vector3f reference12K = (markerPositionsK[2] - markerPositionsK[1]).normalized();
	Eigen::Vector3f reference01K = (markerPositionsK[1] - markerPositionsK[0]).normalized();
	Eigen::Vector3f referenceCK = reference12K.cross(reference01K).normalized();
	Eigen::Vector3f marker0G(410, 185, 400);
	Eigen::Vector3f marker1G(410, 185, 870);
	Eigen::Vector3f marker2G(-410, 185, 870);
	Eigen::Vector3f reference12G = (marker2G - marker1G).normalized();
	Eigen::Vector3f reference01G = (marker1G - marker0G).normalized();
	Eigen::Vector3f referenceCG = reference12G.cross(reference01G).normalized();

	Eigen::Matrix3f refVecsG; refVecsG.col(0) << referenceCG; refVecsG.col(1) << reference12G; refVecsG.col(2) << reference01G;
	Eigen::Matrix3f refVecsK; refVecsK.col(0) << referenceCK; refVecsK.col(1) << reference12K; refVecsK.col(2) << reference01K;
	dcmKinect2Global = refVecsG * refVecsK.inverse();

	std::cout << "refVecsG : \n" << refVecsG << std::endl;
	std::cout << "refVecsK : \n" << refVecsK << std::endl;
	std::cout << "rotationMatrix : \n" << dcmKinect2Global << std::endl;
	positionKinect = -1000 * dcmKinect2Global * markerPositionsK[1] + marker1G;
}
