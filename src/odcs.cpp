#include "autd3.hpp"
#include "KinectApp.hpp"
#include "odcs.hpp"
#include <algorithm>
#include <vector>
#include <deque>
#include <atlbase.h>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <Windows.h>
#include <Eigen\Geometry>
#include <thread>
#include <dlib\matrix.h>
#include <dlib\optimization.h>
#include <nlopt.hpp>
#include "engine.h"
#include "arfModel.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

#define NUM_AUTDS 5
#define NUM_STATES 6
#define NUM_NODES_DCNLP 11
//#define NUM_TABLE_OFFSET 21
//#define NUM_TABLE_DISTANCE 8

#pragma comment(lib, "winmm.lib")


FloatingObject::FloatingObject(Eigen::Vector3f _positionTarget)
{
	position << 0, 0, 0;
	velocity << 0, 0, 0;
	integral << 0, 0, 0;
	positionTarget = _positionTarget;
	velocityTarget << 0, 0, 0;
	force_offset << 0, 0, 0;
	lastDeterminationTime = 0;
	isTracked = false;
	isControlled = true;
	velocityBuffer.resize(velocityBufferSize);
	dTBuffer.resize(velocityBufferSize);
	for (auto itr = velocityBuffer.begin(); itr != velocityBuffer.end(); itr++)
	{
		itr->setZero();
	}
	for (auto itr = dTBuffer.begin(); itr != dTBuffer.end(); itr++)
	{
		*itr = 1;
	}
	gravityForce << 0, 0, -0.05 * 9.8; //0.06 g
}

void FloatingObject::updateStates(DWORD determinationTime, Eigen::Vector3f positionNew)
{
	float dt = (float)(determinationTime - lastDeterminationTime) / 1000.0; // [sec]
	this->velocity = (positionNew - position) / dt;
	this->dTBuffer.push_back(dt);
	this->dTBuffer.pop_front();
	this->velocityBuffer.push_back(velocity);
	this->velocityBuffer.pop_front();
	if (this->isStable() && isTracked)
	{
		this->integral += (0.5 * (positionNew + position) - positionTarget) * dt;
	}
	position = positionNew;
	lastDeterminationTime = determinationTime;
}

void FloatingObject::updateStatesTarget(Eigen::Vector3f _positionTarget, Eigen::Vector3f _velocityTarget)
{
	positionTarget = _positionTarget;
	velocityTarget = _velocityTarget;
}

Eigen::Vector3f FloatingObject::averageVelocity()
{
	Eigen::Vector3f averageVelocity(0, 0, 0);
	auto itrVel = velocityBuffer.begin();
	auto itrDT = dTBuffer.begin();
	float period = 0;
	while (itrVel != velocityBuffer.end())
	{
		averageVelocity += (*itrDT) * (*itrVel);
		period += *itrDT;
		itrDT++;
		itrVel++;
	}
	averageVelocity /= period;
	return averageVelocity;
}

bool FloatingObject::isStable()
{
	return (averageVelocity().norm() < speedLimit);
}

int odcs::Initialize()
{
	autd.Open(autd::LinkType::ETHERCAT);
	if (!autd.isOpen()) return ENXIO;

	//set workspace
	Eigen::Vector3f wsCorner1(-1000, -1000, 0);
	Eigen::Vector3f wsCorner2(1000, 1000, 2000);
	workspace << wsCorner1, wsCorner2;
	/*
	Eigen::Vector3f positionAUTD0(415, 435, 0); Eigen::Vector3f eulerAngleAUTD0(0, 0, 0);
	Eigen::Vector3f positionAUTD1(-585, 435, 0); Eigen::Vector3f eulerAngleAUTD1(0, 0, 0);
	Eigen::Vector3f positionAUTD2(-585, -565, 0); Eigen::Vector3f eulerAngleAUTD2(0, 0, 0);
	Eigen::Vector3f positionAUTD3(415, -565, 0); Eigen::Vector3f eulerAngleAUTD3(0, 0, 0);
	*/
	Eigen::Vector3f positionAUTD0(-85, -65, 0); Eigen::Vector3f eulerAngleAUTD0(0, 0, 0);
	Eigen::Vector3f positionAUTD1(-1000, 65, 1000); Eigen::Vector3f eulerAngleAUTD1(M_PI, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD2(1000, -65, 1000); Eigen::Vector3f eulerAngleAUTD2(0, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD3(-65, -1000, 1000); Eigen::Vector3f eulerAngleAUTD3(-M_PI_2, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD4(65, 1000, 1000); Eigen::Vector3f eulerAngleAUTD4(M_PI_2, -M_PI_2, 0);

	autd.geometry()->AddDevice(positionAUTD0, eulerAngleAUTD0);
	autd.geometry()->AddDevice(positionAUTD1, eulerAngleAUTD1);
	autd.geometry()->AddDevice(positionAUTD2, eulerAngleAUTD2);
	autd.geometry()->AddDevice(positionAUTD3, eulerAngleAUTD3);
	autd.geometry()->AddDevice(positionAUTD4, eulerAngleAUTD4);

	positionAUTD.resize(3, NUM_AUTDS);//autd.geometry()->numDevices());
	positionAUTD << positionAUTD0, positionAUTD1, positionAUTD2, positionAUTD3, positionAUTD4;
	eulerAnglesAUTD.resize(3, NUM_AUTDS);//autd.geometry()->numDevices());
	eulerAnglesAUTD << eulerAngleAUTD0, eulerAngleAUTD1, eulerAngleAUTD2, eulerAngleAUTD3, eulerAngleAUTD4;
	centerAUTD.resize(3, NUM_AUTDS);//autd.geometry()->numDevices());
	directionsAUTD.resize(3, NUM_AUTDS);
	for (int i = 0; i < NUM_AUTDS; i++)
	{
		Eigen::Quaternionf quo =
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ());
		Eigen::Affine3f affineG2AUTD = Eigen::Translation3f(positionAUTD.col(i)) * quo;
		centerAUTD.col(i) << affineG2AUTD * Eigen::Vector3f(85, 65, 0);
		directionsAUTD.col(i) << quo * Eigen::Vector3f::UnitZ();
	}
	// ========== Initialize Kinect ==========
	kinectApp.initialize();
	positionKinect = Eigen::Vector3f(35, -962, 1350);
	dcmGlobal2Kinect = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX())
		*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	dcmKinect2Global = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())
		*Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX())
		*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	/*
	if (!(ep = engOpen(""))) {
	fprintf(stderr, "\nCan't start MATLAB engine\n");
	return EXIT_FAILURE;
	}
	*/
	//engEvalString(ep, "cd 'C:\\Users\\takuro\\Documents\\MATLAB\\AcouticAviator'");
	return 0;
}

void odcs::Close()
{
	autd.Close();
	//engClose(ep);
}

//==================== ODS Module ====================

bool odcs::isInsideWorkSpace(Eigen::Vector3f pos)
{
	Eigen::Vector3f v0 = pos - workspace.col(0);
	Eigen::Vector3f v1 = pos - workspace.col(1);
	return (v0.x() * v1.x() <= 0) && (v0.y() * v1.y() <= 0) && (v0.z() * v1.z() <= 0);
}

void odcs::DeterminePositionByHSV(FloatingObject* obj)
{
	kinectApp.getColorBuffer(); kinectApp.getDepthBuffer();
	//pre-processing
	cv::Mat colorImage = cv::Mat(kinectApp.getColorHeight(), kinectApp.getColorWidth(), CV_8UC4, &kinectApp.colorBuffer[0]);
	cv::Mat hsvImage;
	cv::cvtColor(colorImage, colorImage, CV_BGRA2BGR);
	cv::cvtColor(colorImage, hsvImage, CV_BGR2HSV);
	cv::Mat binaryImage;
	cv::inRange(hsvImage, cv::Scalar(150, 70, 70), cv::Scalar(210, 255, 255), binaryImage);
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
			float outpor = (detectR + obj->radius) / detectR;
			Eigen::Vector3f currentPosition(outpor * detectX, outpor * detectY, outpor * detectZ);
			currentPosition = dcmKinect2Global * currentPosition + positionKinect;
			DWORD currentTime = timeGetTime();
			obj->updateStates(currentTime, currentPosition);
		}
	}
}

void odcs::DeterminePositionByRGB(FloatingObject* obj)
{
	kinectApp.getColorBuffer(); kinectApp.getDepthBuffer();
	cv::Mat colorImage = cv::Mat(kinectApp.getColorHeight(), kinectApp.getColorWidth(), CV_8UC4, &kinectApp.colorBuffer[0]);
	cv::Mat rgbImage;
	cv::cvtColor(colorImage, rgbImage, CV_BGRA2BGR);
	cv::Mat binaryImage;
	cv::inRange(rgbImage, cv::Scalar(100, 0, 0), cv::Scalar(255, 50, 50), binaryImage);
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
			float outpor = (detectR + obj->radius) / detectR;
			Eigen::Vector3f currentPosition; currentPosition << outpor * detectX, outpor * detectY, outpor * detectZ;
			currentPosition = dcmKinect2Global * currentPosition + positionKinect;
			DWORD currentTime = timeGetTime();
			obj->updateStates(currentTime, currentPosition);
		}
		else
		{
			obj->isTracked = false;
		}
	}
	else
	{
		obj->isTracked = false;
	}
}

void odcs::DeterminePositionByDepth(FloatingObject* obj)
{
	kinectApp.getDepthBuffer();
	//=====Preprocessing=====
	cv::Mat depthImage = cv::Mat(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_16UC1, &kinectApp.depthBuffer[0]);
	cv::Mat objectImage;
	depthImage.convertTo(objectImage, CV_8UC1, 255.0 / (float)kinectApp.depthMaxReliableDistance, 0);
	cv::Mat maskedImage;
	cv::Mat depthMask = cv::Mat::zeros(kinectApp.getDepthHeight(), kinectApp.getDepthWidth(), CV_8UC1);
	cv::rectangle(depthMask, cv::Point(0.05 * kinectApp.getDepthWidth(), 0 * kinectApp.getDepthHeight()), cv::Point(0.95 * kinectApp.getDepthWidth(), 1.0 * kinectApp.getDepthHeight()), cv::Scalar(255), -1, 8);
	objectImage.copyTo(maskedImage, depthMask); //Masking
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
			float outpor = (detectR + obj->radius) / detectR;
			Eigen::Vector3f currentPosition; currentPosition << outpor * detectX, outpor * detectY, outpor * detectZ;
			currentPosition = dcmKinect2Global * currentPosition + positionKinect;
			if (isInsideWorkSpace(currentPosition))
			{
				obj->updateStates(currentTime, currentPosition);
				obj->isTracked = true;
			}
			else
			{
				obj->isTracked = false;
			}
		}
		else
		{
			obj->isTracked = false;
		}
	}
	else
	{
		obj->isTracked = false;
	}
	cv::imshow("POSITION", maskedImage);
}

void odcs::DeterminePositionByDepth(std::vector<FloatingObject> &objs)
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
	cv::morphologyEx(maskedImage, imgOpened, CV_MOP_OPEN, cv::Mat(cv::Size(3, 3), CV_8UC1), cv::Point(-1, -1), 2);
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(imgOpened.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	if (!contours.empty())
	{
		cv::Mat imgBound = cv::Mat::zeros(maskedImage.size(), CV_8UC1);
		std::vector<CameraSpacePoint> cameraPoints(contours.size());
		auto itrP = cameraPoints.begin();
		for (auto itrC = contours.begin(); itrC != contours.end(); itrC++, itrP++)
		{
			//cv::Rect bound = cv::boundingRect(*itrC);
			//cv::rectangle(imgRect, bound, cv::Scalar(255), CV_FILLED);
			cv::drawContours(imgBound, contours, std::distance(contours.begin(), itrC), cv::Scalar(255), CV_FILLED);
			cv::imshow("boundaries", imgBound);
			cv::Mat mask2 = cv::Mat::zeros(imgOpened.size(), CV_8UC1);
			//cv::rectangle(mask2, bound, cv::Scalar(255), CV_FILLED);
			cv::drawContours(mask2, contours, std::distance(contours.begin(), itrC), cv::Scalar(255), CV_FILLED);
			cv::Mat extract; imgOpened.copyTo(extract, mask2);
			//cv::imshow("extract", extract);
			cv::Moments m = cv::moments(extract, true);
			*itrP = kinectApp.getPositionAtDepthPixel(m.m10 / m.m00, m.m01 / m.m00);
		}
		DWORD currentTime = timeGetTime();
		/*
		for (auto itr2 = cameraPoints.begin(); itr2 != cameraPoints.end(); itr2++){
		Eigen::Vector3f posTemp(1000 * (*itr2).X, 1000 * (*itr2).Y, 1000 * (*itr2).Z);
		Eigen::Vector3f pTG = dcmKinect2Global * posTemp + positionKinect;
		std::cout << pTG.x() << ", " << pTG.y() << ", " << pTG.z() << " | ";
		}
		std::cout << std::endl;
		*/
		for (auto itrObj = objs.begin(); itrObj != objs.end(); itrObj++)
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
				float distTemp = ((*itrObj).position - dcmKinect2Global * posTemp - positionKinect).squaredNorm();
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
				float outpor = (detectR + (*itrObj).radius) / detectR;
				Eigen::Vector3f currentPosition; currentPosition << outpor * detectX, outpor * detectY, outpor * detectZ;
				currentPosition = dcmKinect2Global * currentPosition + positionKinect;
				if (isInsideWorkSpace(currentPosition))
				{
					(*itrObj).updateStates(currentTime, currentPosition);
					(*itrObj).isTracked = true;
				}
				else
				{
					(*itrObj).isTracked = false;
				}
			}
		}

	}
	
}

void odcs::GetMarkerPosition()
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

//==================== Find Duty Module ====================

void odcs::FindDutyBruteForce(Eigen::VectorXf *const duties, Eigen::MatrixXf *const directions, const Eigen::Vector3f pos, const Eigen::Vector3f force)
{
	const int numPatternDuty = 10;
	const int numPatternDirection = 5;
	const int numPatternTotal = pow(numPatternDuty * numPatternDirection, NUM_AUTDS);
	float radius = 100;
	Eigen::MatrixXf posRel = pos.replicate(1, NUM_AUTDS) - centerAUTD;
	Eigen::Vector3f forceTemp = Eigen::Vector3f::Zero();
	for (int iPattern = 0; iPattern < numPatternTotal; iPattern++)
	{
		if (iPattern % 10000 == 0)
		{
			std::cout << iPattern << "th pattern " << std::endl;
		}
		Eigen::VectorXf dutiesTemp = Eigen::VectorXf::Zero(duties->rows());
		Eigen::MatrixXf directionsTemp = Eigen::MatrixXf::Zero(directions->rows(), directions->cols());
		for (int iAUTD = 0; iAUTD < NUM_AUTDS; iAUTD++)
		{
			const int numPatternDuties = std::pow(numPatternDuty, NUM_AUTDS);
			const int pDuty = std::pow(numPatternDuty, iAUTD);
			dutiesTemp[iAUTD] = (1.0 / numPatternDuty) * (iPattern % numPatternDuties % pDuty);//0 ~ numPatternDuty
			const int pDirection = std::pow(numPatternDirection, iAUTD);
			float angle = (2 * M_PI / numPatternDirection) * (iPattern % numPatternDuties % pDirection); // 0 ~ numPatternDirection
			if (angle = 0)
			{
				directionsTemp.col(iAUTD) << pos.normalized();
			}
			else
			{
				//UNIT_X VECTOR of AUTD i
				Eigen::Quaternionf quo =
					Eigen::AngleAxisf(eulerAnglesAUTD.col(iAUTD).x(), Eigen::Vector3f::UnitZ()) *
					Eigen::AngleAxisf(eulerAnglesAUTD.col(iAUTD).y(), Eigen::Vector3f::UnitY()) *
					Eigen::AngleAxisf(eulerAnglesAUTD.col(iAUTD).z(), Eigen::Vector3f::UnitZ());
				Eigen::Vector3f unitXi = quo * Eigen::Vector3f::UnitX();
				//derive offset vector 
				directionsTemp.col(iAUTD) << pos + radius * unitXi.cross(pos).normalized();
			}
		}
		if ((force - arfModel::arfDirectionsTotal(posRel, dutiesTemp, directionsTemp)).norm() < (force - forceTemp).norm())
		{
			forceTemp = arfModel::arfDirectionsTotal(posRel, dutiesTemp, directionsTemp);
			*directions = directionsTemp;
			*duties = dutiesTemp;
		}
	}
}

int odcs::findDutyNLP(Eigen::VectorXf *duties, Eigen::MatrixXf *farpoints, Eigen::Vector3f pos, Eigen::Vector3f force)
{
	//initial Guess
	DWORD initialTime = timeGetTime();
	int numAUTDS = duties->size();
	Eigen::VectorXd duties0(numAUTDS); duties0.setZero();
	Eigen::MatrixXd offsets0(2, numAUTDS); offsets0.setZero();
	std::vector<double> x(3 * numAUTDS, 0);
	Eigen::Map<Eigen::VectorXd>(&x[0], numAUTDS) = duties0;
	Eigen::Map<Eigen::VectorXd>(&x[numAUTDS], numAUTDS) = offsets0.row(0);
	Eigen::Map<Eigen::VectorXd>(&x[2 * numAUTDS], numAUTDS) = offsets0.row(1);
	dataObj data;
	data.posRel = pos.replicate(1, numAUTDS) - centerAUTD;
	data.forceTarget = force;
	data.eulerAngleAUTDS = eulerAnglesAUTD;
	//Optimization Sequence
	nlopt::opt opt(nlopt::algorithm(nlopt::LN_COBYLA), x.size());
	opt.set_min_objective(SqDiffOfForce, static_cast<void*>(&data));
	std::vector<double> lb(x.size(), 0);
	for (auto itr = lb.begin(); itr != lb.end(); itr++)
	{
		if (itr - lb.begin() < numAUTDS){ *itr = 0; }
		else { *itr = -50; }
	}
	std::vector<double> ub(x.size(), 0);
	for (auto itr = ub.begin(); itr != ub.end(); itr++)
	{
		if (itr - ub.begin() < numAUTDS){ *itr = 1; }
		else { *itr = 50; }
	}
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);
	opt.set_maxeval(1000);
	opt.set_maxtime(0.15);
	opt.set_stopval(1e-3);
	opt.set_ftol_abs(1e-3);
	std::vector<double> grad;
	double minf;
	nlopt::result result = opt.optimize(x, minf);
	*duties = Eigen::Map<Eigen::VectorXd>(&x[0], numAUTDS).cast<float>();
	Eigen::MatrixXf offsetsL(3, numAUTDS);
	offsetsL.row(0) = Eigen::Map<Eigen::RowVectorXd>(&x[numAUTDS], numAUTDS).cast<float>();
	offsetsL.row(1) = Eigen::Map<Eigen::RowVectorXd>(&x[2 * numAUTDS], numAUTDS).cast<float>();
	offsetsL.row(2).setZero();
	Eigen::MatrixXf offsetsG(3, numAUTDS);
	for (int i = 0; i < numAUTDS; i++)
	{
		Eigen::Quaternionf quo =
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ());
		offsetsG.col(i) = quo.inverse() * offsetsL.col(i);
	}
	*farpoints = centerAUTD + 100 * (pos.replicate(1, numAUTDS) + offsetsG - centerAUTD);
	return 0;
}

//variable ; [u0 u1 ... uN xL0 xL1 ... xLN yL0 yL1 ... yLN] (offset should be represented in AUTD local coordinate)
double odcs::SqDiffOfForce(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	dataObj* d = reinterpret_cast<dataObj*>(data);
	Eigen::MatrixXf posRel = d->posRel;
	Eigen::Vector3f forceTarget = d->forceTarget;
	Eigen::MatrixXf eulerAnglesAUTD = d->eulerAngleAUTDS;
	std::vector<double> x_copy = x;
	Eigen::VectorXd duties = Eigen::Map<Eigen::VectorXd>(&x_copy[0], NUM_AUTDS);
	Eigen::MatrixXf offsetsL(3, NUM_AUTDS);
	offsetsL.row(0) = Eigen::Map<Eigen::RowVectorXd>(&x_copy[NUM_AUTDS], NUM_AUTDS).cast<float>();
	offsetsL.row(1) = Eigen::Map<Eigen::RowVectorXd>(&x_copy[2 * NUM_AUTDS], NUM_AUTDS).cast<float>();
	offsetsL.row(2).setZero();
	Eigen::MatrixXf offsetsG(3, NUM_AUTDS);
	for (int i = 0; i < NUM_AUTDS; i++)
	{
		Eigen::Quaternionf quo =
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ());
		offsetsG.col(i) = quo.inverse() * offsetsL.col(i);
	}
	Eigen::Vector3f F = arfModel::arfTotalOffsets(posRel, duties.cast<float>(), offsetsG);
	return (F - forceTarget).norm();
}

//==================== Local Control Module ====================

void odcs::PIDControl(FloatingObject* obj)
{
	Eigen::Vector3f dr = obj->position - obj->positionTarget;
	Eigen::Vector3f drdt = obj->velocity - obj->velocityTarget;
	Eigen::Vector3f forcePID = obj->mass * (gainP.asDiagonal() * dr + gainD.asDiagonal() * drdt + gainI.asDiagonal() * obj->integral) - obj->gravityForce;
	Eigen::VectorXf duties(NUM_AUTDS);
	Eigen::MatrixXf farpoints(3, NUM_AUTDS);
	findDutyNLP(&duties, &farpoints, obj->position, forcePID);
	//Eigen::VectorXi duties = (255 * FindDutyQP(forcePID + obj->force_offset, obj->position)).cast<int>().cwiseMax(0).cwiseMin(255);
	autd.AppendGainSync(autd::MultiFociGain::Create(farpoints, (255 * duties).cast<int>().cwiseMax(0).cwiseMin(255)));
	autd.AppendModulationSync(autd::Modulation::Create(255));
	Eigen::MatrixXf directions(3, NUM_AUTDS);
	directions = (farpoints - centerAUTD).colwise().normalized();
	Eigen::MatrixXf posRel = obj->position.replicate(1, NUM_AUTDS) - centerAUTD;

}

//==================== Global Control Module ====================

int odcs::SetPath(Eigen::Vector3f initialPosition, Eigen::Vector3f finalPosition, FloatingObject* obj)
{
	if (ep == NULL)
	{
		std::cout << "ERROR : ep==NULL" << std::endl;
		return EXIT_FAILURE;
	}
	double state0[NUM_STATES] = { initialPosition.x() / 1000, initialPosition.y() / 1000, initialPosition.z() / 1000, 0, 0, 0 };
	double stateF[NUM_STATES] = { finalPosition.x() / 1000, finalPosition.y() / 1000, finalPosition.z() / 1000, 0, 0, 0 };
	//2. state0 stateFの情報をmxArray化する
	mxArray *mxState0 = NULL, *mxStateF = NULL, *mxNumStates = NULL, *mxNumNodes = NULL, *mxPosAUTD = NULL;
	mxState0 = mxCreateDoubleMatrix(6, 1, mxREAL); memcpy((void*)mxGetPr(mxState0), (void*)state0, sizeof(state0));
	mxStateF = mxCreateDoubleMatrix(6, 1, mxREAL); memcpy((void*)mxGetPr(mxStateF), (void*)stateF, sizeof(stateF));
	mxNumStates = mxCreateDoubleScalar(NUM_STATES);
	mxNumNodes = mxCreateDoubleScalar(NUM_NODES_DCNLP);
	std::cout << "centerAUTD\n" << centerAUTD << std::endl;
	Eigen::MatrixXd cAd = centerAUTD.cast<double>() / 1000.0;
	mxPosAUTD = mxCreateDoubleMatrix(3, NUM_AUTDS, mxREAL); memcpy((void*)mxGetPr(mxPosAUTD), (void*)cAd.data(), sizeof(double) * 3 * NUM_AUTDS);
	//3. engPutVariableを利用してMATLABに情報を渡す
	engPutVariable(ep, "state0", mxState0);
	engPutVariable(ep, "stateF", mxStateF);
	engPutVariable(ep, "numNodes", mxNumNodes);
	engPutVariable(ep, "posAUTD", mxPosAUTD);
	//4. destroyMxArray
	mxDestroyArray(mxState0);
	mxDestroyArray(mxStateF);
	mxDestroyArray(mxNumNodes);
	mxDestroyArray(mxPosAUTD);
	//5. engEvalStringを利用してMATLAB上で経路生成
	engEvalString(ep, "env = setEnvironment(state0, stateF, posAUTD, numNodes);");
	//engEvalString(ep, "objfun = @(x) obj(x, env);");
	//engEvalString(ep, "varOpt = fmincon(objfun, env.var0, env.Ain, env.bin, env.Aeq, env.beq, env.lb, env.ub, @(x) nonlcon(x, env), env.options);");
	engEvalString(ep, "main2");
	engEvalString(ep, "stateOpt = extractState(varOpt, env);");
	engEvalString(ep, "controlOpt = extractControl(varOpt, env);");
	engEvalString(ep, "tFOpt = extractTerminalTime(varOpt, env);");
	engEvalString(ep, "tOpt = 0:tFOpt/(env.numNodes-1):tFOpt;");
	engEvalString(ep, "derivativeOpt = stateEq(stateOpt, controlOpt, env);");
	//=====経路の取得======
	//6. engGetVariableで生成した経路を取得する．
	mxArray *mxStatePath = NULL, *mxDerivativePath = NULL, *mxControlPath = NULL, *mxTimePath = NULL;
	mxStatePath = engGetVariable(ep, "stateOpt"); mxControlPath = engGetVariable(ep, "controlOpt");
	mxTimePath = engGetVariable(ep, "tOpt"); mxDerivativePath = engGetVariable(ep, "derivativeOpt");
	//7. result をEigen::Vectorに変換する．(result内のdataをMapする．)
	//arrayをstd::vectorXfでwrap
	obj->statePath = Eigen::MatrixXd::Map((double*)mxGetData(mxStatePath), NUM_STATES, NUM_NODES_DCNLP).cast<float>();
	obj->derivativePath = Eigen::MatrixXd::Map((double*)mxGetData(mxDerivativePath), NUM_STATES, NUM_NODES_DCNLP).cast<float>();
	obj->controlPath = Eigen::MatrixXd::Map((double*)mxGetData(mxControlPath), NUM_AUTDS, NUM_NODES_DCNLP).cast<float>();
	obj->timePath = Eigen::MatrixXd::Map((double*)mxGetData(mxTimePath), 1, NUM_NODES_DCNLP).cast<float>();
	//8. destroyMxArray
	obj->statePath *= 1000; obj->derivativePath *= 1000; //unit conversion (m -> mm)
	mxDestroyArray(mxStatePath); mxDestroyArray(mxDerivativePath); mxDestroyArray(mxControlPath); mxDestroyArray(mxTimePath);
	return EXIT_SUCCESS;
}

void odcs::PositionControlBySingleAUTD(FloatingObject* obj)
{
	float objRadius = 55;
	Eigen::Vector3f centerOfAUTD; centerOfAUTD << 90, 135, 0;
	DeterminePositionByDepth(obj);

	//corner of AUTD
	Eigen::Vector3f dr; dr << -obj->positionTarget[0] + obj->position[0], -obj->positionTarget[1] + obj->position[1], 0;
	Eigen::Vector3f drdt; drdt << obj->velocity[0], obj->velocity[1], 0;
	Eigen::Vector3f waveDirection; waveDirection = obj->position + obj->radius * (dr + 0.5 * drdt).normalized() - centerOfAUTD;

	int amplitude = round(std::max(std::min(105 - 0.1 * (obj->position[2] - obj->positionTarget[2]) - 0.1 * obj->velocity[2] - 0.001 * obj->integral[2], 255.0), 0.0));
	if (obj->isTracked == true)
	{
		autd.AppendGainSync(autd::FocalPointGain::Create(100 * waveDirection));
		autd.AppendModulationSync(autd::Modulation::Create((uint8_t)amplitude));
		Sleep(25);
	}
	else
	{
		autd.AppendGainSync(autd::NullGain::Create());
	}
}

void odcs::DirectSemiPlaneWave(FloatingObject* obj, Eigen::VectorXi amplitudes)
{
	float outpor = 100;
	Eigen::MatrixXf farPoints = centerAUTD + outpor * (obj->position.replicate(1, centerAUTD.cols()) - centerAUTD);
	autd.AppendGainSync(autd::MultiFociGain::Create(farPoints, amplitudes));
	autd.AppendModulation(autd::Modulation::Create(255));
	Sleep(25);
}

//update the target position of the object so that the object follow the path
void odcs::followPath(FloatingObject* obj)
{
	int latestNodeIndex = 0;
	int nextNodeIndex = 1;
	float tf = obj->timePath[obj->timePath.cols() - 1];
	float initialTime = (float)timeGetTime() / 1000.0;
	while (1)
	{
		float currentTime = (float)timeGetTime() / 1000.0 - initialTime;
		if (currentTime > obj->timePath[nextNodeIndex])
		{
			latestNodeIndex++;
			nextNodeIndex++;
			std::cout << "Passed : " << currentTime << std::endl;
			if (currentTime > tf)
			{
				break;
			}
		}
		else
		{
			//polynominal approximation of the path
			Eigen::VectorXf latestNodeState(NUM_STATES); latestNodeState = obj->statePath.col(latestNodeIndex);
			Eigen::VectorXf nextNodeState(NUM_STATES); nextNodeState = obj->statePath.col(nextNodeIndex);
			Eigen::VectorXf latestNodeDerivative(NUM_STATES); latestNodeDerivative = obj->derivativePath.col(latestNodeIndex);
			Eigen::VectorXf nextNodeDerivative(NUM_STATES); nextNodeDerivative = obj->derivativePath.col(nextNodeIndex);
			Eigen::VectorXf latestNodeControl(NUM_AUTDS); latestNodeControl = obj->controlPath.col(latestNodeIndex);
			Eigen::VectorXf nextNodeControl(NUM_AUTDS); nextNodeControl = obj->controlPath.col(nextNodeIndex);
			float dT = currentTime - obj->timePath[latestNodeIndex];
			float period = obj->timePath[nextNodeIndex] - obj->timePath[latestNodeIndex];
			float dS = dT / period;
			Eigen::Matrix<float, NUM_STATES, 4> adjNodeMat; adjNodeMat << nextNodeState, nextNodeDerivative*period, latestNodeState, latestNodeDerivative * period;
			Eigen::Matrix4f polyMat;
			polyMat <<
				-2, 3, 0, 0,
				1, -1, 0, 0,
				2, -3, 0, 1,
				1, -2, 1, 0;
			Eigen::Matrix<float, NUM_STATES, 4> polyCoefficients = adjNodeMat * polyMat;
			Eigen::Vector4f dSVec; dSVec << dS*dS*dS, dS*dS, dS, 1;
			Eigen::VectorXf stateTarget = polyCoefficients * dSVec;
			Eigen::VectorXf controlTarget = dS * nextNodeControl + (1 - dS) * latestNodeControl;
			obj->positionTarget << stateTarget[0], stateTarget[1], stateTarget[2];
			obj->velocityTarget << stateTarget[3], stateTarget[4], stateTarget[5];
		}
	}
}

//Legacy Module

Eigen::VectorXf odcs::findDutySI(FloatingObject* obj)
{
	Eigen::Vector3f dr = obj->position - obj->positionTarget;
	Eigen::VectorXf dutiesOffset(centerAUTD.cols()); dutiesOffset.setZero();
	Eigen::VectorXf gainPSi(centerAUTD.cols()); gainPSi.setConstant(-0.75); gainPSi[0] = -1.0;
	Eigen::VectorXf gainDSi(centerAUTD.cols()); gainDSi.setConstant(-1.3); gainDSi[0] = -1.8;
	Eigen::VectorXf gainISi(centerAUTD.cols()); gainISi.setConstant(-0.01);
	Eigen::VectorXf drRel = directionsAUTD.transpose() * dr;
	Eigen::VectorXf dvRel = directionsAUTD.transpose() * obj->velocity;
	Eigen::VectorXf diRel = directionsAUTD.transpose() * obj->integral;
	Eigen::VectorXf duties = dutiesOffset + gainPSi.asDiagonal() * drRel + gainDSi.asDiagonal() * dvRel;// +gainI.asDiagonal() * diRel;
	return duties;
}

Eigen::VectorXf odcs::findDutyQPEq(FloatingObject* obj)
{
	Eigen::Vector3f dr = obj->position - obj->positionTarget;
	Eigen::Vector3f dutiesOffset(0, 0, 0);
	Eigen::Vector3f gainPEq(-0.6, -0.6, -1.0);
	Eigen::Vector3f gainDEq(-1.0, -1.0, -1.8);
	Eigen::Vector3f gainIEq(0.0, 0.0, 0.0);
	Eigen::MatrixXf directions2obj = (obj->position.replicate(1, centerAUTD.cols()) - centerAUTD).colwise().normalized();
	Eigen::VectorXf f = dutiesOffset + gainPEq.asDiagonal() * dr + gainDEq.asDiagonal() * obj->averageVelocity() + gainIEq.asDiagonal() * obj->integral;
	Eigen::MatrixXf E = directions2obj.transpose() * directions2obj;
	Eigen::VectorXf b = -directions2obj.transpose() * f;
	dlib::matrix<float, NUM_AUTDS, NUM_AUTDS> Ed = dlib::mat(E);
	dlib::matrix<float, NUM_AUTDS, 1> bd = dlib::mat(b);
	dlib::matrix<float, NUM_AUTDS, 1> u = dlib::zeros_matrix<float>(centerAUTD.cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> upperbound = 255 * dlib::ones_matrix<float>(centerAUTD.cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> lowerbound = dlib::zeros_matrix<float>(centerAUTD.cols(), 1);
	dlib::solve_qp_box_constrained(Ed, bd, u, lowerbound, upperbound, (float)1e-5, 100);
	Eigen::VectorXf duty(centerAUTD.cols());
	for (int index = 0; index < centerAUTD.cols(); index++)
	{
		duty[index] = u(index, 0);
	}
	return duty;
}

Eigen::VectorXf odcs::FindDutyQP(Eigen::Vector3f force, Eigen::Vector3f position)
{
	Eigen::MatrixXf posRel = position.replicate(1, centerAUTD.cols()) - centerAUTD;
	Eigen::MatrixXf F = arfModel::arf(posRel);
	Eigen::MatrixXf Q = F.transpose() * F;
	Eigen::VectorXf b = -F.transpose() * force;
	Eigen::VectorXf duty(NUM_AUTDS);
	dlib::matrix<float, NUM_AUTDS, NUM_AUTDS> Qd = dlib::mat(Q);
	dlib::matrix<float, NUM_AUTDS, 1> bd = dlib::mat(b);
	dlib::matrix<float, NUM_AUTDS, 1> u = 0.5 * dlib::ones_matrix<float>(NUM_AUTDS, 1);
	dlib::matrix<float, NUM_AUTDS, 1> upperbound = dlib::ones_matrix<float>(centerAUTD.cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> lowerbound = dlib::zeros_matrix<float>(centerAUTD.cols(), 1);
	dlib::solve_qp_box_constrained(Qd, bd, u, lowerbound, upperbound, (float)1e-5, 100);
	for (int index = 0; index < NUM_AUTDS; index++)
	{
		duty[index] = u(index, 0);
	}
	return duty;
}

Eigen::VectorXf odcs::FindDutyQP(FloatingObject* obj)
{
	Eigen::Vector3f dr = obj->position - obj->positionTarget;
	Eigen::Vector3f dutiesOffset(0, 0, 0);
	Eigen::Vector3f gainPQp(-0.02, -0.02, -0.02);
	Eigen::Vector3f gainDQp(-0.036, -0.036, -0.036);
	Eigen::Vector3f gainIQp(-0.002, -0.002, -0.002);
	Eigen::Vector3f force = gainPQp.asDiagonal() * dr + gainDQp.asDiagonal() * obj->averageVelocity() + gainIQp.asDiagonal() * obj->integral - obj->gravityForce;
	Eigen::MatrixXf posRel = obj->position.replicate(1, centerAUTD.cols()) - centerAUTD;
	Eigen::MatrixXf F = arfModel::arf(posRel);
	Eigen::MatrixXf Q = F.transpose() * F;
	Eigen::VectorXf b = -F.transpose() * force;
	Eigen::VectorXf duty(NUM_AUTDS);
	dlib::matrix<float, NUM_AUTDS, NUM_AUTDS> Qd = dlib::mat(Q);
	dlib::matrix<float, NUM_AUTDS, 1> bd = dlib::mat(b);
	dlib::matrix<float, NUM_AUTDS, 1> u = dlib::zeros_matrix<float>(NUM_AUTDS, 1);
	dlib::matrix<float, NUM_AUTDS, 1> upperbound = dlib::ones_matrix<float>(centerAUTD.cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> lowerbound = dlib::zeros_matrix<float>(centerAUTD.cols(), 1);
	dlib::solve_qp_box_constrained(Qd, bd, u, lowerbound, upperbound, (float)1e-5, 100);
	for (int index = 0; index < NUM_AUTDS; index++)
	{
		duty[index] = u(index, 0);
	}
	return duty;
}

