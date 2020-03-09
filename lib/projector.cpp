#include "projector.hpp"
#include <Windows.h>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <iostream>
#include <mutex>

projector::projector(std::string _projectorName
	, const int _posX
	, const int _posY
	, const int _width
	, const int _height)
{
	internalParam = (cv::Mat_<float>(3, 3) <<
		6597.971555474551, 0, 2283.760276477737,
		0, 6573.034807690517, 1995.609409793493,
		0, 0, 1);
	distCoeffs = (cv::Mat_<float>(1, 5) <<
		-0.3009079226411982, 0.2283976980586602, -0.02787851156236279, 0.01835231549774181, 0);
	//kinect cooradinate system is temporally used as a reference of the external parameters. 
	rvec = (cv::Mat_<float>(3, 1) << 0.1147274683148607, 0.01720162016008192, -3.133892123979911);
	tvec = (cv::Mat_<float>(3, 1) << 33.37667871965536, 13.01671902885369, 992.7582019726681);
	this->posX = _posX;
	this->posY = _posY;
	this->width = _width;
	this->height = _height;
	this->name = _projectorName;
	//CreateScreen();
}

projector::~projector()
{
	cv::destroyWindow(name);
}

void projector::RefleshScreen()
{
	cv::destroyWindow(name);
	CreateScreen();
}

void projector::CreateScreen()
{
	cv::Mat blank(height, width, CV_8UC1, cv::Scalar(255));
	cv::namedWindow(name, CV_WINDOW_NORMAL);

	HWND windowHandle = ::FindWindowA(NULL, name.c_str());

	if (windowHandle != NULL)
	{
		SetWindowLongPtrA(windowHandle, GWL_STYLE, WS_POPUP);
		SetWindowLongPtrA(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);
		ShowWindow(windowHandle, SW_MAXIMIZE);
		cv::setWindowProperty(name, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		SetWindowPos(windowHandle, NULL, posX, posY, width, height, SWP_FRAMECHANGED | SWP_NOZORDER);
	}
	cv::imshow(name, blank);
}

Eigen::Affine3f projector::affineReference2Projector()
{
	Eigen::Vector3f rodrigues;
	Eigen::Vector3f translation;
	cv::cv2eigen(rvec, rodrigues);
	cv::cv2eigen(tvec, translation);
	return Eigen::Translation3f(translation)*Eigen::AngleAxisf(rodrigues.norm(), rodrigues.normalized());
}

//size should be specified in [mm]
void projector::projectImageOnObject(Eigen::Vector3f posRef, cv::Mat image, cv::Size2f sizeReal, cv::Scalar backGroundColor, float distanceOffset)
{
	float distance = (affineReference2Projector()*posRef).z() + distanceOffset;
	cv::Point3f objectPosition(posRef.x(), posRef.y(), posRef.z());
	std::vector<cv::Point3f> imagePoints3d = { objectPosition };
	//here comes conversion from object position to object points
	std::vector<cv::Point2f> imagePoints2d;
	cv::projectPoints(imagePoints3d, rvec, tvec, internalParam, distCoeffs, imagePoints2d);
	cv::Mat dst(height, width, CV_8UC3, backGroundColor);
	float fx = internalParam.at<float>(0, 0);
	float fy = internalParam.at<float>(1, 1);
	cv::circle(dst, imagePoints2d[0], 30 * fx / distance, cv::Scalar::all(255), -1);
	cv::Rect roi(cv::Point(0 * image.cols, 0), cv::Point(1.0 * image.cols, 1.0 * image.rows));
	cv::Mat affine = (cv::Mat_<float>(2, 3) <<
		fx * sizeReal.width / distance / image.cols, 0, ((int)(imagePoints2d[0].x - sizeReal.width * fx / distance / 2) - 30),
		0, fy * sizeReal.height / distance / image.rows, ((int)(imagePoints2d[0].y - sizeReal.height * fy / distance / 2)));
	cv::warpAffine(image(roi), dst, affine, dst.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	cv::imshow(name, dst);
}

void projector::projectImageOnObject(std::vector<Eigen::Vector3f> positions,
	std::vector<cv::Mat> images,
	std::vector<cv::Size2f> sizes,
	cv::Scalar backgroundColor,
	float distanceOffset) {
	std::vector<cv::Point3f> imagePoints3d;
	for (auto itr = positions.begin(); itr != positions.end(); itr++) {
		imagePoints3d.push_back(cv::Point3f((*itr).x(), (*itr).y(), (*itr).z()));
	}
	std::vector<cv::Point2f> imagePoints2d;
	projectPoints(imagePoints3d, imagePoints2d);
	//here comes conversion from object position to object points

	cv::Mat dst(height, width, CV_8UC3, backgroundColor);
	float fx = internalParam.at<float>(0, 0);
	float fy = internalParam.at<float>(1, 1);

	for (int i = 0; i < positions.size(); i++) {
		float distance = (affineReference2Projector()*positions[i]).z() + distanceOffset;
		cv::circle(dst, imagePoints2d[i], 0 * fx / distance, cv::Scalar::all(255), -1);
		cv::Rect roi(cv::Point(0 * images[i].cols, 0), cv::Point(1.0 * images[i].cols, 1.0 * images[i].rows));
		cv::Mat affine = (cv::Mat_<float>(2, 3) <<
			fx * sizes[i].width / distance / images[i].cols, 0, ((int)(imagePoints2d[i].x - sizes[i].width * fx / distance / 2)),
			0, fy * sizes[i].height / distance / images[i].rows, ((int)(imagePoints2d[i].y - sizes[i].height * fy / distance / 2)));
		cv::warpAffine(images[i](roi), dst, affine, dst.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

	}
	cv::imshow(name, dst);
}

void projector::projectImageOnObject(Eigen::Vector3f position, cv::Size2f sizeReal, cv::Scalar backgroundColor, float distanceOffset) {
	projectImageOnObject(position, this->image, sizeReal, backgroundColor, distanceOffset);
}


void projector::projectPoints(const std::vector<cv::Point3f> &objectPoints, std::vector<cv::Point2f> &imagePoints)
{
	cv::projectPoints(objectPoints, rvec, tvec, internalParam, distCoeffs, imagePoints);
}

void projector::setImage(cv::Mat image) {
	std::lock_guard<std::mutex> lock(mtxImage);
	this->image = image.clone();
}
