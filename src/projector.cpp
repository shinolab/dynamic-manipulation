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

projector::projector(std::string _projectorName
	, const int _posX
	, const int _posY
	, const int _width
	, const int _height)
{
	internalParam = (cv::Mat_<float>(3, 3) <<
		6532, 0, 1345,
		0, 6514, 2324,
		0, 0, 1);
	distCoeffs = (cv::Mat_<float>(1, 5) <<
		0.3016, -2.275, -0.00559, -0.0171, 0);
	//kinect cooradinate system is temporally used as a reference of the external parameters. 
	rvec = (cv::Mat_<float>(3, 1) << -1.274, -0.003, -2.867);
	tvec = (cv::Mat_<float>(3, 1) << -675.7, -261, -69.1);
	this->posX = _posX;
	this->posY = _posY;
	this->width = _width;
	this->height = _height;
	this->name = _projectorName;
	CreateScreen();
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
		fx * sizeReal.width / distance / image.cols, 0, ((int)(imagePoints2d[0].x - sizeReal.width * fx / distance / 2)),
		0, fy * sizeReal.height / distance / image.rows, ((int)(imagePoints2d[0].y - sizeReal.height * fy / distance / 2)));
	cv::warpAffine(image(roi), dst, affine, dst.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	cv::imshow(name, dst);
}

void projector::projectPoints(const std::vector<cv::Point3f> &objectPoints, std::vector<cv::Point2f> &imagePoints)
{
	cv::projectPoints(objectPoints, rvec, tvec, internalParam, distCoeffs, imagePoints);
}

