#ifndef _PROJECTOR_HPP_
#define _PROJECTOR_HPP_

#define SUBDISPLAY_WIDTH 3840
#define SUBDISPLAY_HEIGHT 2160
#define MAINDISPLAY_WIDTH 1920
#define MAINDISPLAY_HEIGHT 1080

#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <Windows.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>

class projector {
private:
	cv::Mat internalParam;
	cv::Mat distCoeffs;
	cv::Mat rvec;
	cv::Mat tvec;

	int posX, posY, width, height;
	std::string name;

public:
	projector(std::string _projectorName 
		, const int _posX = GetSystemMetrics(SM_CXSCREEN) 
		, const int _posY = 0//window position Y (default value: 0)
		, const int _width = SUBDISPLAY_WIDTH //width of the projector screen [px]
		, const int _height = SUBDISPLAY_HEIGHT //height of the projector screen [px]
	);

	void CreateScreen();

	Eigen::Affine3f affineReference2Projector();

	void projectImageOnObject(Eigen::Vector3f position, cv::Mat image, cv::Size sizeReal, cv::Scalar backGroundColor = cv::Scalar(255, 255, 255));

	void projectPoints(const std::vector<cv::Point3f> &objectPoints, std::vector<cv::Point2f> &imagePoints);
};

#endif _PROJECTOR_HPP_