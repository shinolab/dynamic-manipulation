#ifndef _PROJECTOR_HPP_
#define _PROJECTOR_HPP_

#define SUBDISPLAY_WIDTH 3840
#define SUBDISPLAY_HEIGHT 2160
#define MAINDISPLAY_WIDTH 1920
#define MAINDISPLAY_HEIGHT 1080

#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <Windows.h>
#include <vector>
#include <string>
#include <mutex>
#define _USE_MATH_DEFINES
#include <math.h>

class projector {
private:
	cv::Mat internalParam;
	cv::Mat distCoeffs;
	cv::Mat rvec;
	cv::Mat tvec;

	std::mutex mtxImage;
	int posX, posY, width, height;
	std::thread th_projection;

public:
	std::string name;
	cv::Mat image;
	projector(std::string _projectorName 
		, const int _posX = GetSystemMetrics(SM_CXSCREEN) 
		, const int _posY = 0//window position Y (default value: 0)
		, const int _width = SUBDISPLAY_WIDTH //width of the projector screen [px]
		, const int _height = SUBDISPLAY_HEIGHT //height of the projector screen [px]
	);

	~projector();

	void CreateScreen();

	void RefleshScreen();

	Eigen::Affine3f affineReference2Projector();

	void projectImageOnObject(Eigen::Vector3f position, cv::Mat image, cv::Size2f sizeReal, cv::Scalar backGroundColor = cv::Scalar(255, 255, 255), float distanceOffset = 0.f);

	void projectImageOnObject(Eigen::Vector3f position, cv::Size2f sizeReal, cv::Scalar backgroundColor = cv::Scalar::all(255), float distanceOffset = 0.f);

	void projectPoints(const std::vector<cv::Point3f> &objectPoints, std::vector<cv::Point2f> &imagePoints);

	void setImage(cv::Mat mat);

	void Run();

	void Stop();
};

#endif _PROJECTOR_HPP_