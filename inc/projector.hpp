#include <opencv2/core.hpp>
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
	projector(std::string _name, const int _posX, const int _posY, const int _width, const int _height);

	void CreateFullScreen();

	void projectImageOnObject(const char* windowName, Eigen::Vector3f posKinect, cv::Mat image, cv::Size sizeReal);

	cv::Mat GenerateLandoltImage(float acuity, float distanceviewer, float distanceProj, int direction = 0);

	Eigen::Affine3f affineKinect2Projector();

	void projectPoints(const std::vector<cv::Point3f> &objectPoints, std::vector<cv::Point2f> &imagePoints);
};