#ifndef Projector_hpp_
#define Projector_hpp_

#include<Windows.h>
#include<Eigen\Geometry>
#include<opencv2\core.hpp>
#include<opencv2\highgui.hpp>

class Projector
{
private:
	int screenHeight;
	int screenWidth;
	int screenPositionX;
	int screenPositionY;
	const char* windowName;
	Eigen::MatrixXf projParam; //project 3D point coordinate on 2D image coordinate

public:
	Projector(const char* _windowName,
		int _screenWidth,
		int _screenHeight,
		int _screenPosX,
		int _screenPosY,
		Eigen::MatrixXf _projParam
		);
	
	~Projector();
	
	void displayImage(cv::Mat image);
};

#endif