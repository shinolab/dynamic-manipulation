#include "Projector.hpp"

Projector::Projector(
	const char* _windowName,
	int screenPosX,
	int screenPosY,
	int screenWidth,
	int screenHeight,
	Eigen::MatrixXf _projParam
) : windowName(_windowName)
{
	cv::namedWindow(windowName, CV_WINDOW_NORMAL);
	HWND windowHandle = ::FindWindowA(NULL, windowName);
	projParam = _projParam;

	if (windowHandle != NULL)
	{
		SetWindowLongPtr(windowHandle, GWL_STYLE, WS_POPUP);
		SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

		ShowWindow(windowHandle, SW_MAXIMIZE);
		cv::setWindowProperty(windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

		SetWindowPos(windowHandle, NULL,
			screenPosX, screenPosY, screenWidth, screenHeight, SWP_FRAMECHANGED | SWP_NOZORDER);
	}
}

Projector::~Projector()
{
	cv::destroyWindow(windowName);
}

void Projector::displayImage(cv::Mat image)
{
	cv::imshow(windowName, image);
	cv::waitKey(1); //time to display image
}
