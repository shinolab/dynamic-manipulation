#include "DrawUtil.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#define NOMINMAX
#include <Windows.h>
#include <vector>

std::vector<cv::Point2f> DrawUtil::drawCircleGrid(cv::Mat &dst, cv::Point upperLeftCorner, cv::Size patternSize, int gridStepX, int gridStepY, int radius)
{
	int r = radius;
	if (r < 0)
	{
		r = std::min(gridStepX, gridStepY) / 6;
	}
	std::vector<cv::Point2f> centers;
	for (int iY = 0; iY < patternSize.height; iY++)
	{
		for (int iX = 0; iX < patternSize.width; iX++)
		{
			cv::Point2f center = cv::Point2f(iX * gridStepX + upperLeftCorner.x, iY * gridStepY + upperLeftCorner.y);
			cv::circle(dst, center, r, cv::Scalar(0, 0, 0), CV_FILLED);
			//cv::circle(dst, center, 3, cv::Scalar(0, 0, 0), CV_FILLED);
			centers.push_back(center);
		}
	}
	return centers;
}

void DrawUtil::imshowPopUp(const char* windowName, cv::Mat image, int posX, int posY) {
	cv::namedWindow(windowName, CV_WINDOW_NORMAL);

	HWND windowHandle = ::FindWindowA(NULL, windowName);

	if (windowHandle != NULL)
	{
		SetWindowLongPtr(windowHandle, GWL_STYLE, WS_POPUP);
		SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

		ShowWindow(windowHandle, SW_MAXIMIZE);
		cv::setWindowProperty(windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

		SetWindowPos(windowHandle, NULL,
			posX, posY, image.cols, image.rows, SWP_FRAMECHANGED | SWP_NOZORDER);
	}
	cv::imshow(windowName, image);
	cv::waitKey(20);
}