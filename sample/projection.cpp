#define SUBDISPLAY_WIDTH 1024
#define SUBDISPLAY_HEIGHT 768
#define MAINDISPLAY_WIDTH 1366
#define MAINDISPLAY_HEIGHT 768

#include "odcs.hpp"
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

void imshowPopUp(const char* windowName, cv::Mat image, int posX, int posY)
{
	cv::namedWindow(windowName, CV_WINDOW_NORMAL);

	HWND windowHandle = ::FindWindowA(NULL, windowName);

	if (windowHandle != NULL)
	{
		SetWindowLongPtrA(windowHandle, GWL_STYLE, WS_POPUP);
		SetWindowLongPtrA(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

		ShowWindow(windowHandle, SW_MAXIMIZE);
		cv::setWindowProperty(windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

		SetWindowPos(windowHandle, NULL,
			posX, posY, image.cols, image.rows, SWP_FRAMECHANGED | SWP_NOZORDER);
	}
	cv::imshow(windowName, image);
	cv::waitKey(20);
}

int main()
{
	ods ods;
	ods.Initialize();
	FloatingObjectPtr objPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1350)));
	while (1)
	{
		ods.DeterminePositionByDepthWithROI(objPtr);
		Eigen::Vector3f objPos = ods.getAffineKinect2Global().inverse()*objPtr->getPosition();
		cv::Point3f objectPosition(objPos.x(), objPos.y(), objPos.z() - 75);
		std::vector<cv::Point3f> imagePoints3d = { objectPosition };
		//here comes conversion from object position to object points
		std::vector<cv::Point2f> imagePoints2d;
		cv::Mat internalParam = (cv::Mat_<float>(3, 3) <<
			770, 0, SUBDISPLAY_WIDTH / 2,
			0, 760, SUBDISPLAY_HEIGHT + 105,
			0, 0, 1);
		cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.02374, -0.002134, -3.1495);
		cv::Mat tvec = (cv::Mat_<float>(3, 1) << 57.4, -617.5, 189.95);

		cv::Point2f centerImage;
		cv::projectPoints(imagePoints3d, rvec, tvec, internalParam, cv::Mat(), imagePoints2d);
		cv::Mat src = cv::imread("img/saffron.jpg");
		cv::Mat dst(SUBDISPLAY_HEIGHT, SUBDISPLAY_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
		float scale = 0.3;
		cv::Mat affine = (cv::Mat_<float>(2, 3) <<
			scale, 0, (imagePoints2d[0].x - 0.5 * scale * src.cols),
			0, scale, (imagePoints2d[0].y - 0.5 * scale * src.rows));
		cv::warpAffine(src, dst, affine, dst.size());
		/*
		std::vector<cv::Point> originalImageCorners = {cv::Point(0,0), cv::Point(img0.cols, 0), cv::Point(0, img0.rows), cv::Point(img0.cols, img0.rows)};
		cv::Mat perspectiveMat = cv::getPerspectiveTransform(originalImageCorners, imagePoints2d);
		cv::Mat img1;
		cv::perspectiveTransform(img0, img1, perspectiveMat);
		*/
		imshowPopUp("FULL2", dst, MAINDISPLAY_WIDTH, 0);
		auto key = cv::waitKey(1);
		if(key == 'q')
		{
			break;
		}
	}

	return 0;
}