#include "KinectUtility.hpp"
#include "ImgProcUtil.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <fstream>

int main(int argc, char** argv) {
	std::ofstream ofs("output.csv");
	int period = 60000;
	int loopPeriod = 30;
	cv::Mat img_target = cv::imread("target.png");
	std::vector<cv::Mat> imgs_target = { img_target };
	int lowerbound = 10;
	int upperbound = 255;
	imgProc::hue_backproject_extractor extractor(imgs_target, lowerbound, upperbound);

	KinectUtility::KinectColorManager colorManager;
	std::vector<BYTE> colorBuffer;

	int initTime = timeGetTime();
	while (timeGetTime() - initTime < period) {
		int loopInit = timeGetTime();
		colorManager.acquireBuffer(colorBuffer);
		cv::Mat img(colorManager.height(), colorManager.width(), CV_8UC3, &colorBuffer[0]);
		cv::Point2f point = extractor.extract_center(img);
		cv::circle(img, point, 1, cv::Scalar(0, 0, 255), -1);
		cv::imshow("image", img);
		cv::waitKey(3);
		ofs << loopInit << ", " << point.x << ", " << point.y << ", " << std::endl;
		int waitTime = loopPeriod - (timeGetTime() - loopInit);
		timeBeginPeriod(1);
		Sleep(std::max(waitTime, 0));
		timeEndPeriod(1);
	}
	ofs.close();
}