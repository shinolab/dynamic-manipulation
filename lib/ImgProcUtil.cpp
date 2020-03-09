#include "ImgProcUtil.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void imgProcUtil::get_center_thr(const cv::Mat& img, const cv::Scalar lowerbound, const cv::Scalar upperbound, cv::Point2f& center) {
	cv::Mat imgBin;
	cv::inRange(img, lowerbound, upperbound, imgBin);
	auto mu = cv::moments(imgBin, true);
	center = cv::Point2f((mu.m10 / mu.m00), (mu.m01 / mu.m00));
#ifdef _DEBUG
	cv::Mat imgDebug;
	cv::cvtColor(imgBin, imgDebug, cv::COLOR_GRAY2BGR);
	cv::circle(imgDebug, center, 3, cv::Scalar(0, 0, 255), -1);
	cv::imshow("center", imgDebug);
	cv::waitKey(5);
#endif
}

void imgProcUtil::get_center_thr(const cv::Mat& img, const cv::Scalar lowerbound, const cv::Scalar upperbound, cv::Point2f& center, cv::ColorConversionCodes code) {
	cv::Mat img_to_process;
	cv::cvtColor(img, img_to_process, code);
	imgProcUtil::get_center_thr(img_to_process, lowerbound, upperbound, center);
}
